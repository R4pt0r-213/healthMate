from pathlib import Path
import sys
import time

import cv2
from ultralytics import YOLO


PROJECT_ROOT = Path(__file__).resolve().parent.parent
MODEL_PATH = PROJECT_ROOT / "ia" / "model" / "best.pt"
WINDOW_NAME = "Test IA + Arduino"

sys.path.insert(0, str(PROJECT_ROOT))

from camera.plateau import (
    DEPOT_X_CM,
    DEPOT_Y_CM,
    DISTANCE_MAX_PRISE_CM,
    DISTANCE_MIN_PRISE_CM,
    ROBOT_MARKER_ID,
    analyse_position,
    construire_dictionnaire_marqueurs,
    get_robot_position,
    obtenir_marqueur,
    obtenir_transformation_plateau,
    repere_plateau_visible,
)
from robot.communication import Robot
from ia.config import CONFIANCE_MIN_GOBELET


def main():
    model = YOLO(str(MODEL_PATH))
    dictionnaire = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    detecteur = cv2.aruco.ArucoDetector(
        dictionnaire,
        cv2.aruco.DetectorParameters(),
    )
    camera = cv2.VideoCapture(0)

    if not camera.isOpened():
        raise RuntimeError("Impossible d'ouvrir la caméra")

    # Force une fenêtre visible à une position connue sur macOS.
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, 1100, 700)
    cv2.moveWindow(WINDOW_NAME, 50, 50)

    # Affiche immédiatement une première image. Sans cela, macOS peut ne pas
    # montrer la fenêtre pendant la connexion Arduino et la première inférence.
    success, premiere_image = camera.read()

    if not success:
        camera.release()
        cv2.destroyAllWindows()
        raise RuntimeError("Impossible de lire la caméra")

    cv2.imshow(WINDOW_NAME, premiere_image)
    cv2.waitKey(100)

    robot = Robot()
    derniere_commande = 0
    commande_en_cours = False
    debut_commande = 0

    print("S : envoyer la position détectée à l'Arduino")
    print("Q ou Échap : quitter")

    position_selectionnee = None

    try:
        while True:
            # Lit les messages Arduino sans bloquer la boucle caméra.
            ligne_arduino = robot.lire()

            if ligne_arduino != "":
                print("Arduino >", ligne_arduino)

                if ligne_arduino == "TERMINE":
                    commande_en_cours = False
                    derniere_commande = time.time()
                    print("Mouvement terminé")

            # Évite de rester bloqué si l'Arduino ne répond jamais TERMINE.
            if commande_en_cours and time.time() - debut_commande > 60:
                commande_en_cours = False
                derniere_commande = time.time()
                print("L'Arduino n'a pas confirmé la fin du mouvement")

            success, frame = camera.read()

            if not success:
                raise RuntimeError("Impossible de lire la caméra")

            coins, ids, _ = detecteur.detectMarkers(frame)
            markers = construire_dictionnaire_marqueurs(coins, ids)
            repere_visible = repere_plateau_visible(markers)
            repere_disponible = False
            angle_depot = None
            distance_depot = None

            try:
                transformation = obtenir_transformation_plateau(markers)
                marqueur_robot = obtenir_marqueur(
                    ROBOT_MARKER_ID,
                    markers,
                )
                repere_disponible = True
                depot = get_robot_position(
                    DEPOT_X_CM,
                    DEPOT_Y_CM,
                    marqueur_robot,
                    transformation,
                )
                if depot["angle_servo"] is not None:
                    angle_depot = float(depot["angle_servo"])
                    distance_depot = float(depot["distance_cm"])
            except (RuntimeError, ValueError):
                pass

            resultats = model.predict(
                source=frame,
                conf=CONFIANCE_MIN_GOBELET,
                verbose=False,
            )

            # On reconstruit les candidats à chaque image.
            position_selectionnee = None
            gobelets_vides = []
            gobelets_pleins = []

            for box in resultats[0].boxes:
                x1, y1, x2, y2 = (
                    box.xyxy[0]
                    .cpu()
                    .numpy()
                    .astype(int)
                )

                class_id = int(box.cls[0].item())
                confiance = float(box.conf[0].item())
                etat = model.names[class_id]

                # Point de contact du gobelet avec le plateau
                point_x = int((x1 + x2) / 2)
                point_y = int(y2)

                if not repere_disponible or angle_depot is None:
                    continue

                position = analyse_position(point_x, point_y, markers)

                distance = float(position["distance_robot"])
                angle_plateau = float(position["angle_plateau"])
                angle_cible = float(position["angle_cible_pince"])
                angle = position["angle_servo"]
                zone = position["zone"]
                medicament = position["medicament"]

                candidat = (
                    angle is not None
                    and DISTANCE_MIN_PRISE_CM <= distance <= DISTANCE_MAX_PRISE_CM
                    and medicament in ("A", "B")
                )

                donnees = None

                if candidat:
                    donnees = {
                        "angle": angle,
                        "distance": distance,
                        "etat": etat,
                        "zone": zone,
                        "medicament": medicament,
                        "confiance": confiance,
                    }

                if etat == "cup_empty" and zone == "W" and donnees:
                    gobelets_vides.append(donnees)
                    couleur = (0, 255, 0)
                elif etat == "cup_full" and zone == "S" and donnees:
                    gobelets_pleins.append(donnees)
                    couleur = (255, 128, 0)
                else:
                    couleur = (0, 165, 255)

                if distance < DISTANCE_MIN_PRISE_CM:
                    texte = (
                        f"{etat} | zone={zone} | med={medicament} | "
                        f"TROP PROCHE (< {DISTANCE_MIN_PRISE_CM:.0f} cm) | "
                        f"d={distance:.1f} cm"
                    )
                elif distance > DISTANCE_MAX_PRISE_CM:
                    texte = (
                        f"{etat} | zone={zone} | med={medicament} | "
                        f"TROP LOIN (> {DISTANCE_MAX_PRISE_CM:.0f} cm) | "
                        f"d={distance:.1f} cm"
                    )
                elif angle is None:
                    texte = (
                        f"{etat} | zone={zone} | cible={angle_cible:.1f} | "
                        f"med={medicament} | servo=HORS PORTEE | "
                        f"d={distance:.1f} cm"
                    )
                else:
                    texte = (
                        f"{etat} | zone={zone} | cible={angle_cible:.1f} | "
                        f"med={medicament} | servo={angle:.1f} | "
                        f"d={distance:.1f} cm"
                    )

                cv2.rectangle(
                    frame,
                    (x1, y1),
                    (x2, y2),
                    couleur,
                    2,
                )

                cv2.circle(
                    frame,
                    (point_x, point_y),
                    7,
                    (255, 0, 255),
                    -1,
                )
                cv2.drawMarker(
                    frame,
                    (point_x, point_y),
                    (255, 255, 255),
                    cv2.MARKER_CROSS,
                    22,
                    2,
                )
                cv2.putText(
                    frame,
                    "POSITION",
                    (point_x + 10, point_y - 8),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.45,
                    (255, 0, 255),
                    2,
                )

                cv2.putText(
                    frame,
                    texte,
                    (x1, max(25, y1 - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    couleur,
                    2,
                )

            # Choisit le vide le plus fiable qui possède un plein correspondant.
            gobelets_vides.sort(
                key=lambda objet: objet["confiance"],
                reverse=True,
            )

            for vide in gobelets_vides:
                correspondants = [
                    plein
                    for plein in gobelets_pleins
                    if plein["medicament"] == vide["medicament"]
                ]

                if correspondants:
                    plein = max(
                        correspondants,
                        key=lambda objet: objet["confiance"],
                    )
                    position_selectionnee = {
                        "vide": vide,
                        "plein": plein,
                    }
                    break

            if commande_en_cours:
                instruction = "ROBOT EN MOUVEMENT..."
                couleur_instruction = (0, 165, 255)
            elif not repere_disponible:
                instruction = "INITIALISATION : MONTRER 0, 1, 2, 3"
                couleur_instruction = (0, 0, 255)
            elif angle_depot is None:
                instruction = "ANGLE DU DEPOT IMPOSSIBLE"
                couleur_instruction = (0, 0, 255)
            elif position_selectionnee is not None:
                medicament = position_selectionnee["vide"]["medicament"]
                instruction = f"S : REMPLACER LE MEDICAMENT {medicament}"
                couleur_instruction = (0, 255, 0)
            elif gobelets_vides:
                instruction = "AUCUN GOBELET PLEIN CORRESPONDANT DANS S"
                couleur_instruction = (0, 0, 255)
            else:
                instruction = "AUCUN GOBELET VIDE VALIDE DANS W"
                couleur_instruction = (0, 0, 255)

            if repere_disponible and not repere_visible and not commande_en_cours:
                instruction += " | REPERE MEMORISE"

            cv2.putText(
                frame,
                instruction,
                (15, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                couleur_instruction,
                2,
            )

            cv2.imshow(WINDOW_NAME, frame)

            touche = cv2.waitKey(1) & 0xFF

            if touche == ord("q") or touche == 27:
                break

            if (
                touche == ord("s")
                and position_selectionnee is not None
                and not commande_en_cours
                and time.time() - derniere_commande > 2
            ):
                vide = position_selectionnee["vide"]
                plein = position_selectionnee["plein"]

                print(
                    f"Remplacement {vide['medicament']} : "
                    f"vide -> D, puis plein -> ancienne place du vide"
                )

                robot.envoyer_remplacement(
                    vide["angle"],
                    vide["distance"],
                    angle_depot,
                    distance_depot,
                    plein["angle"],
                    plein["distance"],
                    vide["angle"],
                    vide["distance"],
                )
                commande_en_cours = True
                debut_commande = time.time()

    except KeyboardInterrupt:
        print("\nArrêt demandé")

    finally:
        camera.release()
        cv2.destroyAllWindows()
        robot.fermer()


if __name__ == "__main__":
    main()
