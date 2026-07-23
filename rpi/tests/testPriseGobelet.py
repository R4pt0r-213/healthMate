from pathlib import Path
import sys
import time

import cv2
from ultralytics import YOLO


ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from camera.plateau import (
    DISTANCE_MAX_PRISE_CM,
    DISTANCE_MIN_PRISE_CM,
    analyse_position,
    construire_dictionnaire_marqueurs,
    obtenir_transformation_plateau,
    repere_plateau_visible,
)
from robot.communication import Robot
from ia.config import CONFIANCE_MIN_GOBELET
from camera.visualisation import dessiner_portee_robot


MODEL_PATH = ROOT / "ia" / "model" / "best.pt"
WINDOW_NAME = "Test prise d'un gobelet vide"


def main():
    dictionnaire = cv2.aruco.getPredefinedDictionary(
        cv2.aruco.DICT_4X4_50
    )

    detecteur = cv2.aruco.ArucoDetector(
        dictionnaire,
        cv2.aruco.DetectorParameters(),
    )
    model = YOLO(str(MODEL_PATH))
    camera = cv2.VideoCapture(0)

    if not camera.isOpened():
        raise RuntimeError("Impossible d'ouvrir la caméra")

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, 1100, 700)
    cv2.moveWindow(WINDOW_NAME, 50, 50)

    robot = None
    commande_en_cours = False
    debut_commande = 0.0

    print("S : prendre le gobelet vide sélectionné")
    print("Q ou Échap : quitter")

    try:
        # La connexion série est faite après l'ouverture de la fenêtre.
        robot = Robot()

        while True:
            if robot is not None:
                ligne = robot.lire()
                if ligne:
                    print("Arduino >", ligne)
                    if ligne in ("TERMINE", "ERREUR"):
                        commande_en_cours = False

            if commande_en_cours and time.time() - debut_commande > 60:
                print("Délai Arduino dépassé")
                commande_en_cours = False

            ok, frame = camera.read()
            if not ok:
                raise RuntimeError("Impossible de lire la caméra")

            coins, ids, _ = detecteur.detectMarkers(frame)
            markers = construire_dictionnaire_marqueurs(coins, ids)
            repere_visible = repere_plateau_visible(markers)

            try:
                obtenir_transformation_plateau(markers)
                repere_disponible = True
            except RuntimeError:
                repere_disponible = False

            resultats = model.predict(
                source=frame,
                conf=CONFIANCE_MIN_GOBELET,
                verbose=False,
            )
            
            if repere_disponible:
                dessiner_portee_robot(frame, markers)


            candidats = []

            for box in resultats[0].boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                classe = int(box.cls[0].item())
                confiance = float(box.conf[0].item())
                etat = model.names[classe]

                point_x = int((x1 + x2) / 2)
                point_y = int(y2)
                if not repere_disponible:
                    continue

                position = analyse_position(
                    point_x,
                    point_y,
                    markers,
                )
                
                angle = position["angle_servo"]
                distance = float(position["distance_robot"])
                zone = position["zone"]

                atteignable = (
                    etat == "cup_empty"
                    and zone == "W"
                    and angle is not None
                    and DISTANCE_MIN_PRISE_CM <= distance <= DISTANCE_MAX_PRISE_CM
                )

                couleur = (0, 255, 0) if atteignable else (0, 165, 255)

                if atteignable:
                    candidats.append({
                        "angle": float(angle),
                        "distance": distance,
                        "confiance": confiance,
                        "boite": (x1, y1, x2, y2),
                    })

                if distance < DISTANCE_MIN_PRISE_CM:
                    servo_texte = "trop proche"
                elif distance > DISTANCE_MAX_PRISE_CM:
                    servo_texte = "trop loin"
                else:
                    servo_texte = "hors portee" if angle is None else f"{angle:.1f} deg"
                texte = (
                    f"{etat} | zone={zone} | servo={servo_texte} | "
                    f"d={distance:.1f} cm"
                )
                cv2.rectangle(frame, (x1, y1), (x2, y2), couleur, 2)
                cv2.circle(frame, (point_x, point_y), 7, (255, 0, 255), -1)
                cv2.drawMarker(frame, (point_x, point_y), (255, 255, 255),
                               cv2.MARKER_CROSS, 22, 2)
                cv2.putText(frame, "POSITION", (point_x + 10, point_y - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 0, 255), 2)
                cv2.putText(
                    frame, texte, (x1, max(25, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, couleur, 2,
                )

            selection = max(
                candidats,
                key=lambda objet: objet["confiance"],
                default=None,
            )

            if commande_en_cours:
                instruction = "ROBOT EN MOUVEMENT - NE PAS APPROCHER"
                couleur = (0, 165, 255)
            elif not repere_disponible:
                instruction = "INITIALISATION : MONTRER 0, 1, 2, 3"
                couleur = (0, 0, 255)
            elif selection is None:
                instruction = "AUCUN GOBELET VIDE ATTEIGNABLE DANS W"
                couleur = (0, 0, 255)
            else:
                instruction = (
                    f"S : PRENDRE | servo={selection['angle']:.1f} | "
                    f"d={selection['distance']:.1f} cm"
                )
                couleur = (0, 255, 0)
                x1, y1, x2, y2 = selection["boite"]
                cv2.rectangle(frame, (x1 - 4, y1 - 4), (x2 + 4, y2 + 4), (255, 0, 255), 3)

            if repere_disponible and not repere_visible and not commande_en_cours:
                instruction += " | REPERE MEMORISE"

            cv2.putText(
                frame, instruction, (15, 32),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, couleur, 2,
            )
            cv2.imshow(WINDOW_NAME, frame)

            touche = cv2.waitKey(1) & 0xFF
            if touche in (ord("q"), 27):
                break

            if touche == ord("s") and selection is not None and not commande_en_cours:
                print(
                    f"Prise envoyée : servo={selection['angle']:.1f}°, "
                    f"distance={selection['distance']:.1f} cm"
                )
                robot.envoyer(selection["angle"], selection["distance"])
                commande_en_cours = True
                debut_commande = time.time()

    except KeyboardInterrupt:
        print("\nArrêt demandé")
    finally:
        camera.release()
        cv2.destroyAllWindows()
        if robot is not None:
            robot.fermer()


if __name__ == "__main__":
    main()
