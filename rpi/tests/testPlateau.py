from pathlib import Path
import sys

import cv2
from ultralytics import YOLO


# Chemins du projet
PROJECT_ROOT = Path(__file__).resolve().parent.parent
MODEL_PATH = PROJECT_ROOT / "ia" / "model" / "best.pt"

# Permet d'importer camera.plateau lors d'un lancement direct
sys.path.insert(0, str(PROJECT_ROOT))

from camera.plateau import (
    DISTANCE_MAX_PRISE_CM,
    DISTANCE_MIN_PRISE_CM,
    analyse_position,
    construire_dictionnaire_marqueurs,
    obtenir_transformation_plateau,
    repere_plateau_visible,
)
from ia.config import CONFIANCE_MIN_GOBELET
from camera.visualisation import dessiner_portee_robot

dictionnaire = cv2.aruco.getPredefinedDictionary(
    cv2.aruco.DICT_4X4_50
)

detecteur = cv2.aruco.ArucoDetector(
    dictionnaire,
    cv2.aruco.DetectorParameters(),
)

def main():
    # Chargement du modèle de détection
    model = YOLO(str(MODEL_PATH))

    # Ouverture de la caméra principale
    camera = cv2.VideoCapture(0)

    if not camera.isOpened():
        raise RuntimeError("Impossible d'ouvrir la caméra")

    print("Caméra ouverte.")
    print("Appuie sur Q ou Échap pour quitter.")

    while True:
        success, frame = camera.read()

        if not success:
            print("Impossible de récupérer une image")
            break

        coins, ids, _ = detecteur.detectMarkers(frame)
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(
                frame,
                coins,
                ids,
            )

        markers = construire_dictionnaire_marqueurs(coins, ids)
        repere_visible = repere_plateau_visible(markers)

        try:
            obtenir_transformation_plateau(markers)
            repere_disponible = True
        except RuntimeError:
            repere_disponible = False

        # Détection des gobelets sur l'image originale
        resultats = model.predict(
            source=frame.copy(),
            conf=CONFIANCE_MIN_GOBELET,
            verbose=False,
        )

        resultat = resultats[0]

        # On dessine la portée seulement après la détection YOLO
        if repere_disponible:
            dessiner_portee_robot(frame, markers)

        resultat = resultats[0]

        if not repere_disponible:
            cv2.putText(
                frame,
                "INITIALISATION : MONTRER 0, 1, 2, 3",
                (15, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2,
            )

        for box in resultat.boxes:
            # Coordonnées du rectangle de détection
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)

            # Classe détectée par YOLO
            class_id = int(box.cls[0].item())
            confiance = float(box.conf[0].item())
            etat = model.names[class_id]

            # Point de contact du gobelet avec le plateau
            centre_x = int((x1 + x2) / 2)
            centre_y = int(y2)

            try:
                if not repere_disponible:
                    continue

                position = analyse_position(
                    centre_x,
                    centre_y,
                    markers,
                )

                x_cm = float(position["x_cm"])
                y_cm = float(position["y_cm"])

                dx = float(position["dx"])
                dy = float(position["dy"])
                distance = float(position["distance_robot"])
                angle_plateau = float(position["angle_plateau"])
                angle_cible = float(position["angle_cible_pince"])
                angle_servo = position["angle_servo"]
                zone = position["zone"]
                medicament = position["medicament"]

                texte = (
                    f"{etat} | zone={zone} | medicament={medicament} | "
                    f"confiance={confiance:.0%}"
                )

                if distance < DISTANCE_MIN_PRISE_CM:
                    coordonnees = (
                        f"TROP PROCHE (< {DISTANCE_MIN_PRISE_CM:.0f} cm), "
                        f"d={distance:.1f} cm"
                    )
                    couleur = (0, 0, 255)
                elif distance > DISTANCE_MAX_PRISE_CM:
                    coordonnees = (
                        f"TROP LOIN (> {DISTANCE_MAX_PRISE_CM:.0f} cm), "
                        f"d={distance:.1f} cm"
                    )
                    couleur = (0, 0, 255)
                elif angle_servo is None:
                    coordonnees = (
                        f"pince={angle_plateau:.1f} deg, "
                        f"servo=HORS PORTEE, d={distance:.1f} cm"
                    )
                    couleur = (0, 165, 255)
                else:
                    coordonnees = (
                        f"cible={angle_cible:.1f} deg, "
                        f"servo={angle_servo:.1f} deg, "
                        f"d={distance:.1f} cm"
                    )
                    couleur = (0, 255, 0)

            except Exception as erreur:
                texte = f"{etat} | position inconnue"
                coordonnees = str(erreur)
                couleur = (0, 0, 255)

            # Rectangle autour du gobelet
            cv2.rectangle(
                frame,
                (x1, y1),
                (x2, y2),
                couleur,
                2,
            )

            # Cible utilisée pour calculer la position : milieu du bord bas.
            cv2.circle(
                frame,
                (centre_x, centre_y),
                7,
                (255, 0, 255),
                -1,
            )
            cv2.drawMarker(
                frame,
                (centre_x, centre_y),
                (255, 255, 255),
                cv2.MARKER_CROSS,
                22,
                2,
            )
            cv2.putText(
                frame,
                "POSITION",
                (centre_x + 10, centre_y - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (255, 0, 255),
                2,
            )

            # Fond du texte
            largeur_texte = max(400, x2 - x1)

            cv2.rectangle(
                frame,
                (x1, max(0, y1 - 55)),
                (x1 + largeur_texte, y1),
                couleur,
                -1,
            )

            # Zone, type et état
            cv2.putText(
                frame,
                texte,
                (x1 + 5, max(20, y1 - 32)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 0, 0),
                2,
            )

            # Coordonnées en centimètres
            cv2.putText(
                frame,
                coordonnees,
                (x1 + 5, max(40, y1 - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 0, 0),
                1,
            )

        # Instructions
        if repere_disponible and not repere_visible:
            cv2.putText(
                frame,
                "REPERE MEMORISE - MARQUEUR MANQUANT",
                (15, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 165, 255),
                2,
            )

        cv2.putText(
            frame,
            "Q ou Echap : quitter",
            (15, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
        )

        cv2.imshow("Test visuel du plateau", frame)

        touche = cv2.waitKey(1) & 0xFF

        if touche == ord("q") or touche == 27:
            break

    camera.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
