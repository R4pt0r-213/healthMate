from pathlib import Path
import sys

import cv2
from ultralytics import YOLO


# Chemins du projet
PROJECT_ROOT = Path(__file__).resolve().parent.parent
MODEL_PATH = PROJECT_ROOT / "ia" / "model" / "best.pt"

# Permet d'importer camera.plateau lors d'un lancement direct
sys.path.insert(0, str(PROJECT_ROOT))

from camera.plateau import analyse_position


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

        # Détection des gobelets
        resultats = model.predict(
            source=frame,
            conf=0.5,
            verbose=False,
        )

        resultat = resultats[0]

        for box in resultat.boxes:
            # Coordonnées du rectangle de détection
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)

            # Classe détectée par YOLO
            class_id = int(box.cls[0].item())
            confiance = float(box.conf[0].item())
            etat = model.names[class_id]

            # Centre du gobelet dans l'image
            centre_x = int((x1 + x2) / 2)
            centre_y = int((y1 + y2) / 2)

            try:
                position = analyse_position(centre_x, centre_y)

                x_cm = float(position["x_cm"])
                y_cm = float(position["y_cm"])

                dx = float(position["dx"])
                dy = float(position["dy"])
                distance = float(position["distance_robot"])
                angle_plateau = float(position["angle_plateau"])
                angle_servo = float(position["angle_servo"])

                zone = position["zone"]
                medicament = position["medicament"]

                texte = (
                    f"{etat} | zone {zone} | type {medicament} "
                    f"| {confiance:.0%}"
                )

                coordonnees = (
                    f"relatif: dx={dx:.1f}, dy={dy:.1f}, "
                    f"d={distance:.1f} cm, servo={angle_servo:.1f} deg"
                )

                # Vert si le gobelet est sur le plateau
                if (
                    zone != "Hors plateau"
                    and medicament != "Hors plateau"
                ):
                    couleur = (0, 255, 0)
                else:
                    couleur = (0, 165, 255)

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

            # Point central utilisé pour calculer la position
            cv2.circle(
                frame,
                (centre_x, centre_y),
                5,
                (255, 0, 255),
                -1,
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