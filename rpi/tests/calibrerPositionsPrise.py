"""Envoie à l'Arduino la distance caméra du gobelet à calibrer."""

from pathlib import Path
import sys
import time

import cv2
import serial
from ultralytics import YOLO


ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from camera.plateau import (
    analyse_position,
    construire_dictionnaire_marqueurs,
    obtenir_transformation_plateau,
)
from ia.config import CONFIANCE_MIN_GOBELET
from robot.communication import BAUDRATE, PORT


MODEL_PATH = ROOT / "ia" / "model" / "best.pt"
WINDOW_NAME = "Calibration des positions de prise"
INTERVALLE_ENVOI_DISTANCE_S = 0.25


def lire_lignes_arduino(connexion, tampon):
    nombre = connexion.in_waiting
    if nombre > 0:
        tampon.extend(connexion.read(nombre))

    lignes = []
    while True:
        fin = tampon.find(b"\n")
        if fin < 0:
            break
        ligne = tampon[:fin]
        del tampon[:fin + 1]
        lignes.append(
            ligne.decode("utf-8", errors="replace").strip()
        )
    return lignes


def main():
    model = YOLO(str(MODEL_PATH))
    dictionnaire = cv2.aruco.getPredefinedDictionary(
        cv2.aruco.DICT_4X4_50
    )
    parametres = cv2.aruco.DetectorParameters()
    parametres.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    detecteur = cv2.aruco.ArucoDetector(
        dictionnaire,
        parametres,
    )

    camera = cv2.VideoCapture(0)
    if not camera.isOpened():
        raise RuntimeError("Impossible d'ouvrir la caméra")

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, 1100, 700)
    cv2.moveWindow(WINDOW_NAME, 50, 50)

    connexion = serial.Serial(PORT, BAUDRATE, timeout=0)
    tampon = bytearray()
    dernier_envoi = 0.0

    print("Place un seul gobelet dans la zone de prise.")
    print("Régle les servos avec A0, A1, A2, A3 et A4.")
    print("Bouton court : position intermédiaire.")
    print("Bouton maintenu 4 secondes : position finale.")
    print("Q ou Échap : quitter.")

    try:
        while True:
            for ligne in lire_lignes_arduino(connexion, tampon):
                if ligne:
                    print("Arduino >", ligne)

            ok, frame = camera.read()
            if not ok:
                raise RuntimeError("Impossible de lire la caméra")

            coins, ids, _ = detecteur.detectMarkers(frame)
            markers = construire_dictionnaire_marqueurs(
                coins,
                ids,
            )

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

            candidats = []

            if repere_disponible:
                for box in resultats[0].boxes:
                    x1, y1, x2, y2 = (
                        box.xyxy[0].cpu().numpy().astype(int)
                    )
                    confiance = float(box.conf[0].item())
                    classe = int(box.cls[0].item())
                    etat = model.names[classe]
                    point_x = int((x1 + x2) / 2)
                    point_y = int((y1 + y2) / 2)

                    try:
                        position = analyse_position(
                            point_x,
                            point_y,
                            markers,
                        )
                    except (RuntimeError, ValueError):
                        continue

                    candidats.append({
                        "distance": float(
                            position["distance_robot"]
                        ),
                        "confiance": confiance,
                        "etat": etat,
                        "boite": (x1, y1, x2, y2),
                        "point": (point_x, point_y),
                    })

            selection = max(
                candidats,
                key=lambda objet: objet["confiance"],
                default=None,
            )

            if selection is None:
                texte = "AUCUN GOBELET MESURABLE"
                couleur = (0, 0, 255)
            else:
                distance = selection["distance"]
                texte = (
                    f"{selection['etat']} | "
                    f"distance robot={distance:.2f} cm | "
                    f"confiance={selection['confiance']:.0%}"
                )
                couleur = (0, 255, 0)

                x1, y1, x2, y2 = selection["boite"]
                point_x, point_y = selection["point"]
                cv2.rectangle(
                    frame,
                    (x1, y1),
                    (x2, y2),
                    couleur,
                    3,
                )
                cv2.circle(
                    frame,
                    (point_x, point_y),
                    7,
                    (255, 0, 255),
                    -1,
                )

                if (
                    time.time() - dernier_envoi
                    >= INTERVALLE_ENVOI_DISTANCE_S
                ):
                    message = f"D;{distance:.2f}\n"
                    connexion.write(message.encode("utf-8"))
                    connexion.flush()
                    dernier_envoi = time.time()

            cv2.putText(
                frame,
                texte,
                (15, 32),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                couleur,
                2,
            )
            cv2.imshow(WINDOW_NAME, frame)

            touche = cv2.waitKey(1) & 0xFF
            if touche in (ord("q"), 27):
                break

    except KeyboardInterrupt:
        print("\nArrêt demandé")
    finally:
        connexion.close()
        camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
