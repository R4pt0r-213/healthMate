from pathlib import Path
import math
import sys

import cv2
from ultralytics import YOLO


PROJECT_ROOT = Path(__file__).resolve().parent.parent
MODEL_PATH = PROJECT_ROOT / "ia" / "model" / "best.pt"
WINDOW_NAME = "Alignement bras et gobelet"
ARUCO_BRAS_ID = 4

sys.path.insert(0, str(PROJECT_ROOT))

from camera.plateau import OFFSET_PINCE_MARQUEUR_DEG, pixel_to_cm
from ia.config import CONFIANCE_MIN_GOBELET


def erreur_angulaire(angle_cible, angle_bras):
    """Renvoie l'écart signé le plus court, entre -180° et +180°."""
    return ((angle_cible - angle_bras + 180) % 360) - 180


def calculer_angle(centre_robot, point):
    """Calcule l'angle d'un point autour du centre cliqué du robot."""
    robot_x, robot_y = pixel_to_cm(*centre_robot)
    point_x, point_y = pixel_to_cm(*point)
    angle = math.degrees(
        math.atan2(point_y - robot_y, point_x - robot_x)
    )
    return angle % 360


def calculer_orientation_marqueur(marker_coins):
    """Mesure la rotation du bord supérieur du marqueur sur le plateau."""
    coin_0 = marker_coins[0][0]
    coin_1 = marker_coins[0][1]
    x_0, y_0 = pixel_to_cm(float(coin_0[0]), float(coin_0[1]))
    x_1, y_1 = pixel_to_cm(float(coin_1[0]), float(coin_1[1]))
    angle = math.degrees(math.atan2(y_1 - y_0, x_1 - x_0))
    return angle % 360


def main():
    model = YOLO(str(MODEL_PATH))

    dictionnaire = cv2.aruco.getPredefinedDictionary(
        cv2.aruco.DICT_4X4_50
    )
    parametres = cv2.aruco.DetectorParameters()
    parametres.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    detecteur_aruco = cv2.aruco.ArucoDetector(
        dictionnaire,
        parametres,
    )

    camera = cv2.VideoCapture(0)

    if not camera.isOpened():
        raise RuntimeError("Impossible d'ouvrir la caméra")

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, 1100, 700)
    cv2.moveWindow(WINDOW_NAME, 50, 50)

    centre_robot = {"pixel": None}

    def choisir_centre_robot(event, x, y, flags, parametre):
        if event == cv2.EVENT_LBUTTONDOWN:
            centre_robot["pixel"] = (x, y)
            centre_x_cm, centre_y_cm = pixel_to_cm(x, y)
            print(
                f"Centre du robot : pixel=({x}, {y}), "
                f"cm=({centre_x_cm:.2f}, {centre_y_cm:.2f})"
            )

    cv2.setMouseCallback(WINDOW_NAME, choisir_centre_robot)

    print("Clique sur l'axe de rotation de la base du robot")
    print("Q ou Échap : quitter")

    try:
        while True:
            success, frame = camera.read()

            if not success:
                raise RuntimeError("Impossible de lire la caméra")

            angle_marqueur = None
            angle_pince = None
            centre_marqueur = None

            coins, identifiants, _ = detecteur_aruco.detectMarkers(frame)

            if identifiants is not None:
                cv2.aruco.drawDetectedMarkers(
                    frame,
                    coins,
                    identifiants,
                )

                for marker_id, marker_coins in zip(
                    identifiants.flatten(),
                    coins,
                ):
                    if int(marker_id) != ARUCO_BRAS_ID:
                        continue

                    centre = marker_coins[0].mean(axis=0)
                    centre_marqueur = (
                        int(centre[0]),
                        int(centre[1]),
                    )
                    angle_marqueur = calculer_orientation_marqueur(
                        marker_coins
                    )
                    angle_pince = (
                        angle_marqueur + OFFSET_PINCE_MARQUEUR_DEG
                    ) % 360

                    cv2.circle(
                        frame,
                        centre_marqueur,
                        7,
                        (255, 0, 255),
                        -1,
                    )
                    break

            resultats = model.predict(
                source=frame,
                conf=CONFIANCE_MIN_GOBELET,
                verbose=False,
            )

            meilleur_gobelet = None

            for box in resultats[0].boxes:
                confiance = float(box.conf[0].item())

                if (
                    meilleur_gobelet is None
                    or confiance > meilleur_gobelet["confiance"]
                ):
                    x1, y1, x2, y2 = (
                        box.xyxy[0]
                        .cpu()
                        .numpy()
                        .astype(int)
                    )
                    meilleur_gobelet = {
                        "confiance": confiance,
                        "rectangle": (x1, y1, x2, y2),
                    }

            angle_gobelet = None
            point_gobelet = None

            if meilleur_gobelet is not None:
                x1, y1, x2, y2 = meilleur_gobelet["rectangle"]
                point_gobelet = (
                    int((x1 + x2) / 2),
                    int(y2),
                )

                cv2.rectangle(
                    frame,
                    (x1, y1),
                    (x2, y2),
                    (0, 255, 0),
                    2,
                )
                cv2.circle(
                    frame,
                    point_gobelet,
                    7,
                    (0, 255, 255),
                    -1,
                )

            if centre_robot["pixel"] is not None:
                cv2.circle(
                    frame,
                    centre_robot["pixel"],
                    9,
                    (0, 0, 255),
                    -1,
                )

                if point_gobelet is not None:
                    angle_gobelet = calculer_angle(
                        centre_robot["pixel"],
                        point_gobelet,
                    )
                    cv2.line(
                        frame,
                        centre_robot["pixel"],
                        point_gobelet,
                        (0, 255, 255),
                        2,
                    )

            if centre_robot["pixel"] is None:
                message = "CLIQUE SUR L'AXE DE ROTATION DU ROBOT"
                couleur = (0, 0, 255)
            elif angle_pince is None:
                message = "MARQUEUR ARUCO 4 NON DETECTE"
                couleur = (0, 0, 255)
            elif angle_gobelet is None:
                message = (
                    f"marqueur={angle_marqueur:.1f} | "
                    f"pince={angle_pince:.1f} | AUCUN GOBELET"
                )
                couleur = (0, 165, 255)
            else:
                erreur = erreur_angulaire(angle_gobelet, angle_pince)
                message = (
                    f"pince={angle_pince:.1f} | "
                    f"gobelet={angle_gobelet:.1f} | "
                    f"erreur={erreur:+.1f} deg"
                )
                couleur = (
                    (0, 255, 0)
                    if abs(erreur) <= 2
                    else (0, 165, 255)
                )

            cv2.rectangle(
                frame,
                (5, 5),
                (900, 48),
                (0, 0, 0),
                -1,
            )
            cv2.putText(
                frame,
                message,
                (15, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.75,
                couleur,
                2,
            )

            cv2.imshow(WINDOW_NAME, frame)
            touche = cv2.waitKey(1) & 0xFF

            if touche == ord("q") or touche == 27:
                break

    except KeyboardInterrupt:
        print("\nArrêt demandé")

    finally:
        camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
