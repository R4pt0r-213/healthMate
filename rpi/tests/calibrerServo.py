from pathlib import Path
import math
import sys
import time

import cv2


PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from camera.plateau import (
    DECALAGE_CALIBRATION_SERVO_DEG,
    OFFSET_PINCE_MARQUEUR_DEG,
    pixel_to_cm,
)
from robot.communication import Robot


ARUCO_BRAS_ID = 4
ANGLES_TEST = [0, 30, 60, 90, 120, 150, 180]
DISTANCE_TEST = 11


def orientation_marqueur(marker_coins):
    coin_0 = marker_coins[0][0]
    coin_1 = marker_coins[0][1]

    x_0, y_0 = pixel_to_cm(float(coin_0[0]), float(coin_0[1]))
    x_1, y_1 = pixel_to_cm(float(coin_1[0]), float(coin_1[1]))

    angle = math.degrees(math.atan2(y_1 - y_0, x_1 - x_0))
    return angle % 360


def moyenne_circulaire(angles):
    sinus = sum(math.sin(math.radians(angle)) for angle in angles)
    cosinus = sum(math.cos(math.radians(angle)) for angle in angles)
    return math.degrees(math.atan2(sinus, cosinus)) % 360


def difference_angulaire(angle_final, angle_initial):
    return ((angle_final - angle_initial + 180) % 360) - 180


def mesurer_pince(camera, detecteur, nombre_mesures=30, timeout=8):
    mesures = []
    debut = time.time()

    while len(mesures) < nombre_mesures and time.time() - debut < timeout:
        success, frame = camera.read()

        if not success:
            continue

        coins, identifiants, _ = detecteur.detectMarkers(frame)

        if identifiants is None:
            continue

        for marker_id, marker_coins in zip(
            identifiants.flatten(),
            coins,
        ):
            if int(marker_id) != ARUCO_BRAS_ID:
                continue

            angle_marqueur = orientation_marqueur(marker_coins)
            angle_pince = (
                angle_marqueur
                + OFFSET_PINCE_MARQUEUR_DEG
                + DECALAGE_CALIBRATION_SERVO_DEG
            ) % 360
            mesures.append(angle_pince)
            break

    if not mesures:
        raise RuntimeError("Marqueur ArUco ID 4 non détecté")

    return moyenne_circulaire(mesures)


def commander_et_mesurer(robot, camera, detecteur, commande):
    print(f"\nCommande du servo : {commande}°")
    robot.envoyer(commande, DISTANCE_TEST)

    if not robot.attendre_fin(timeout=30):
        raise RuntimeError("L'Arduino n'a pas répondu TERMINE")

    # Attend que les vibrations mécaniques disparaissent.
    time.sleep(0.5)

    angle_pince = mesurer_pince(camera, detecteur)
    print(f"Angle réel de la pince : {angle_pince:.2f}°")
    return angle_pince


def derouler_angle(angle, reference):
    """Évite le saut artificiel de 0° à 360° dans la table finale."""
    while angle - reference > 180:
        angle -= 360
    while angle - reference < -180:
        angle += 360
    return angle


def main():
    print("Calibration du servo de base avec le marqueur ArUco ID 4")
    print("Le bras va parcourir automatiquement les angles 0° à 180°.")

    dictionnaire = cv2.aruco.getPredefinedDictionary(
        cv2.aruco.DICT_4X4_50
    )
    parametres = cv2.aruco.DetectorParameters()
    parametres.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    detecteur = cv2.aruco.ArucoDetector(dictionnaire, parametres)

    camera = cv2.VideoCapture(0)

    if not camera.isOpened():
        raise RuntimeError("Impossible d'ouvrir la caméra")

    robot = None

    try:
        robot = Robot()

        mesures = []

        for commande in ANGLES_TEST:
            angle_pince = commander_et_mesurer(
                robot,
                camera,
                detecteur,
                commande,
            )

            if mesures:
                angle_pince = derouler_angle(
                    angle_pince,
                    mesures[-1][1],
                )
            mesures.append((commande, angle_pince))

        print("\n--- RÉSULTAT À M'ENVOYER ---")

        for commande, angle_pince in mesures:
            print(f"Commande {commande}° -> pince {angle_pince:.2f}°")

        print("\nGains entre les mesures :")

        for index in range(1, len(mesures)):
            commande_1, pince_1 = mesures[index - 1]
            commande_2, pince_2 = mesures[index]
            variation_pince = difference_angulaire(pince_2, pince_1)
            gain = variation_pince / (commande_2 - commande_1)
            print(
                f"{commande_1}° à {commande_2}° : "
                f"gain={gain:.4f}"
            )

    except KeyboardInterrupt:
        print("\nCalibration interrompue")

    finally:
        camera.release()

        if robot is not None:
            robot.fermer()


if __name__ == "__main__":
    main()
