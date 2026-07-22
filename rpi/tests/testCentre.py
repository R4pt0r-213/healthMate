import cv2
import numpy as np

from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from camera.plateau import (
    H,
    ROBOT_MARKER_ID,
    centre_robot_global,
    construire_dictionnaire_marqueurs,
    obtenir_marqueur,
    obtenir_transformation_plateau,
    repere_plateau_visible,
)

ARUCO_ID = ROBOT_MARKER_ID


def cm_to_pixel(x_cm, y_cm):
    H_inv = np.linalg.inv(H)

    point = np.array(
        [[[x_cm, y_cm]]],
        dtype=np.float32,
    )

    pixel = cv2.perspectiveTransform(point, H_inv)

    return (
        int(pixel[0][0][0]),
        int(pixel[0][0][1]),
    )


dictionnaire = cv2.aruco.getPredefinedDictionary(
    cv2.aruco.DICT_4X4_50
)

detecteur = cv2.aruco.ArucoDetector(
    dictionnaire,
    cv2.aruco.DetectorParameters(),
)

camera = cv2.VideoCapture(0)

while True:

    ok, frame = camera.read()

    if not ok:
        break

    corners, ids, _ = detecteur.detectMarkers(frame)
    markers = construire_dictionnaire_marqueurs(corners, ids)

    if ids is not None:
        cv2.aruco.drawDetectedMarkers(
            frame,
            corners,
            ids,
        )

    try:
        obtenir_transformation_plateau(markers)
        marker = obtenir_marqueur(ARUCO_ID, markers)
        centre = marker.mean(axis=0)

        cv2.circle(
            frame,
            (int(centre[0]), int(centre[1])),
            6,
            (0, 255, 0),
            -1,
        )

        robot_x, robot_y = centre_robot_global(marker)
        pixel_robot = cm_to_pixel(robot_x, robot_y)

        cv2.circle(
            frame,
            pixel_robot,
            8,
            (0, 0, 255),
            -1,
        )

        if not repere_plateau_visible(markers):
            cv2.putText(
                frame,
                "CENTRE CALCULE AVEC LA MEMOIRE",
                (15, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 165, 255),
                2,
            )
    except (RuntimeError, ValueError):
        cv2.putText(
            frame,
            "INITIALISATION : MONTRER 0, 1, 2, 3",
            (15, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 0, 255),
            2,
        )

    cv2.imshow("Test centre robot", frame)

    if cv2.waitKey(1) == 27:
        break

camera.release()
cv2.destroyAllWindows()
