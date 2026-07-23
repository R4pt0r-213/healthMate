import cv2
import numpy as np

from camera.plateau import (
    DISTANCE_MAX_PRISE_CM,
    DISTANCE_MIN_PRISE_CM,
    ROBOT_MARKER_ID,
    centre_robot_plateau,
    construire_polygone_portee_robot,
    obtenir_marqueur,
    obtenir_transformation_plateau,
    points_plateau_to_pixels,
)


def dessiner_portee_robot(frame, markers):
    """Dessine la zone où angle et distance autorisent une prise."""
    try:
        transformation = obtenir_transformation_plateau(markers)
        marqueur_robot = obtenir_marqueur(ROBOT_MARKER_ID, markers)
    except (RuntimeError, ValueError):
        return False

    robot_x, robot_y = centre_robot_plateau(
        marqueur_robot,
        transformation,
    )
    polygone_cm = construire_polygone_portee_robot(robot_x, robot_y)
    polygone_pixels = points_plateau_to_pixels(
        polygone_cm,
        transformation,
    ).astype(np.int32)

    centre_pixel = points_plateau_to_pixels(
        np.array([[robot_x, robot_y]], dtype=np.float32),
        transformation,
    )[0].astype(int)

    overlay = frame.copy()
    cv2.fillPoly(overlay, [polygone_pixels], (0, 200, 0))
    cv2.addWeighted(overlay, 0.16, frame, 0.84, 0, frame)
    cv2.polylines(frame, [polygone_pixels], True, (0, 255, 0), 2)
    cv2.circle(frame, tuple(centre_pixel), 7, (0, 0, 255), -1)
    cv2.putText(
        frame,
        (
            f"PORTEE ROBOT : {DISTANCE_MIN_PRISE_CM:.0f} "
            f"A {DISTANCE_MAX_PRISE_CM:.0f} CM"
        ),
        (15, 90),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (0, 255, 0),
        2,
    )
    return True
