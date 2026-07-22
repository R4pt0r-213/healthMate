from pathlib import Path
import sys

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from camera.plateau import (
    H,
    PLATEAU_WIDTH_CM,
    PLATEAU_HEIGHT_CM,
    ZONES,
    construire_dictionnaire_marqueurs,
    obtenir_transformation_plateau,
    repere_plateau_visible,
)


dictionnaire = cv2.aruco.getPredefinedDictionary(
    cv2.aruco.DICT_4X4_50
)

detecteur = cv2.aruco.ArucoDetector(
    dictionnaire,
    cv2.aruco.DetectorParameters(),
)

camera = cv2.VideoCapture(0)


# Couleur de chaque zone en BGR
COULEURS_ZONES = {
    "S-A": (0, 255, 0),
    "S-B": (0, 150, 0),
    "D": (255, 0, 0),
    "W-A": (0, 255, 255),
    "W-B": (0, 150, 150),
}


def transformer_points(points_cm, transformation):
    """
    Transforme des coordonnées du plateau en coordonnées pixels.
    """
    points = np.array(
        [points_cm],
        dtype=np.float32,
    )

    points_pixels = cv2.perspectiveTransform(
        points,
        transformation,
    )

    return points_pixels[0].astype(np.int32)


def dessiner_zone(
    overlay,
    frame,
    points_cm,
    transformation,
    couleur,
    nom,
):
    """
    Dessine une zone colorée avec son nom.
    """
    points_pixels = transformer_points(
        points_cm,
        transformation,
    )

    # Remplissage de la zone sur l'overlay
    cv2.fillPoly(
        overlay,
        [points_pixels],
        couleur,
    )

    return points_pixels


def dessiner_contour_et_nom(
    frame,
    points_pixels,
    couleur,
    nom,
):
    """
    Dessine le contour et le nom après l'application
    de la transparence.
    """
    cv2.polylines(
        frame,
        [points_pixels],
        isClosed=True,
        color=couleur,
        thickness=3,
    )

    centre = points_pixels.mean(axis=0).astype(int)

    cv2.putText(
        frame,
        nom,
        (
            int(centre[0]) - 25,
            int(centre[1]) + 10,
        ),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (255, 255, 255),
        3,
    )


while True:

    ok, frame = camera.read()

    if not ok:
        break

    corners, ids, _ = detecteur.detectMarkers(frame)
    markers = construire_dictionnaire_marqueurs(corners, ids)
    repere_visible = repere_plateau_visible(markers)

    if ids is not None:
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

    try:
        # plateau = transformation_plateau × H × pixel
        # donc pixel = inverse(transformation_plateau × H) × plateau.
        transformation_plateau = obtenir_transformation_plateau(markers)
        transformation = np.linalg.inv(transformation_plateau @ H)

        points_plateau_cm = np.array(
            [
                [0.0, 0.0],
                [PLATEAU_WIDTH_CM, 0.0],
                [PLATEAU_WIDTH_CM, PLATEAU_HEIGHT_CM],
                [0.0, PLATEAU_HEIGHT_CM],
            ],
            dtype=np.float32,
        )

        points_plateau_pixels = transformer_points(
            points_plateau_cm,
            transformation,
        )

        overlay = frame.copy()

        zones_pixels = {}

        # Remplissage de toutes les zones définies
        # dans plateau.py.
        for nom_zone, points_zone in ZONES.items():

            couleur = COULEURS_ZONES.get(
                nom_zone,
                (255, 255, 255),
            )

            zones_pixels[nom_zone] = dessiner_zone(
                overlay,
                frame,
                points_zone,
                transformation,
                couleur,
                nom_zone,
            )

        # Application de la transparence.
        cv2.addWeighted(
            overlay,
            0.30,
            frame,
            0.70,
            0,
            frame,
        )

        # Dessin des contours et des noms
        # après la transparence.
        for nom_zone, points_pixels in zones_pixels.items():

            couleur = COULEURS_ZONES.get(
                nom_zone,
                (255, 255, 255),
            )

            dessiner_contour_et_nom(
                frame,
                points_pixels,
                couleur,
                nom_zone,
            )

        # Contour extérieur du plateau.
        contour_plateau = points_plateau_pixels.astype(
            np.int32
        )

        cv2.polylines(
            frame,
            [contour_plateau],
            isClosed=True,
            color=(0, 255, 0),
            thickness=3,
        )

        if not repere_visible:
            cv2.putText(
                frame,
                "REPERE MEMORISE",
                (15, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 165, 255),
                2,
            )

    except RuntimeError:
        cv2.putText(
            frame,
            "INITIALISATION : MONTRER 0, 1, 2, 3",
            (15, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 255),
            2,
        )

    cv2.imshow(
        "Zones du plateau",
        frame,
    )

    # Échap pour quitter.
    if cv2.waitKey(1) == 27:
        break


camera.release()
cv2.destroyAllWindows()
