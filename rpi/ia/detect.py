"""
Conversion coordonnées caméra -> coordonnées plateau
avec une matrice d'homographie calculée par ArUco.
"""


import cv2
import numpy as np



# Chargement de la matrice calculée
# pendant la calibration
H = np.load(
    "homography.npy"
)




def pixel_to_robot(px, py):

    """
    Transforme une position pixel caméra
    en position réelle sur le plateau.


    Entrée :
        px : position x en pixel
        py : position y en pixel


    Sortie :
        x,y en cm
    """



    # Format demandé par OpenCV :
    # tableau de points
    point = np.float32(
        [
            [
                px,
                py
            ]
        ]
    )



    # Application homographie
    point_transforme = cv2.perspectiveTransform(
        point.reshape(-1,1,2),
        H
    )



    x,y = point_transforme[0][0]



    return round(x,2), round(y,2)