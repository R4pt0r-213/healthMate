"""
Gestion de la caméra.
"""

import cv2


# Création de la caméra
camera = cv2.VideoCapture(0)



def get_image():

    """
    Capture une image.

    Retour :
    une image OpenCV
    """


    success, image = camera.read()


    if not success:
        raise Exception(
            "Impossible de récupérer l'image"
        )


    return image