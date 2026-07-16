"""
Programme principal du robot.

Rôle :
- Capturer une image
- Détecter un objet vide
- Calculer sa position
- Envoyer l'ordre au bras
"""

from camera.camera import get_image
from ia.detect import detect_objects
from geometry.position import pixel_to_robot
from robot.communication import connect_arduino, send_command
from robot.commands import grab


def main():

    # Connexion avec Arduino
    arduino = connect_arduino()

    print("Robot démarré")

    while True:

        # 1) Capture image caméra
        image = get_image()


        # 2) Détection YOLO
        objects = detect_objects(image)


        # 3) Analyse des objets détectés
        for obj in objects:


            # On cherche uniquement les objets vides
            if obj["type"] == "cup_empty":

                print("Objet vide détecté")


                # Coordonnées en pixels
                px = obj["x"]
                py = obj["y"]


                # Conversion pixels -> coordonnées robot
                x, y = pixel_to_robot(px, py)


                print(
                    "Position robot :",
                    x,
                    y
                )


                # Déplacement du bras
                send_command(
                    arduino,
                    f"MOVE {x} {y}"
                )


                # Prise de l'objet
                grab(arduino)



if __name__ == "__main__":
    main()