from robot.communication import Robot

def main():

    robot = Robot()

    try:

        angle = 120
        distance = 11

        print("Envoi de la commande...")

        robot.envoyer(angle, distance)

        if robot.attendre_fin(timeout=60):
            print("Cycle terminé")
        else:
            print("Cycle non confirmé par l'Arduino")

    finally:

        robot.fermer()


if __name__ == "__main__":
    main()
