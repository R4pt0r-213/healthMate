"""
Communication série avec Arduino.
"""


import serial
import time



def connect_arduino():

    """
    Connexion USB avec Arduino.
    """


    arduino = serial.Serial(
        "/dev/ttyUSB0",
        9600
    )


    # Laisse le temps à Arduino de démarrer
    time.sleep(2)


    return arduino




def send_command(arduino, command):

    """
    Envoie une commande texte.
    """


    message = command + "\n"


    arduino.write(
        message.encode()
    )



    print(
        "Commande envoyée :",
        command
    )