"""
Fonctions haut niveau du robot.
"""


from robot.communication import send_command



def move_to(arduino,x,y):

    """
    Déplace le bras à une position.
    """


    send_command(
        arduino,
        f"MOVE {x} {y}"
    )




def grab(arduino):

    """
    Séquence de prise.
    """


    send_command(
        arduino,
        "GRAB"
    )




def release(arduino):

    """
    Séquence de dépose.
    """


    send_command(
        arduino,
        "RELEASE"
    )