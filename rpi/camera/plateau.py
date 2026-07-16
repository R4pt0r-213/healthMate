from pathlib import Path

import cv2
import numpy as np
import math


H_PATH = Path(__file__).with_name("homography.npy")
H = np.load(H_PATH)

# Définition des zones du plateau
ZONES_X = [
    ("S", 0, 12),
    ("W", 12, 30),
    ("D", 30, 40.1)
]


# Position du centre de rotation du robot
ROBOT_X = 21
ROBOT_Y = 17

# Correspondance avec Arduino
ANGLE_REFERENCE_PLATEAU = 155
ANGLE_REFERENCE_SERVO = 180


def pixel_to_cm(x_pixel, y_pixel):
    """
    Convertit une position en pixels (caméra)
    vers une position en centimètres sur le plateau.
    """
    point = np.array([[[x_pixel, y_pixel]]], dtype=np.float32)
    point_cm = cv2.perspectiveTransform(point, H)
    return point_cm[0][0][0], point_cm[0][0][1]


ZONES_X = [
    ("S", 0, 12),
    ("W", 12, 26),
    ("D", 26, 40.1)
]


def get_zone(x_cm):

    for nom, xmin, xmax in ZONES_X:
        if xmin <= x_cm < xmax:
            return nom

    return "Hors plateau"



def get_medicament(x_cm, y_cm, zone):

    # Zone S : séparation en Y
    if zone == "S":

        if 0 <= y_cm < 9:
            return "A"

        elif 9 <= y_cm < 18:
            return "B"


    # Zone travail : séparation en X
    elif zone == "W":

        if 12 <= x_cm < 21:
            return "A"

        elif 21 <= x_cm < 30:
            return "B"


    elif zone == "D":
        return "Depot"


    return "Hors zone"



def get_robot_position(x_cm, y_cm):
    # Position relative au centre du robot
    dx = x_cm - ROBOT_X
    dy = y_cm - ROBOT_Y

    # Distance entre le robot et le gobelet
    distance = math.sqrt(dx**2 + dy**2)

    # Angle autour du robot, dans le repère du plateau
    angle_plateau = math.degrees(math.atan2(dy, dx))

    if angle_plateau < 0:
        angle_plateau += 360

    # Différence circulaire entre -180° et +180°
    ecart_angle = (
        (angle_plateau - ANGLE_REFERENCE_PLATEAU + 180) % 360
    ) - 180

    # Conversion vers l’angle du servomoteur
    angle_servo = ANGLE_REFERENCE_SERVO - ecart_angle

    # Limitation à la course physique du servo
    angle_servo = max(0, min(180, angle_servo))

    return {
        "dx": dx,
        "dy": dy,
        "distance_cm": distance,
        "angle_plateau": angle_plateau,
        "angle_servo": angle_servo,
    }

def analyse_position(x_pixel, y_pixel):

    x_cm, y_cm = pixel_to_cm(
        x_pixel,
        y_pixel
    )

    robot = get_robot_position(
        x_cm,
        y_cm
    )

    zone = get_zone(x_cm)

    medicament = get_medicament(
        x_cm,
        y_cm,
        zone
    )

    return {
        # Coordonnées absolues sur le plateau
        "x_cm": x_cm,
        "y_cm": y_cm,

        # Coordonnées relatives au robot
        "dx": robot["dx"],
        "dy": robot["dy"],

        # Zone et médicament
        "zone": zone,
        "medicament": medicament,

        # Informations robot
        "distance_robot": robot["distance_cm"],
        "angle_plateau": robot["angle_plateau"],
        "angle_servo": robot["angle_servo"],
    }