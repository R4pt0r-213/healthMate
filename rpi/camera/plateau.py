from pathlib import Path
import math

import cv2
import numpy as np


H_PATH = Path(__file__).with_name("homography.npy")
H = np.load(H_PATH)


# ============================================================
# DIMENSIONS RÉELLES DU PLATEAU
# ============================================================

PLATEAU_WIDTH_CM = 42.0
PLATEAU_HEIGHT_CM = 29.5


# ============================================================
# ZONES DU PLATEAU
# ============================================================

ZONE_S_A = np.array(
    [
        [0.0, 29.5],
        [13.0, 29.5],
        [13.0, 22.5],
        [0.0, 22.5],
    ],
    dtype=np.float32,
)

ZONE_S_B = np.array(
    [
        [0.0, 22.5],
        [13.0, 22.5],
        [13.0, 15.5],
        [0.0, 15.5],
    ],
    dtype=np.float32,
)

ZONE_D = np.array(
    [
        [13.0, 19.0],
        [29.0, 19.0],
        [29.0, 29.5],
        [13.0, 29.5],
    ],
    dtype=np.float32,
)

ZONE_W_A = np.array(
    [
        [29.0, 29.5],
        [42.0, 29.5],
        [42.0, 22.5],
        [29.0, 22.5],
    ],
    dtype=np.float32,
)

ZONE_W_B = np.array(
    [
        [29.0, 22.5],
        [42.0, 22.5],
        [42.0, 15.5],
        [29.0, 15.5],
    ],
    dtype=np.float32,
)


ZONES = {
    "S-A": ZONE_S_A,
    "S-B": ZONE_S_B,
    "D": ZONE_D,
    "W-A": ZONE_W_A,
    "W-B": ZONE_W_B,
}


# ============================================================
# PARAMÈTRES DU ROBOT
# ============================================================

ROBOT_OFFSET_X = -15.0
ROBOT_OFFSET_Y = 10.0

DEPOT_X_CM = 23.91
DEPOT_Y_CM = 21.41
# Commande imposée pour le dépôt. Elle évite que les petites variations
# du centre du robot détecté changent l'orientation finale de la base.
ANGLE_SERVO_DEPOT = 90.0

PLATEAU_MARKER_IDS = (0, 1, 2, 3)
ROBOT_MARKER_ID = 1


CALIBRATION_SERVO = [
    (0.0, 153.26),
    (30.0, 130.99),
    (60.0, 100.93),
    (90.0, 73.86),
    (120.0, 48.45),
    (150.0, 22.40),
    (180.0, -6.37),
]

# Décalage entre le bord supérieur du marqueur ArUco 4 et l'axe réel
# des mors. Recalculé depuis une position où la pince était physiquement
# alignée avec le gobelet : 103.7° + 143.2° = 246.9°.
OFFSET_PINCE_MARQUEUR_DEG = 246.9

# La direction utilisable par le servo correspond au demi-cercle opposé
# à celui mesuré initialement par le bord du marqueur.
DECALAGE_CALIBRATION_SERVO_DEG = -180.0

CORRECTION_ANGLE_PRISE_DEG = -10.0

# Un gobelet plus proche risquerait de se trouver sous le bras ou trop près
# de l'axe de rotation pour être saisi correctement.
DISTANCE_MIN_PRISE_CM = 6.0
DISTANCE_MAX_PRISE_CM = 25.0


# ============================================================
# MÉMOIRE DU PLATEAU
# ============================================================

# Dernière transformation valide calculée lorsque les quatre
# marqueurs du plateau étaient visibles.
_transformation_plateau_memorisee = None

# Dernière position détectée du marqueur utilisé pour calculer
# la position du robot.
_marqueurs_memorises = {}


def reinitialiser_memoire_plateau():
    """
    Efface la transformation et les marqueurs mémorisés.

    À utiliser si la caméra ou le plateau a été déplacé.
    """
    global _transformation_plateau_memorisee
    global _marqueurs_memorises

    _transformation_plateau_memorisee = None
    _marqueurs_memorises = {}


def repere_plateau_memorise():
    """
    Indique si une transformation valide du plateau
    a déjà été mémorisée.
    """
    return _transformation_plateau_memorisee is not None


# ============================================================
# FONCTIONS DE CONVERSION
# ============================================================

def normaliser_corners(marker_corners):
    """
    Transforme les coins d'un marqueur en tableau (4, 2).
    """
    corners = np.asarray(
        marker_corners,
        dtype=np.float32,
    )

    return corners.reshape(4, 2)


def construire_dictionnaire_marqueurs(corners, ids):
    """
    Convertit le résultat OpenCV ArUco en dictionnaire :

        {
            identifiant: coins
        }
    """
    if ids is None:
        return {}

    return {
        int(marker_id): normaliser_corners(marker_corners)
        for marker_id, marker_corners
        in zip(ids.flatten(), corners)
    }


def repere_plateau_visible(markers):
    """
    Indique si les quatre marqueurs du plateau sont visibles.
    """
    return all(
        marker_id in markers
        for marker_id in PLATEAU_MARKER_IDS
    )


def memoriser_marqueurs_visibles(markers):
    """
    Mémorise les dernières positions connues des marqueurs visibles.
    """
    global _marqueurs_memorises

    for marker_id, marker_corners in markers.items():
        _marqueurs_memorises[marker_id] = (
            normaliser_corners(marker_corners).copy()
        )


def obtenir_marqueur(marker_id, markers):
    """
    Retourne un marqueur actuellement visible.

    S'il n'est plus visible, retourne sa dernière position mémorisée.
    """
    if marker_id in markers:
        marker_corners = normaliser_corners(
            markers[marker_id]
        )

        _marqueurs_memorises[marker_id] = (
            marker_corners.copy()
        )

        return marker_corners

    if marker_id in _marqueurs_memorises:
        return _marqueurs_memorises[marker_id].copy()

    raise ValueError(
        f"Le marqueur {marker_id} n'a jamais été détecté."
    )


def pixel_to_cm(x_pixel, y_pixel):
    """
    Convertit une position en pixels vers une position en cm
    dans le repère global de calibration.
    """
    point = np.array(
        [[[x_pixel, y_pixel]]],
        dtype=np.float32,
    )

    point_cm = cv2.perspectiveTransform(
        point,
        H,
    )

    return (
        float(point_cm[0][0][0]),
        float(point_cm[0][0][1]),
    )


def centre_marqueur_pixel(marker_corners):
    """
    Retourne le centre d'un marqueur en pixels.
    """
    corners = normaliser_corners(marker_corners)
    centre = corners.mean(axis=0)

    return (
        float(centre[0]),
        float(centre[1]),
    )


def centre_marqueur_cm(marker_corners):
    """
    Retourne le centre d'un marqueur dans le repère global en cm.
    """
    centre_x, centre_y = centre_marqueur_pixel(
        marker_corners
    )

    return pixel_to_cm(
        centre_x,
        centre_y,
    )


def orientation_marqueur(marker_corners):
    """
    Retourne l'orientation du marqueur dans le repère global.
    """
    corners = normaliser_corners(marker_corners)

    coin_0 = corners[0]
    coin_1 = corners[1]

    x_0, y_0 = pixel_to_cm(
        coin_0[0],
        coin_0[1],
    )

    x_1, y_1 = pixel_to_cm(
        coin_1[0],
        coin_1[1],
    )

    return math.atan2(
        y_1 - y_0,
        x_1 - x_0,
    )


# ============================================================
# CONSTRUCTION DU REPÈRE DU PLATEAU
# ============================================================

def trouver_coins_exterieurs(markers):
    """
    Trouve le coin extérieur de chaque marqueur.

    Organisation attendue :

        0 ---------------- 1
        |                  |
        |     plateau      |
        |                  |
        2 ---------------- 3
    """
    centres_pixels = {
        marker_id: np.array(
            centre_marqueur_pixel(markers[marker_id]),
            dtype=np.float32,
        )
        for marker_id in PLATEAU_MARKER_IDS
    }

    centre_plateau = np.mean(
        [
            centres_pixels[0],
            centres_pixels[1],
            centres_pixels[2],
            centres_pixels[3],
        ],
        axis=0,
    )

    coins_exterieurs = {}

    for marker_id in PLATEAU_MARKER_IDS:

        coins = normaliser_corners(
            markers[marker_id]
        )

        distances = np.linalg.norm(
            coins - centre_plateau,
            axis=1,
        )

        index_coin_exterieur = np.argmax(
            distances
        )

        coin_pixel = coins[index_coin_exterieur]

        coins_exterieurs[marker_id] = pixel_to_cm(
            coin_pixel[0],
            coin_pixel[1],
        )

    return coins_exterieurs


def construire_repere_plateau(markers):
    """
    Construit une nouvelle transformation du repère global
    vers le repère du plateau.

    Cette fonction nécessite les quatre marqueurs.
    """
    if not repere_plateau_visible(markers):
        raise ValueError(
            "Les marqueurs 0, 1, 2 et 3 sont nécessaires "
            "pour construire une nouvelle transformation."
        )

    coins_exterieurs = trouver_coins_exterieurs(
        markers
    )

    # Ordre :
    # haut gauche, haut droit, bas droit, bas gauche.
    points_globaux = np.array(
        [
            coins_exterieurs[0],
            coins_exterieurs[1],
            coins_exterieurs[3],
            coins_exterieurs[2],
        ],
        dtype=np.float32,
    )

    points_plateau = np.array(
        [
            [0.0, 0.0],
            [PLATEAU_WIDTH_CM, 0.0],
            [PLATEAU_WIDTH_CM, PLATEAU_HEIGHT_CM],
            [0.0, PLATEAU_HEIGHT_CM],
        ],
        dtype=np.float32,
    )

    return cv2.getPerspectiveTransform(
        points_globaux,
        points_plateau,
    )


def obtenir_transformation_plateau(markers):
    """
    Retourne la transformation du plateau.

    Si les quatre marqueurs sont visibles :
        une nouvelle transformation est calculée et mémorisée.

    Sinon :
        la dernière transformation valide est réutilisée.
    """
    global _transformation_plateau_memorisee

    # On mémorise tous les marqueurs visibles.
    memoriser_marqueurs_visibles(markers)

    if repere_plateau_visible(markers):

        _transformation_plateau_memorisee = (
            construire_repere_plateau(markers)
        )

    if _transformation_plateau_memorisee is None:
        raise RuntimeError(
            "Le plateau n'a pas encore été initialisé. "
            "Les quatre marqueurs 0, 1, 2 et 3 doivent être "
            "visibles au moins une fois."
        )

    return _transformation_plateau_memorisee.copy()


def global_to_plateau(
    x_global,
    y_global,
    transformation_plateau,
):
    """
    Convertit un point du repère global vers le repère
    relatif du plateau.
    """
    point_global = np.array(
        [[[x_global, y_global]]],
        dtype=np.float32,
    )

    point_plateau = cv2.perspectiveTransform(
        point_global,
        transformation_plateau,
    )

    return (
        float(point_plateau[0][0][0]),
        float(point_plateau[0][0][1]),
    )


def pixel_to_plateau(
    x_pixel,
    y_pixel,
    transformation_plateau,
):
    """
    Convertit directement un point caméra en coordonnées
    relatives au plateau.
    """
    x_global, y_global = pixel_to_cm(
        x_pixel,
        y_pixel,
    )

    return global_to_plateau(
        x_global,
        y_global,
        transformation_plateau,
    )


# ============================================================
# POSITION DU ROBOT
# ============================================================

def centre_robot_global(marker_corners):
    """
    Calcule la position du centre du robot à partir
    du marqueur de référence.
    """
    x_marqueur, y_marqueur = centre_marqueur_cm(
        marker_corners
    )

    theta = orientation_marqueur(
        marker_corners
    )

    dx = (
        ROBOT_OFFSET_X * math.cos(theta)
        - ROBOT_OFFSET_Y * math.sin(theta)
    )

    dy = (
        ROBOT_OFFSET_X * math.sin(theta)
        + ROBOT_OFFSET_Y * math.cos(theta)
    )

    return (
        x_marqueur + dx,
        y_marqueur + dy,
    )


def centre_robot_plateau(
    robot_marker_corners,
    transformation_plateau,
):
    """
    Retourne le centre du robot dans le repère du plateau.
    """
    robot_x_global, robot_y_global = centre_robot_global(
        robot_marker_corners
    )

    return global_to_plateau(
        robot_x_global,
        robot_y_global,
        transformation_plateau,
    )


# ============================================================
# GESTION DU SERVO
# ============================================================

def angle_pince_to_servo(angle_pince):
    """
    Interpole la commande du servo depuis les mesures réelles.
    """
    # La table peut traverser 0° et contenir des valeurs négatives. On choisit
    # donc la représentation circulaire de la cible la plus proche du centre
    # de la plage calibrée : angle, angle - 360 ou angle + 360.
    angle_normalise = angle_pince % 360
    centre_calibration = (
        CALIBRATION_SERVO[0][1]
        + CALIBRATION_SERVO[-1][1]
    ) / 2
    angle_pince = min(
        (
            angle_normalise - 360,
            angle_normalise,
            angle_normalise + 360,
        ),
        key=lambda candidat: abs(candidat - centre_calibration),
    )

    for index in range(
        1,
        len(CALIBRATION_SERVO),
    ):
        servo_1, pince_1 = CALIBRATION_SERVO[index - 1]
        servo_2, pince_2 = CALIBRATION_SERVO[index]

        if pince_2 <= angle_pince <= pince_1:

            ratio = (
                (angle_pince - pince_1)
                / (pince_2 - pince_1)
            )

            return (
                servo_1
                + ratio * (servo_2 - servo_1)
            )

    return None


def construire_polygone_portee_robot(
    robot_x,
    robot_y,
    nombre_points=100,
):
    """
    Construit en coordonnées plateau le secteur réellement prenable.

    Il combine la plage angulaire calibrée du servo et les distances
    minimale/maximale autorisées pour une prise.
    """
    angles_pinces = [angle_pince for _, angle_pince in CALIBRATION_SERVO]
    angle_min = min(angles_pinces) - CORRECTION_ANGLE_PRISE_DEG
    angle_max = max(angles_pinces) - CORRECTION_ANGLE_PRISE_DEG

    angles_exterieurs = np.linspace(
        angle_min,
        angle_max,
        nombre_points,
    )
    angles_interieurs = angles_exterieurs[::-1]

    def points_arc(distance, angles):
        radians = np.radians(angles)
        return np.column_stack(
            (
                robot_x + distance * np.cos(radians),
                robot_y + distance * np.sin(radians),
            )
        )

    arc_exterieur = points_arc(
        DISTANCE_MAX_PRISE_CM,
        angles_exterieurs,
    )
    arc_interieur = points_arc(
        DISTANCE_MIN_PRISE_CM,
        angles_interieurs,
    )

    return np.vstack((arc_exterieur, arc_interieur)).astype(np.float32)


def points_plateau_to_pixels(points_plateau, transformation_plateau):
    """Projette des points du repère plateau vers l'image caméra."""
    transformation_pixel_vers_plateau = transformation_plateau @ H
    transformation_plateau_vers_pixel = np.linalg.inv(
        transformation_pixel_vers_plateau
    )

    points = np.array([points_plateau], dtype=np.float32)
    pixels = cv2.perspectiveTransform(
        points,
        transformation_plateau_vers_pixel,
    )
    return pixels[0]


# ============================================================
# DÉTECTION DES ZONES
# ============================================================

def est_dans_plateau(x_cm, y_cm):
    """
    Vérifie si un point se trouve dans le plateau.
    """
    return (
        0.0 <= x_cm <= PLATEAU_WIDTH_CM
        and 0.0 <= y_cm <= PLATEAU_HEIGHT_CM
    )


def est_dans_zone(x_cm, y_cm, zone):
    """
    Vérifie si un point se trouve dans une zone.
    """
    resultat = cv2.pointPolygonTest(
        zone,
        (float(x_cm), float(y_cm)),
        False,
    )

    return resultat >= 0


def get_zone_detaillee(x_cm, y_cm):
    """
    Retourne la zone détaillée du point.
    """
    if not est_dans_plateau(x_cm, y_cm):
        return "Hors plateau"

    for nom_zone, points_zone in ZONES.items():

        if est_dans_zone(
            x_cm,
            y_cm,
            points_zone,
        ):
            return nom_zone

    return "Hors zone"


def get_zone(x_cm, y_cm):
    """
    Retourne la zone principale : S, D ou W.
    """
    zone_detaillee = get_zone_detaillee(
        x_cm,
        y_cm,
    )

    if zone_detaillee in ("S-A", "S-B"):
        return "S"

    if zone_detaillee == "D":
        return "D"

    if zone_detaillee in ("W-A", "W-B"):
        return "W"

    return zone_detaillee


def get_medicament(x_cm, y_cm, zone=None):
    """
    Retourne le médicament A ou B.
    """
    zone_detaillee = get_zone_detaillee(
        x_cm,
        y_cm,
    )

    if zone_detaillee in ("S-A", "W-A"):
        return "A"

    if zone_detaillee in ("S-B", "W-B"):
        return "B"

    return None


# ============================================================
# POSITION PAR RAPPORT AU ROBOT
# ============================================================

def get_robot_position(
    x_cm,
    y_cm,
    robot_marker_corners,
    transformation_plateau,
):
    """
    Calcule la position d'un point par rapport au robot.
    """
    robot_x, robot_y = centre_robot_plateau(
        robot_marker_corners,
        transformation_plateau,
    )

    dx = x_cm - robot_x
    dy = y_cm - robot_y

    distance = math.hypot(
        dx,
        dy,
    )

    angle_plateau = math.degrees(
        math.atan2(dy, dx)
    ) % 360

    angle_cible_pince = (
        angle_plateau
        + CORRECTION_ANGLE_PRISE_DEG
    ) % 360

    angle_servo = angle_pince_to_servo(
        angle_cible_pince
    )

    return {
        "robot_x": robot_x,
        "robot_y": robot_y,
        "dx": dx,
        "dy": dy,
        "distance_cm": distance,
        "angle_plateau": angle_plateau,
        "angle_cible_pince": angle_cible_pince,
        "angle_servo": angle_servo,
    }


# ============================================================
# ANALYSE COMPLÈTE
# ============================================================

def analyse_position(
    x_pixel,
    y_pixel,
    markers,
    robot_marker_id=ROBOT_MARKER_ID,
):
    """
    Analyse un point détecté dans l'image.

    Au premier appel, les quatre marqueurs du plateau doivent
    être visibles.

    Après cette initialisation, la dernière transformation valide
    est réutilisée si un marqueur devient momentanément illisible.
    """
    transformation_plateau = obtenir_transformation_plateau(
        markers
    )

    x_cm, y_cm = pixel_to_plateau(
        x_pixel,
        y_pixel,
        transformation_plateau,
    )

    # Le marqueur actuel est utilisé s'il est visible.
    # Sinon, sa dernière position connue est récupérée.
    robot_marker_corners = obtenir_marqueur(
        robot_marker_id,
        markers,
    )

    robot = get_robot_position(
        x_cm,
        y_cm,
        robot_marker_corners,
        transformation_plateau,
    )

    zone = get_zone(
        x_cm,
        y_cm,
    )

    zone_detaillee = get_zone_detaillee(
        x_cm,
        y_cm,
    )

    medicament = get_medicament(
        x_cm,
        y_cm,
        zone,
    )

    return {
        "x_cm": x_cm,
        "y_cm": y_cm,

        "robot_x": robot["robot_x"],
        "robot_y": robot["robot_y"],

        "dx": robot["dx"],
        "dy": robot["dy"],

        "zone": zone,
        "zone_detaillee": zone_detaillee,
        "medicament": medicament,

        "distance_robot": robot["distance_cm"],
        "angle_plateau": robot["angle_plateau"],
        "angle_cible_pince": robot["angle_cible_pince"],
        "angle_servo": robot["angle_servo"],

        # Indique si les quatre marqueurs étaient visibles
        # pendant cette analyse.
        "plateau_actuellement_visible": repere_plateau_visible(
            markers
        ),

        # Indique qu'une transformation valide est disponible.
        "plateau_memorise": repere_plateau_memorise(),
    }
