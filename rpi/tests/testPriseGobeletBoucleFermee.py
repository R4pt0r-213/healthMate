from pathlib import Path
import math
import sys
import time

import cv2
from ultralytics import YOLO


ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from camera.plateau import (
    CALIBRATION_SERVO,
    DECALAGE_CALIBRATION_SERVO_DEG,
    DISTANCE_MAX_PRISE_CM,
    DISTANCE_MIN_PRISE_CM,
    OFFSET_PINCE_MARQUEUR_DEG,
    analyse_position,
    construire_dictionnaire_marqueurs,
    obtenir_transformation_plateau,
    pixel_to_cm,
    repere_plateau_visible,
)
from robot.communication import Robot
from ia.config import CONFIANCE_MIN_GOBELET


MODEL_PATH = ROOT / "ia" / "model" / "best.pt"
WINDOW_NAME = "Prise avec boucle fermee ArUco"
ARUCO_BRAS_ID = 4
ERREUR_MAX_DEG = 2.0
MAX_CORRECTIONS = 4
CORRECTION_SERVO_MAX = 6.0
NB_MESURES_ARUCO = 15
DELAI_MESURE_ARUCO = 2.0


def orientation_marqueur(marker_coins):
    coin_0 = marker_coins[0][0]
    coin_1 = marker_coins[0][1]
    x_0, y_0 = pixel_to_cm(float(coin_0[0]), float(coin_0[1]))
    x_1, y_1 = pixel_to_cm(float(coin_1[0]), float(coin_1[1]))
    return math.degrees(math.atan2(y_1 - y_0, x_1 - x_0)) % 360


def moyenne_circulaire(angles):
    sinus = sum(math.sin(math.radians(angle)) for angle in angles)
    cosinus = sum(math.cos(math.radians(angle)) for angle in angles)
    return math.degrees(math.atan2(sinus, cosinus)) % 360


def erreur_angulaire(cible, mesure):
    return ((cible - mesure + 180) % 360) - 180


def gain_local(commande):
    """Gain réel local : degrés de pince par degré de commande servo."""
    for index in range(1, len(CALIBRATION_SERVO)):
        servo_1, pince_1 = CALIBRATION_SERVO[index - 1]
        servo_2, pince_2 = CALIBRATION_SERVO[index]
        if servo_1 <= commande <= servo_2:
            return (pince_2 - pince_1) / (servo_2 - servo_1)

    if commande < CALIBRATION_SERVO[0][0]:
        servo_1, pince_1 = CALIBRATION_SERVO[0]
        servo_2, pince_2 = CALIBRATION_SERVO[1]
    else:
        servo_1, pince_1 = CALIBRATION_SERVO[-2]
        servo_2, pince_2 = CALIBRATION_SERVO[-1]
    return (pince_2 - pince_1) / (servo_2 - servo_1)


def afficher_alignement(camera, detecteur, message, duree, collecter=False):
    mesures = []
    debut = time.time()

    while time.time() - debut < duree:
        ok, frame = camera.read()
        if not ok:
            continue

        coins, ids, _ = detecteur.detectMarkers(frame)
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, coins, ids)
            for marker_id, marker_coins in zip(ids.flatten(), coins):
                if int(marker_id) == ARUCO_BRAS_ID:
                    angle = (
                        orientation_marqueur(marker_coins)
                        + OFFSET_PINCE_MARQUEUR_DEG
                        + DECALAGE_CALIBRATION_SERVO_DEG
                    ) % 360
                    if collecter:
                        mesures.append(angle)
                    cv2.putText(
                        frame, f"pince mesuree={angle:.1f} deg", (15, 65),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2,
                    )
                    break

        cv2.putText(
            frame, message, (15, 32), cv2.FONT_HERSHEY_SIMPLEX,
            0.65, (0, 165, 255), 2,
        )
        cv2.imshow(WINDOW_NAME, frame)
        cv2.waitKey(1)

        if collecter and len(mesures) >= NB_MESURES_ARUCO:
            break

    return mesures


def attendre_aligne(robot, camera, detecteur, timeout=15):
    debut = time.time()
    while time.time() - debut < timeout:
        ligne = robot.lire()
        if ligne:
            print("Arduino >", ligne)
            if ligne == "ALIGNE":
                return True
            if ligne == "ERREUR":
                return False

        afficher_alignement(
            camera, detecteur, "ROTATION DE LA BASE...", 0.03,
        )
    return False


def aligner_base(robot, camera, detecteur, angle_cible, commande_initiale):
    commande = float(commande_initiale)

    for tentative in range(1, MAX_CORRECTIONS + 1):
        print(f"Alignement {tentative}/{MAX_CORRECTIONS} : servo={commande:.2f}°")
        robot.envoyer_rotation(commande)

        if not attendre_aligne(robot, camera, detecteur):
            print("Rotation non confirmée : poursuite en boucle ouverte")
            return commande, False

        # Laisse disparaître les vibrations avant de mesurer le marqueur.
        afficher_alignement(camera, detecteur, "STABILISATION...", 0.35)
        mesures = afficher_alignement(
            camera,
            detecteur,
            "MESURE ARUCO 4...",
            DELAI_MESURE_ARUCO,
            collecter=True,
        )

        if not mesures:
            print("ArUco 4 invisible : poursuite en boucle ouverte")
            return commande, False

        angle_mesure = moyenne_circulaire(mesures)
        erreur = erreur_angulaire(angle_cible, angle_mesure)
        print(
            f"Cible pince={angle_cible:.2f}°, mesure={angle_mesure:.2f}°, "
            f"erreur={erreur:+.2f}°"
        )

        if abs(erreur) <= ERREUR_MAX_DEG:
            print("Alignement visuel validé")
            return commande, True

        gain = gain_local(commande)
        correction = erreur / gain
        correction = max(-CORRECTION_SERVO_MAX, min(CORRECTION_SERVO_MAX, correction))
        nouvelle_commande = max(0.0, min(180.0, commande + correction))

        if abs(nouvelle_commande - commande) < 0.5:
            print("Correction trop petite : arrêt de la boucle")
            return commande, True
        commande = nouvelle_commande

    print("Nombre maximal de corrections atteint")
    return commande, True


def main():
    model = YOLO(str(MODEL_PATH))
    dictionnaire = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    parametres = cv2.aruco.DetectorParameters()
    parametres.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    detecteur = cv2.aruco.ArucoDetector(dictionnaire, parametres)

    camera = cv2.VideoCapture(0)
    if not camera.isOpened():
        raise RuntimeError("Impossible d'ouvrir la caméra")

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, 1100, 700)
    cv2.moveWindow(WINDOW_NAME, 50, 50)

    robot = None
    commande_en_cours = False

    try:
        robot = Robot()
        print("S : aligner puis prendre le gobelet vide")
        print("Q ou Échap : quitter")

        while True:
            ligne = robot.lire()
            if ligne:
                print("Arduino >", ligne)
                if ligne in ("TERMINE", "ERREUR"):
                    commande_en_cours = False

            ok, frame = camera.read()
            if not ok:
                raise RuntimeError("Impossible de lire la caméra")

            coins_plateau, ids_plateau, _ = detecteur.detectMarkers(frame)
            markers = construire_dictionnaire_marqueurs(
                coins_plateau,
                ids_plateau,
            )
            repere_visible = repere_plateau_visible(markers)

            try:
                obtenir_transformation_plateau(markers)
                repere_disponible = True
            except RuntimeError:
                repere_disponible = False

            resultats = model.predict(
                frame,
                conf=CONFIANCE_MIN_GOBELET,
                verbose=False,
            )
            candidats = []

            for box in resultats[0].boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                etat = model.names[int(box.cls[0].item())]
                confiance = float(box.conf[0].item())
                px, py = int((x1 + x2) / 2), int(y2)
                if not repere_disponible:
                    continue

                position = analyse_position(px, py, markers)
                angle_servo = position["angle_servo"]
                distance = float(position["distance_robot"])
                valide = (
                    etat == "cup_empty"
                    and position["zone"] == "W"
                    and angle_servo is not None
                    and DISTANCE_MIN_PRISE_CM <= distance <= DISTANCE_MAX_PRISE_CM
                )
                couleur = (0, 255, 0) if valide else (0, 165, 255)

                if valide:
                    candidats.append({
                        "angle_servo": float(angle_servo),
                        "angle_cible": float(position["angle_cible_pince"]),
                        "distance": distance,
                        "confiance": confiance,
                        "boite": (x1, y1, x2, y2),
                    })

                cv2.rectangle(frame, (x1, y1), (x2, y2), couleur, 2)
                cv2.circle(frame, (px, py), 7, (255, 0, 255), -1)
                cv2.drawMarker(frame, (px, py), (255, 255, 255),
                               cv2.MARKER_CROSS, 22, 2)
                cv2.putText(frame, "POSITION", (px + 10, py - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 0, 255), 2)

            selection = max(candidats, key=lambda c: c["confiance"], default=None)
            if commande_en_cours:
                texte, couleur = "PRISE EN COURS", (0, 165, 255)
            elif not repere_disponible:
                texte, couleur = "INITIALISATION : MONTRER 0, 1, 2, 3", (0, 0, 255)
            elif selection is None:
                texte, couleur = "AUCUN GOBELET VIDE AVEC ANGLE VALIDE", (0, 0, 255)
            else:
                texte, couleur = "S : ALIGNER ET PRENDRE", (0, 255, 0)
                x1, y1, x2, y2 = selection["boite"]
                cv2.rectangle(frame, (x1 - 4, y1 - 4), (x2 + 4, y2 + 4), (255, 0, 255), 3)

            if repere_disponible and not repere_visible and not commande_en_cours:
                texte += " | REPERE MEMORISE"

            cv2.putText(frame, texte, (15, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.7, couleur, 2)
            cv2.imshow(WINDOW_NAME, frame)
            touche = cv2.waitKey(1) & 0xFF

            if touche in (ord("q"), 27):
                break

            if touche == ord("s") and selection is not None and not commande_en_cours:
                commande, boucle_fermee = aligner_base(
                    robot,
                    camera,
                    detecteur,
                    selection["angle_cible"],
                    selection["angle_servo"],
                )
                mode = "fermée" if boucle_fermee else "ouverte (ArUco invisible)"
                print(f"Prise lancée avec commande {commande:.2f}° en boucle {mode}")
                robot.envoyer(commande, selection["distance"])
                commande_en_cours = True

    except KeyboardInterrupt:
        print("\nArrêt demandé")
    finally:
        camera.release()
        cv2.destroyAllWindows()
        if robot is not None:
            robot.fermer()


if __name__ == "__main__":
    main()
