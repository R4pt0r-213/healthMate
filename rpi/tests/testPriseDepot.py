from pathlib import Path
import sys
import time

import cv2
from ultralytics import YOLO


ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from camera.plateau import (
    DEPOT_X_CM,
    DEPOT_Y_CM,
    DISTANCE_MAX_PRISE_CM,
    DISTANCE_MIN_PRISE_CM,
    ROBOT_MARKER_ID,
    analyse_position,
    construire_dictionnaire_marqueurs,
    get_robot_position,
    obtenir_marqueur,
    obtenir_transformation_plateau,
    repere_plateau_visible,
)
from robot.communication import Robot
from ia.config import CONFIANCE_MIN_GOBELET


MODEL_PATH = ROOT / "ia" / "model" / "best.pt"
WINDOW_NAME = "Test prise puis depot"


def main():
    model = YOLO(str(MODEL_PATH))
    dictionnaire = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    detecteur = cv2.aruco.ArucoDetector(
        dictionnaire,
        cv2.aruco.DetectorParameters(),
    )
    camera = cv2.VideoCapture(0)
    if not camera.isOpened():
        raise RuntimeError("Impossible d'ouvrir la caméra")

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, 1100, 700)
    cv2.moveWindow(WINDOW_NAME, 50, 50)

    robot = None
    commande_en_cours = False
    debut_commande = 0.0

    print("S : prendre le gobelet vide puis le déposer dans D")
    print("Q ou Échap : quitter")

    try:
        robot = Robot()

        while True:
            ligne = robot.lire()
            if ligne:
                print("Arduino >", ligne)
                if ligne in ("TERMINE", "ERREUR"):
                    commande_en_cours = False

            if commande_en_cours and time.time() - debut_commande > 90:
                print("Délai Arduino dépassé")
                commande_en_cours = False

            ok, frame = camera.read()
            if not ok:
                raise RuntimeError("Impossible de lire la caméra")

            coins, ids, _ = detecteur.detectMarkers(frame)
            markers = construire_dictionnaire_marqueurs(coins, ids)
            repere_visible = repere_plateau_visible(markers)
            repere_disponible = False
            angle_depot = None
            distance_depot = None

            try:
                transformation = obtenir_transformation_plateau(markers)
                marqueur_robot = obtenir_marqueur(
                    ROBOT_MARKER_ID,
                    markers,
                )
                repere_disponible = True
                depot = get_robot_position(
                    DEPOT_X_CM,
                    DEPOT_Y_CM,
                    marqueur_robot,
                    transformation,
                )
                if depot["angle_servo"] is not None:
                    angle_depot = float(depot["angle_servo"])
                    distance_depot = float(depot["distance_cm"])
            except (RuntimeError, ValueError):
                pass

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
                point_x, point_y = int((x1 + x2) / 2), int(y2)
                if not repere_disponible or angle_depot is None:
                    continue

                position = analyse_position(point_x, point_y, markers)
                angle = position["angle_servo"]
                distance = float(position["distance_robot"])
                zone = position["zone"]

                valide = (
                    etat == "cup_empty"
                    and zone == "W"
                    and angle is not None
                    and DISTANCE_MIN_PRISE_CM <= distance <= DISTANCE_MAX_PRISE_CM
                )
                couleur = (0, 255, 0) if valide else (0, 165, 255)

                if valide:
                    candidats.append({
                        "angle": float(angle),
                        "distance": distance,
                        "confiance": confiance,
                        "boite": (x1, y1, x2, y2),
                    })

                if distance < DISTANCE_MIN_PRISE_CM:
                    servo = "trop proche"
                elif distance > DISTANCE_MAX_PRISE_CM:
                    servo = "trop loin"
                else:
                    servo = "hors angle" if angle is None else f"{angle:.1f} deg"
                texte = f"{etat} | zone={zone} | servo={servo} | d={distance:.1f} cm"
                cv2.rectangle(frame, (x1, y1), (x2, y2), couleur, 2)
                cv2.circle(frame, (point_x, point_y), 7, (255, 0, 255), -1)
                cv2.drawMarker(frame, (point_x, point_y), (255, 255, 255),
                               cv2.MARKER_CROSS, 22, 2)
                cv2.putText(frame, "POSITION", (point_x + 10, point_y - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 0, 255), 2)
                cv2.putText(frame, texte, (x1, max(25, y1 - 8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.48, couleur, 2)

            selection = max(candidats, key=lambda c: c["confiance"], default=None)

            if commande_en_cours:
                instruction, couleur = "CYCLE EN COURS - NE PAS APPROCHER", (0, 165, 255)
            elif not repere_disponible:
                instruction, couleur = "INITIALISATION : MONTRER 0, 1, 2, 3", (0, 0, 255)
            elif angle_depot is None:
                instruction, couleur = "ANGLE DU DEPOT IMPOSSIBLE", (0, 0, 255)
            elif selection is None:
                instruction, couleur = "AUCUN GOBELET VIDE DANS W AVEC ANGLE VALIDE", (0, 0, 255)
            else:
                instruction, couleur = "S : PRENDRE PUIS DEPOSER DANS D", (0, 255, 0)
                x1, y1, x2, y2 = selection["boite"]
                cv2.rectangle(frame, (x1 - 4, y1 - 4), (x2 + 4, y2 + 4), (255, 0, 255), 3)

            if repere_disponible and not repere_visible and not commande_en_cours:
                instruction += " | REPERE MEMORISE"

            cv2.putText(frame, instruction, (15, 32), cv2.FONT_HERSHEY_SIMPLEX,
                        0.65, couleur, 2)
            cv2.imshow(WINDOW_NAME, frame)

            touche = cv2.waitKey(1) & 0xFF
            if touche in (ord("q"), 27):
                break

            if touche == ord("s") and selection is not None and not commande_en_cours:
                print(
                    f"Prise servo={selection['angle']:.1f}°, d={selection['distance']:.1f} cm; "
                    f"dépôt servo={angle_depot:.1f}°, d={distance_depot:.1f} cm"
                )
                robot.envoyer_cycle(
                    selection["angle"], selection["distance"],
                    angle_depot, distance_depot,
                )
                commande_en_cours = True
                debut_commande = time.time()

    except KeyboardInterrupt:
        print("\nArrêt demandé")
    finally:
        camera.release()
        cv2.destroyAllWindows()
        if robot is not None:
            robot.fermer()


if __name__ == "__main__":
    main()
