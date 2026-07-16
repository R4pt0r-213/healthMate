import cv2
import numpy as np

cap = cv2.VideoCapture(0)

dictionary = cv2.aruco.getPredefinedDictionary(
    cv2.aruco.DICT_4X4_50
)

parameters = cv2.aruco.DetectorParameters()

# améliore la précision des coins
parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

detector = cv2.aruco.ArucoDetector(
    dictionary,
    parameters
)


print("Recherche des 4 marqueurs...")

while True:

    ret, frame = cap.read()

    if not ret:
        print("Erreur caméra")
        break


    corners, ids, _ = detector.detectMarkers(frame)


    if ids is not None:

        cv2.aruco.drawDetectedMarkers(
            frame,
            corners,
            ids
        )


        # Quand les 4 marqueurs sont visibles
        if len(ids) == 4:

            centres = {}

            for marker_id, marker in zip(ids, corners):

                centre = marker[0].mean(axis=0)

                centres[int(marker_id)] = centre


            # Vérifie qu'on a bien les IDs attendus
            if all(i in centres for i in [0, 1, 2, 3]):

                # Coordonnées caméra (pixels)
                points_camera = np.float32([
                    centres[3],   # haut gauche
                    centres[2],   # haut droit
                    centres[0],   # bas droit
                    centres[1]    # bas gauche
                ])


                # Coordonnées réelles du plateau (cm)
                points_plateau = np.float32([
                    [1.9, 1.9],
                    [40.1, 1.9],
                    [40.1, 27.6],
                    [1.9, 27.6]
                ])


                # Calcul homographie
                H = cv2.getPerspectiveTransform(
                    points_camera,
                    points_plateau
                )


                print("\nCalibration réussie !")
                print("Matrice H :")
                print(H)


                # Sauvegarde
                np.save(
                    "homography.npy",
                    H
                )

                print("\nMatrice sauvegardée dans homography.npy")

                break


    cv2.imshow(
        "Calibration ArUco",
        frame
    )


    # touche ESC
    if cv2.waitKey(1) == 27:
        break


cap.release()
cv2.destroyAllWindows()