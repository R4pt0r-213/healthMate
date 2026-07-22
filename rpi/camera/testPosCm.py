import cv2
import numpy as np
from pathlib import Path


# Charger la calibration
H = np.load(Path(__file__).with_name("homography.npy"))


# Inverser la transformation :
# cm -> pixels
H_inverse = np.linalg.inv(H)


# Position demandée sur le plateau (en cm)
X_cm = 3
Y_cm = 19


# Point en cm
point_cm = np.array(
    [[[X_cm, Y_cm]]],
    dtype=np.float32
)


# Conversion cm -> pixel caméra
point_pixel = cv2.perspectiveTransform(
    point_cm,
    H_inverse
)


x_pixel = int(point_pixel[0][0][0])
y_pixel = int(point_pixel[0][0][1])


print(f"Position plateau : X={X_cm} cm Y={Y_cm} cm")
print(f"Position caméra : x={x_pixel} px y={y_pixel} px")


# Affichage caméra
cap = cv2.VideoCapture(0)

while True:

    ret, frame = cap.read()

    if not ret:
        break


    # Dessine le point
    cv2.circle(
        frame,
        (x_pixel, y_pixel),
        10,
        (0,0,255),
        -1
    )


    cv2.imshow(
        "Point plateau",
        frame
    )


    if cv2.waitKey(1) == 27:
        break


cap.release()
cv2.destroyAllWindows()