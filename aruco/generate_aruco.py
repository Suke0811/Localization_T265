import cv2
import cv2.aruco as aruco
import numpy as np

# Define the ArUco dictionary (Change as needed)
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)

# Specify the IDs of ArUco tags to generate
ids_to_generate = [0, 1, 2, 3, 4, 5]

# Size of the ArUco marker images (in pixels)
marker_size = 200

for id in ids_to_generate:
    # Generate the marker
    img = aruco.drawMarker(aruco_dict, id, marker_size)

    # Save the marker to a file
    cv2.imwrite(f"aruco_marker_{id}.png", img)

    # Optionally display the generated marker
    cv2.imshow(f"Marker ID: {id}", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
