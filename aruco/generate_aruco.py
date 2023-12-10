from typing import List, Union
import cv2
import cv2.aruco as aruco
import numpy as np
import os

# Define the ArUco dictionary constants
DICT_4X4 = aruco.DICT_4X4_250
DICT_5X5 = aruco.DICT_5X5_250
DICT_6X6 = aruco.DICT_6X6_250
DICT_7X7 = aruco.DICT_7X7_250

# Mapping of dictionary constants to their string names
DICT_MAPPING = {
    DICT_4X4: "DICT_4X4",
    DICT_5X5: "DICT_5X5",
    DICT_6X6: "DICT_6X6",
    DICT_7X7: "DICT_7X7"
}


"""
Generate ArUco markers for the given IDs and tag types.

:param ids: List of IDs for the ArUco markers.
:param tag_types: ArUco tag type or a list of tag types to be used.
:param size: Size of the ArUco markers.
:param file_format: File format for the output images.
:param directory: Directory where the ArUco markers will be saved.
"""
def aruco_generator(ids: List[int], 
                    tag_types: Union[int, List[int]] = DICT_6X6, 
                    size: int = 200, 
                    file_format: str = 'png', 
                    directory: str = 'aruco_markers') -> None:
    
    if isinstance(tag_types, int):
        tag_types = [tag_types]

    # Create the directory if it doesn't exist
    if not os.path.exists(directory):
        os.makedirs(directory)
    
    # Loop through each tag type
    for tag_type in tag_types:
        # Get the ArUco dictionary
        aruco_dict = aruco.Dictionary_get(tag_type)
        tag_type_str = DICT_MAPPING[tag_type] # Get the string representation
        
        # Generate markers for each id
        for id in ids:
            # Generate the marker
            img = aruco.drawMarker(aruco_dict, id, size)
            
            # Construct the file name
            file_name = f"{tag_type_str}_id{id}.{file_format}"
            file_path = os.path.join(directory, file_name)
            
            # Save the marker to a file
            cv2.imwrite(file_path, img)
            
    print(f"Generated {len(ids) * len(tag_types)} ArUco markers in '{directory}' directory.")


# Example usage:
ids = [0, 1, 2, 3]
tag_types = [DICT_5X5] 
# Generate ArUco markers
aruco_generator(ids, tag_types, 200, "png", "DICT_5X5_250")
