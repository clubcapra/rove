from typing import Sequence
import cv2
import numpy as np
from rove_opi.image_processors.ImageProcessor import ImageProcessor


class TrapezoidFinder(ImageProcessor):
    def __init__(self):
        super().__init__()
        
    def __call__(self, img:cv2.Mat, cnts: Sequence[cv2.Mat], valid: np.ndarray, rects: np.ndarray, boxes:np.ndarray) -> np.ndarray:
        """

        Args:
            img (cv2.Mat): 
            cnts (Sequence[cv2.Mat]): 
            valid (np.ndarray[np.bool_]): 
            rects (np.ndarray[np.float_]): 
            boxes (np.ndarray[np.float_]): 

        Returns:
            np.ndarray[np.float_]: 
        """
        pass