from typing import Sequence, Tuple
import cv2
import numpy as np
from rove_opi.image_processors.ImageProcessor import ImageProcessor


class EdgeFilter(ImageProcessor):
    def __init__(self):
        super().__init__()
        
    def __call__(self, img:cv2.Mat, cnts:Sequence[cv2.Mat]) -> Tuple[np.ndarray,np.ndarray,np.ndarray,np.ndarray]:
        """

        Args:
            img (cv2.Mat): 
            cnts (Sequence[cv2.Mat]): 

        Returns:
            Tuple[np.ndarray[np.bool_],np.ndarray[np.float_],np.ndarray[np.float_],np.ndarray[np.float_]]: 
        """
        pass