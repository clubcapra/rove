from typing import List
import cv2
import numpy as np
from image_processors.ImageProcessor import ImageProcessor


class TrapezoidRectifier(ImageProcessor):
    def __init__(self):
        super().__init__()
        
    def __call__(self, img: cv2.Mat, trapezoids:np.ndarray[np.float_]) -> List[cv2.Mat]:
        pass