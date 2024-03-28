from typing import Dict, List, Tuple

import cv2
from image_processors.ImageProcessor import ImageProcessor
from utils import debugScore


class ShapeIdentifier(ImageProcessor):
    def __init__(self):
        super().__init__()
        
    def debugScore(self, score:Dict[str,float], ndigits=2):
        if not self.debug:
            return
        debugScore(score, ndigits)
        
    def __call__(self, img: cv2.Mat, warps: List[cv2.Mat]) -> Tuple[List[Dict[str, float]], List[cv2.Mat]]:
        pass