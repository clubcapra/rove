from typing import Sequence
import cv2
import imutils
from image_processors.edge_detectors.EdgeDetector import EdgeDetector


class SimpleEdgeDetect(EdgeDetector):
    def __init__(self):
        super().__init__()

    def __call__(self, threshold:cv2.Mat) -> Sequence[cv2.Mat]:
        # Find the contours
        cnts = cv2.findContours(threshold, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        return cnts