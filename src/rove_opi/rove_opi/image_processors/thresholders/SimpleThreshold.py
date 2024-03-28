import cv2

from image_processors.thresholders.Thresholder import Thresholder


class SimpleThreshold(Thresholder):
    def __init__(self, threshold:int):
        super().__init__()
        self.threshold = threshold
    
    def __call__(self, orangeness:cv2.Mat) -> cv2.Mat:
        res = cv2.threshold(orangeness, self.threshold, 255, cv2.THRESH_BINARY)[1]
        self.debugImg(res)
        return res