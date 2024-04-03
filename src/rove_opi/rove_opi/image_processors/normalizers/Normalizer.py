import cv2

from rove_opi.image_processors.ImageProcessor import ImageProcessor



class Normalizer(ImageProcessor):
    def __init__(self):
        super().__init__()
        
    def __call__(self, img:cv2.Mat) -> cv2.Mat:
        pass