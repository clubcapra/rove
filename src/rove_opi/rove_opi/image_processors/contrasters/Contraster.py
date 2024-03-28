import cv2

from image_processors.ImageProcessor import ImageProcessor


class Contraster(ImageProcessor):
    def __init__(self):
        super().__init__()
        
    def __call__(self, img: cv2.Mat) -> cv2.Mat:    
        pass