from typing import Sequence
import cv2

from image_processors.ImageProcessor import ImageProcessor


class EdgeDetector(ImageProcessor):
    def __init__(self):
        super().__init__()

    def __call__(self, threshold:cv2.Mat) -> Sequence[cv2.Mat]:
        pass    