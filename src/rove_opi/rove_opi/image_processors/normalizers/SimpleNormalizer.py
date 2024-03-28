import cv2
import numpy as np
from image_processors.normalizers.Normalizer import Normalizer


class SimpleNormalizer(Normalizer):
    def __init__(self):
        super().__init__()
        
    def __call__(self, img:cv2.Mat) -> cv2.Mat:
        mn = img.min((0,1))
        mx = img.max((0,1))
        dt = mx-mn
        fimg = img.astype(np.float32)
        res = ((fimg[:,:]-mn)/dt*255).astype(np.uint8)
        self.debugImg(res)
        return res