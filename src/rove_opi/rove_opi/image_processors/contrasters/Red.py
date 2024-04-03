import cv2
from .Contraster import Contraster
from rove_opi.image_processors.normalizers.Normalizer import Normalizer


class Red(Contraster):
    def __init__(self, postNormalizer: Normalizer = None):
        super().__init__()
        self.postNormalizer = postNormalizer
        
    def __call__(self, img: cv2.Mat) -> cv2.Mat:
        res = img[:,:,2]
        if self.postNormalizer is not None:
            res = self.postNormalizer(res)
        self.debugImg(res)
        return res