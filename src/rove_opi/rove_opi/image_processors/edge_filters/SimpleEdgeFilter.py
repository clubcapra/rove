from typing import Sequence, Tuple
import cv2
import numpy as np
from image_processors.edge_filters.EdgeFilter import EdgeFilter
from utils import compactRect, expandRect


class SimpleEdgeFilter(EdgeFilter):
    def __init__(self, minArea:float, maxArea:float, margin:float):
        super().__init__()
        self.minArea = minArea
        self.maxArea = maxArea
        self.margin = margin
        
    def __call__(self, img:cv2.Mat, cnts:Sequence[cv2.Mat]) -> Tuple[np.ndarray[np.bool_],np.ndarray[np.float_],np.ndarray[np.float_],np.ndarray[np.float_]]:
        height, width, _ = img.shape
        minArea = self.minArea * height * width
        maxArea = self.maxArea * height * width
        margin = self.margin * height
        
        areas = np.array([cv2.contourArea(contour) for contour in cnts])
        valid = (areas > minArea) & (areas < maxArea)
        if not np.any(valid):
            return valid, None, None, None
        
        rects = np.array([expandRect(cv2.minAreaRect(c)) if v else (0,0,0,0,0)  for v, c in zip(valid, cnts)])
        rectAreas = rects[:,2] * rects[:,3]
        valid &= rectAreas < maxArea
        
        if not np.any(valid):
            return valid, None, None, None
        
        boxes = np.array([cv2.boxPoints(compactRect(r)) if v else np.zeros((4,2),np.float_) for v, r in zip(valid, rects)])
        
        valid &= np.any((boxes > (margin, margin)) & (boxes < (width-margin, height-margin)), (1,2))
        
        return valid, rects, rectAreas, boxes