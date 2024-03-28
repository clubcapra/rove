from typing import Sequence, Tuple
import cv2
import numpy as np
from image_processors.edge_detectors.EdgeDetector import EdgeDetector
from image_processors.edge_filters.EdgeFilter import EdgeFilter
from image_processors.shape_postprocessors.ShapePostProcessor import ShapePostProcessor
from utils import compactRect


class FillPostProcess(ShapePostProcessor):
    def __init__(self, edgeDetector:EdgeDetector, edgeFilter:EdgeFilter, grow:float):
        super().__init__()
        self.edgeDetector = edgeDetector
        self.edgeFilter = edgeFilter
        self.grow = grow
    
    def __call__(self, img:cv2.Mat, rects:np.ndarray[np.float_], valid:np.ndarray[np.bool_], cnts:Sequence[cv2.Mat]) -> Tuple[np.ndarray[np.bool_],np.ndarray[np.float_],np.ndarray[np.float_],np.ndarray[np.float_], Sequence[cv2.Mat]]:
        if valid is None or not np.any(valid):
            return valid, None, None, None, cnts
        h, w, _ = img.shape
        mask = np.zeros((h,w), np.uint8)
        for r in rects[valid]:
            (xx,yy),(ww,hh),rr = compactRect(r)
            
            
            # Expand the boxes to allow 2 seperated shapes to become one
            expanded = cv2.boxPoints(((xx,yy),(ww*self.grow, hh*self.grow),rr))
            expanded = np.int_(expanded)
            mask = cv2.drawContours(mask, [expanded], 0, (255), -1)
        cnts2 = self.edgeDetector(mask)
        valid2, rects, rectAreas, boxes = self.edgeFilter(img, cnts2)
        return valid, rects, rectAreas, boxes, cnts