from typing import Sequence, Tuple
import cv2
import numpy as np
from image_processors.edge_filters.EdgeFilter import EdgeFilter
from image_processors.shape_postprocessors.ShapePostProcessor import ShapePostProcessor
from utils import compactRect, findOverlappingRotatedRectangles, minAreaRectRotatedRects


class RectMergePostProcess(ShapePostProcessor):
    def __init__(self, edgeFilter:EdgeFilter, grow:float):
        super().__init__()
        self.edgeFilter = edgeFilter
        self.grow = grow
    
    def __call__(self, img:cv2.Mat, rects:np.ndarray[np.float_], valid:np.ndarray[np.bool_], cnts:Sequence[cv2.Mat]) -> Tuple[np.ndarray[np.bool_],np.ndarray[np.float_],np.ndarray[np.float_],np.ndarray[np.float_],Sequence[cv2.Mat]]:
        if valid is None or not np.any(valid):
            return valid, None, None, None, cnts
        r = rects.copy()
        
        minRects = np.array([minAreaRectRotatedRects(pair) for pair in findOverlappingRotatedRectangles(img, r, valid, self.grow)], ndmin=2)
        if self.debug and minRects.shape[-1] != 0:
            dbg = img.copy()
            for rr in minRects:
                dbg = cv2.drawContours(dbg, [np.int_(cv2.boxPoints(compactRect(rr)))], 0, (0,255,0), 2)
                
            for rr in r:
                dbg = cv2.drawContours(dbg, [np.int_(cv2.boxPoints(compactRect(rr)))], 0, (255,0,0), 1)
                
            self.debugImg(dbg)
        
        areas = minRects[:,2:4].prod(1)
        if len(minRects) != 1:
            boxes = np.array([np.int_(cv2.boxPoints(compactRect(rr))) for rr in minRects])
        else:
            boxes = np.empty((1,0), np.float_)
        return valid, minRects, areas, boxes, cnts