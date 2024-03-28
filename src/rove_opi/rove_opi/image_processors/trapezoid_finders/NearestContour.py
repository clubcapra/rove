from itertools import chain
from typing import Sequence
import cv2
import numpy as np
from image_processors.trapezoid_finders.TrapezoidFinder import TrapezoidFinder


class NearestContour(TrapezoidFinder):
    def __init__(self):
        super().__init__()
        
    def __call__(self, img:cv2.Mat, cnts: Sequence[cv2.Mat], valid: np.ndarray[np.bool_], rects: np.ndarray[np.float_], boxes:np.ndarray[np.float_]) -> np.ndarray[np.float_]:
        if valid is None or not np.any(valid):
            return None
        res = None
        
        candidates = np.array([vvv for vvv in chain(*[values for values in [vv[1] for vv in filter(lambda c: c[0], zip(valid, cnts))]])]).squeeze()
        trapezoids = []
        if boxes is None:
            return None
        for b in boxes:
            # Draw the rough outline
            if self.debug:
                res = cv2.drawContours(img.copy(), [np.int_(b)], 0, (0,0,255),2)
            
            # Find the trapezoid contained in this expanded shape
            trapezoid = np.zeros((4,2), np.int_)
            
            # Get the clossest contour point for each corner
            for i, corner in enumerate(b):
                # Get all distances
                m = np.linalg.norm(candidates[:] - corner, axis=1, keepdims=True)
                # Find minimum
                idx = np.where(m == m.min())
                # Grab minimum's contour coordinates
                if idx[0].size > 1:
                    trapezoid[i] = candidates[idx[0][0]]
                else:
                    trapezoid[i] = candidates[idx[0]]
            
            # Draw the resulting trapezoid
            if self.debug:
                res = cv2.drawContours(res, [trapezoid], 0, (255,255,0),3)
            trapezoids.append(trapezoid)
            
        if self.debug and res is not None:
            cv2.imshow('NearestContour', res)
        
        return np.array(trapezoids)