

from math import nan
from numbers import Number
from typing import Dict, List, Tuple, Union
import cv2
import numpy as np
from image_processors.shape_selectors.ShapeSeletor import ShapeSelector


class HardCodedSelector(ShapeSelector):
    def __init__(self, weights:Union[Dict[str, float], float], threshold: float):
        super().__init__()
        self.weights = weights
        self.threshold = threshold
        
    def calculateFactor(self, scores: Dict[str, float]) -> float:
        total = 0
        if isinstance(self.weights, Number):
            self.weights: dict[str, float] = {k:self.weights for k in scores.keys()}
        s = 0
        for k, v in scores.items():
            if k == 'valid' or 'OCR' in k:
                continue
            if v == nan:
                continue
            if self.weights[k] == nan:
                continue
            s += self.weights[k]
            ss = 1
            if k.startswith('no'):
                ss = -1
            
            total += v * self.weights[k] * ss
        total += 1 - (abs(2 - scores['topOCR']) / 2) * self.weights['topOCR']
        total += 1 - (abs(4 - scores['bottomOCR']) / 4) * self.weights['bottomOCR']
            
        return total / s
        
    def __call__(self, img: cv2.Mat, scores: List[Dict[str, float]], warps: List[cv2.Mat], trapezoids: np.ndarray[np.float_]) -> Tuple[List[int], List[np.ndarray[np.float_]], List[float]]:
        indices: List[int] = []
        if scores is None or warps is None or trapezoids is None:
            return [], [], []
        res: List[np.ndarray[np.float_]] = []
        factors: List[float] = []
        for i, ss in enumerate(scores):
            s = self.calculateFactor(ss)
            factors.append(s)
            if s >= self.threshold:
                indices.append(i//4)
                res.append(trapezoids[i//4])
                ss['valid'] = s
        
        if self.debug:
            dbg = img.copy()
            for i, t in zip(indices, res):
                dbg = cv2.drawContours(dbg, [t], 0, (255,0,0), 2)
                (x, y), (_, _), _ = cv2.minAreaRect(t)
                dbg = cv2.putText(dbg, f"score: {factors[i]}", (x, y), cv2.FONT_HERSHEY_SCRIPT_SIMPLEX, 1, (0,255,255), 2)
            
            self.debugImg(dbg)
                
        
        return indices, res, factors
    
    