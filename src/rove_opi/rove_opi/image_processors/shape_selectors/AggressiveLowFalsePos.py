

from math import nan
from numbers import Number
from typing import Dict, List, Tuple, Union, overload
from cv2 import Mat
from matplotlib import pyplot as plt
import numpy as np
from common import AccuracyStatsDict
from image_processors.shape_selectors.ShapeSeletor import ShapeSelector
from prediction import adjustThresholdForPrecision


class AggressiveLowFalsePos(ShapeSelector):
    def __init__(self, initialThresholds:Union[Dict[str, float], float], weights:Union[Dict[str, float], float], initialThreshold:float):
        super().__init__()
        self.initialThresholds = initialThresholds
        self.thresholds = initialThresholds
        self.weights = weights
        self.initialThreshold = initialThreshold
        self.threshold = initialThreshold
        
    def calibrate(self, results:AccuracyStatsDict, desiredPrecision:float):
        if isinstance(self.initialThresholds, Number):
            self.initialThresholds: dict[str, float] = {k:self.initialThresholds for k in results['results'].keys()}
            self.thresholds: dict[str, float] = self.initialThresholds
            
        if isinstance(self.weights, Number):
            self.weights: dict[str, float] = {k:self.weights for k in results['results'].keys()}
            
        f = None
        def adjust(prec: float):
            global f
            for parameterName, initial, scores, expected in zip(results['results'].keys(), self.initialThresholds.values(), results['scores'].values(), results['expected'].values()):
                    
                if parameterName == 'valid':
                    continue
                self.thresholds[parameterName] = adjustThresholdForPrecision(np.array(scores), np.array(expected), 0.1, prec)
            
        
            f = None
            for k, scores in results['scores'].items():
                if f is None:
                    f = [{} for _ in range(len(scores))]
                for i, s in enumerate(scores):
                    f[i][k] = s
            
            res = np.array([self.calculateFactor(ff) for ff in f], np.float_)
            return res
        r = adjust(0.1)
        self.threshold = adjustThresholdForPrecision(r,  np.array(results['expected']['valid'], np.float_), self.initialThreshold, desiredPrecision, adjust)
        if self.debug:
            for k, w, t in zip(self.weights.keys(), self.weights.values(), self.thresholds.values()):
                print(f"{k.ljust(18)}: weight: {str(round(w, 3)).ljust(5)} thresh: {str(round(t, 3)).ljust(5)}")
                
            e = results['expected']['valid']
            s = np.array(f)
        print(f"threshold: {round(self.threshold, 3)}")
                    
                
                
        
    def calculateFactor(self, scores: Dict[str, float]) -> float:
        total = 0
        if isinstance(self.initialThresholds, Number):
            self.initialThresholds: dict[str, float] = {k:self.initialThresholds for k in scores.keys()}
            self.thresholds: dict[str, float] = self.initialThresholds
            
        if isinstance(self.weights, Number):
            self.weights: dict[str, float] = {k:self.weights for k in scores.keys()}
        s = 0
        for k, v in scores.items():
            if k == 'valid' or 'OCR' in k:
                continue
            if v == nan:
                continue
            if self.thresholds[k] == nan:
                continue
            if self.weights[k] == nan:
                continue
            s += self.weights[k]
            ss = 1
            if k.startswith('no'):
                ss = -1
            
            total += (v >= self.thresholds[k]) * self.weights[k] * ss
        total += 1 - (abs(2 - scores['topOCR']) / 2) * self.weights['topOCR']
        total += 1 - (abs(4 - scores['bottomOCR']) / 4) * self.weights['bottomOCR']
            
        return total / s
            
    def __call__(self, img: Mat, scores: List[Dict[str, float]], warps: List[Mat], trapezoids: np.ndarray[np.float_]) -> Tuple[List[int], List[np.ndarray[np.float_]], List[float]]:
        indices: List[int] = []
        if scores is None or warps is None or trapezoids is None:
            return [], [], []
        res: List[np.ndarray[np.float_]] = []
        factors: List[float] = []
        for i, s in enumerate(scores):
            f = self.calculateFactor(s)
            factors.append(f)
            s['valid'] = f
            if f >= 0.5:
                indices.append(i)
        idx = set()
        for i in indices:
            idx.add(i//4)
        for i in idx:
            res.append(trapezoids[i])
        if self.debug:
            # plt.hist(f)
            # plt.show()
            print(factors)
                
                
        return indices, res, factors
            