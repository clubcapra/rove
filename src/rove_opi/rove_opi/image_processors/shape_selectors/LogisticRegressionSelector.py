from typing import Dict, List, Tuple
import cv2
import numpy as np
from sklearn.linear_model import LogisticRegression
from image_processors.shape_selectors.ShapeSeletor import ShapeSelector


class LogisticRegressionSelector(ShapeSelector):
    def __call__(self, img: cv2.Mat, scores: List[Dict[str, float]], warps: List[cv2.Mat], trapezoids: np.ndarray[np.float_]) -> Tuple[List[int], List[np.ndarray[np.float_]]]:
        if scores is None:
            return None, None
        # Assuming scores are normalized
        scores_array = np.array([list(score.values()) for score in scores])
        model = LogisticRegression()
        model.fit(scores_array, range(len(scores)))

        # Classify trapezoids
        predictions = model.predict(scores_array)

        selected_indices = list(predictions//4)
        selected_trapezoids = trapezoids[selected_indices]

        return selected_indices, selected_trapezoids