

from typing import Callable
import numpy as np


def adjustThresholdForPrecision(probabilities: np.ndarray[np.float_], expected: np.ndarray[np.float_], threshold:float, desiredPrecision:float, update:Callable[[float],np.ndarray[np.float_]]= None):
    predictedLabels = (probabilities >= threshold).astype(int)
    precision = np.sum((predictedLabels == 1) & (expected == 1)) / np.sum(predictedLabels)
    while precision > desiredPrecision:
        threshold += 0.01  # Increase threshold
        if update is not None:
            probabilities = update(threshold)
        predictedLabels = (probabilities >= threshold).astype(int)
        precision = np.sum((predictedLabels == 1) & (expected == 1)) / np.sum(predictedLabels)
    return threshold