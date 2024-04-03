from pathlib import Path
from typing import Dict, List, TypedDict
import numpy as np


BASE_PATH = Path("src/rove_opi/rove_opi")
SAMPLES_PATH = BASE_PATH / Path("samples")
CONVERTED_PATH = BASE_PATH / Path("converted")
STATS_PATH = BASE_PATH / Path("stats")

PATHS = [SAMPLES_PATH, CONVERTED_PATH, STATS_PATH]

CACHE_PATH = BASE_PATH / Path("cache.npy")
CACHE_START_PATH = BASE_PATH / Path("cache_start.npy")

class AccuracyDict(TypedDict):
    true_pos : float
    false_neg : float
    true_neg : float
    false_pos : float
    precision : float
    recall : float
    f1_score : float
    
class AccuracyStatsDict(TypedDict):
    results: Dict[str, AccuracyDict]
    scores: Dict[str, List[float]]
    expected: Dict[str, List[float]]

KEY_LEFT = 81
KEY_UP = 82
KEY_RIGHT = 83
KEY_DOWN = 84
KEY_ESC = 27

ORANGE = np.array([54, 114, 238], np.int16)

