import os
from pathlib import Path
import shutil
from typing import Dict, Sequence, Tuple, Union
from colorama import Fore
import cv2
from matplotlib import pyplot as plt
import numpy as np
from .common import BASE_PATH, CACHE_PATH, CACHE_START_PATH, CONVERTED_PATH, PATHS, SAMPLES_PATH


def ensureExists():
    for p in PATHS:
        p.mkdir(parents=True, exist_ok=True)
    
    
    if not CACHE_PATH.exists():
        shutil.copyfile(str(CACHE_START_PATH), str(CACHE_PATH))

def convert():
    for p in SAMPLES_PATH.iterdir():
        newPath = CONVERTED_PATH / (p.stem + '.png')
        if newPath.exists():
            continue
        im = cv2.imread(str(p))
        if im is None:
            continue
        cv2.imwrite(str(newPath), im)


def expandRect(rect:Tuple[Tuple[float,float],Tuple[float,float],float]) -> Tuple[float,float,float,float,float]:
    (x, y), (w, h), r = rect
    return x,y,w,h,r

def compactRect(rect:Union[np.ndarray, Tuple[float,float,float,float,float]]) -> Tuple[Tuple[float,float],Tuple[float,float],float]:
    """

    Args:
        rect (Union[np.ndarray[np.float_], Tuple[float,float,float,float,float]]): 

    Returns:
        Tuple[Tuple[float,float],Tuple[float,float],float]: 
    """
    return (rect[0], rect[1]), (rect[2], rect[3]), rect[4]

# Function to find overlapping rotated rectangles
def findOverlappingRotatedRectangles(img: cv2.Mat, rectangles:np.ndarray, valid: np.ndarray, grow:float):
    """

    Args:
        img (cv2.Mat): 
        rectangles (np.ndarray[np.float_]): 
        valid (np.ndarray[np.bool_]): 
        grow (float): 

    Returns:
        _type_: 
    """
    overlapping_pairs = []
    grown = rectangles.copy()
    grown[:,2:4] *= grow
    oldValid = valid.copy()
    valid[:] = False
    for i, rect1, grown1 in zip(range(len(rectangles)), rectangles, grown):
        if not oldValid[i]:
            continue
        for j, rect2, grown2 in zip(range(i+1, len(rectangles)), rectangles[i+1:], grown[i+1:]):
            if not oldValid[j]:
                continue
            inside = cv2.rotatedRectangleIntersection(compactRect(rect1), compactRect(rect2))
            if inside[0] != cv2.INTERSECT_NONE and len(inside[1]) == 4:
                continue
            intersection = cv2.rotatedRectangleIntersection(compactRect(grown1), compactRect(grown2))
            
            if intersection[0] == cv2.INTERSECT_PARTIAL:
                overlapping_pairs.append((grown1, grown2))
                valid[i] = True
                valid[j] = True
                
            
            # dbg = img.copy()
            # if intersection[0] == cv2.INTERSECT_PARTIAL:
            #     overlapping_pairs.append((grown1, grown2))
            #     dbg = cv2.drawContours(dbg, [np.int_(cv2.boxPoints(compactRect(grown1)))], 0, (0,255,0), 2)
            #     dbg = cv2.drawContours(dbg, [np.int_(cv2.boxPoints(compactRect(grown2)))], 0, (0,255,0), 2)
            # else:
            #     dbg = cv2.drawContours(dbg, [np.int_(cv2.boxPoints(compactRect(rect1)))], 0, (0,0,255), 2)
            #     dbg = cv2.drawContours(dbg, [np.int_(cv2.boxPoints(compactRect(rect2)))], 0, (0,0,255), 2)
            #     dbg = cv2.drawContours(dbg, [np.int_(cv2.boxPoints(compactRect(grown1)))], 0, (255,0,0), 1)
            #     dbg = cv2.drawContours(dbg, [np.int_(cv2.boxPoints(compactRect(grown2)))], 0, (255,0,0), 1)
            # cv2.imshow('overlap', dbg)
            # while cv2.waitKey(100) == -1:
            #     pass

    return np.array(overlapping_pairs)

# Function to compute the minimum area rectangle for a set of rotated rectangles
def minAreaRectRotatedRects(rects: np.ndarray):
    """

    Args:
        rects (np.ndarray[np.float_]): 

    Returns:
        _type_: 
    """
    points = []
    for rect in rects:
        box = cv2.boxPoints(compactRect(rect))
        points.extend(box)
    return expandRect(cv2.minAreaRect(np.array(points)))

def debugScore(score:Dict[str,float], ndigits=2):
    s = ''
    for n, v in score.items():
        no = n.startswith('no')
        v = round(v, ndigits)
        if not no:
            s += f'\n{n}: '
        else:
            s += ' | '
        
        color = Fore.GREEN if no != (v > 0.5) else Fore.RED
        s += f'{color}{v}{Fore.RESET}'
    print(s)
    
def calculate_metrics(tp, fn, tn, fp):
    precision = tp / (tp + fp)
    recall = tp / (tp + fn)
    f1_score = 2 * (precision * recall) / (precision + recall)
    return precision, recall, f1_score

def debugMetrics(score:np.ndarray, expected:np.ndarray):
    """

    Args:
        score (np.ndarray[np.bool_]): 
        expected (np.ndarray[np.bool_]): 

    Returns:
        _type_: 
    """
    dbg = ''
    truePos = (expected == True) & (score == True)
    falsePos = (expected == False) & (score == True)
    trueNeg = (expected == False) & (score == False)
    falseNeg = (expected == True) & (score == False)
    metrics1 = {
        'true_pos' : truePos[expected==True],
        'false_neg' : falseNeg[expected==True],
        'true_neg' : trueNeg[expected==False],
        'false_pos' : falsePos[expected==False],
    }
    for name, val in metrics1.items():
        val = val.sum()
        ratio = val / len(score)
        metrics1[name] = ratio
        ratio = round(ratio * 100, 1)
        dbg += f"{name.ljust(9)}: {str(val).rjust(3)}/{str(len(score)).ljust(3)} {ratio}%\n"
    precision, recall, f1_score = calculate_metrics(truePos.sum(), falseNeg.sum(), trueNeg.sum(), falsePos.sum())
    metrics2 = {
        "precision": precision,
        "recall": recall,
        "f1_score": f1_score
    }
    for name, val in metrics2.items():
        dbg += f"{name.ljust(9)}: {str(round(val, 3))}\n"
    
    print(dbg)
    return { **metrics1, **metrics2 }


def arrangeBars(x:np.ndarray, *ys:np.ndarray):
    spacing = 0.1
    count = len(ys)
    maxWidth = 1-spacing
    width = maxWidth / count
    for offset, y in zip(np.arange(0, maxWidth, width), ys):
        plt.bar(x+offset, y, width)
    