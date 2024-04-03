from __future__ import annotations

from abc import ABCMeta, abstractmethod
from pathlib import Path
from typing import Collection, List, Self, SupportsIndex, Tuple, overload
import cv2

import numpy as np




class AccuracyData(ABCMeta):
    def __init__(self):
        pass
    
    @classmethod
    @abstractmethod
    @overload
    def load(path:str) -> Self: ...
    
    @classmethod
    @abstractmethod
    @overload
    def load(path:Path) -> Self: ...
    
    @abstractmethod
    @overload
    def save(self, path:str): ...
    
    @abstractmethod
    @overload
    def save(self, path:Path): ...
    
    @abstractmethod
    @property
    def names(self) -> Collection[str]: ...
    
    @abstractmethod
    def updateNames(self): ...
    
    @abstractmethod
    def addName(self, name:str): ...
    
    @abstractmethod
    def removeName(self, name:str): ...
    
    @abstractmethod
    def getScores(self, _key:str) -> np.ndarray: ...
    
    @abstractmethod
    def setScores(self, _key:str, scores:np.ndarray): ...
    
    @abstractmethod
    @overload
    def getScore(self, _key:str, _index: SupportsIndex) -> float: ...
    
    @abstractmethod
    @overload
    def setScore(self, _key:str, _index: SupportsIndex, score:float): ...
    
    @overload
    def getScore(self, _key:str, _imgIndex: SupportsIndex, _warpIndex: SupportsIndex) -> float:
        return self.getScore(_key, _imgIndex * 4 + _warpIndex)
    
    @overload
    def setScore(self, _key:str, _imgIndex: SupportsIndex, _warpIndex: SupportsIndex, score:float):
        self.setScore(_key, _imgIndex * 4 + _warpIndex, score)
    
    @abstractmethod
    def getExpecteds(self, _key:str) -> np.ndarray: ...
    
    @abstractmethod
    def setExpecteds(self, _key:str, expected:np.ndarray): ...
    
    @abstractmethod
    @overload
    def getExpected(self, _key:str, _index: SupportsIndex) -> np.ndarray: ...
    
    @abstractmethod
    @overload
    def setExpected(self, _key:str, _index: SupportsIndex, expected:np.ndarray): ...
    
    @overload
    def getExpected(self, _key:str, _imgIndex: SupportsIndex, _warpIndex: SupportsIndex) -> np.ndarray:
        return self.getExpected(_key, _imgIndex * 4 + _warpIndex)
    
    @overload
    def setExpected(self, _key:str, _imgIndex: SupportsIndex, _warpIndex: SupportsIndex, expected:np.ndarray):
        self.setExpected(_key, _imgIndex*4 + _warpIndex, expected)
    
    @abstractmethod
    def getImages(self) -> List[cv2.Mat]: ...
    
    @overload
    def getImage(self, _index: SupportsIndex) -> cv2.Mat:
        return self.getImages()[_index]
    
    @overload
    def getImage(self, _imgIndex: SupportsIndex, _warpIndex: SupportsIndex) -> cv2.Mat:
        return self.getImages()[_imgIndex * 4 + _warpIndex]
    