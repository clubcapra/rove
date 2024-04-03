from __future__ import annotations
from operator import index
from pathlib import Path
from typing import Collection, Dict, Generic, List, Literal, LiteralString, Self, SupportsIndex, Tuple, TypeVar, TypedDict, Union, ValuesView, overload, override
import cv2

import numpy as np
from numpy import float_, ndarray
from metadata.AccuracyData import AccuracyData
import json
import itertools


# _K = TypeVar("_K", str, covariant=True)
# T = TypeVar("T")

# class WarpDict(TypedDict, Generic[T]):
#     img: str
#     results: Dict[T, float]
#     expected: Dict[T, float]
    

# class DataDict(TypedDict, Generic[T]):
#     warps:List[WarpDict[T]]
    

# class AccuracyDataDict(TypedDict, Generic[T], total=False):
#     names: List[T]
#     data: DataDict[T]
    
    
class WarpDict(TypedDict):
    img: str
    results: Dict[str, float]
    expected: Dict[str, float]
    

class DataDict(TypedDict):
    warps:List[WarpDict]
    

class AccuracyDataDict(TypedDict, total=False):
    names: List[str]
    data: List[DataDict]



class JSONAccuracyData(AccuracyData):
    def __init__(self):
        
        """ JSON hierarchy
        {
            "names" : [
                // score names ...
            ],
            "data" : [
                {
                    "warps" : [
                        {
                            "img" : "path/to/warp.png",
                            "results" : {
                                // score data ...
                            },
                            "expected" : {
                                // score data ...
                            },
                        },
                    ],
                },
            ],
        }
        """
        # self.data:List[Dict[Literal["warps"], List[Union[
        #     Dict[Literal["img"], str], 
        #     Dict[Literal["results"], Dict[str, float]],
        #     Dict[Literal["expected"], Dict[str, float]]
        # ]]]] = []
        # self.data:Dict[Literal["names", "data"], Union[
        #     List[str], # names
        #     List[
        #         Dict[Literal["warps"], List[Dict[ # data
        #             Literal["img", "results", "expected"], 
        #             Union[str, Dict[str, float]] # img : str | results : Dict[str, float] | expected : Dict[str, float]
        #         ]]]
        #     ]
        # ]] = []
        super().__init__()
        self.data:AccuracyDataDict = {}
        self.data["names"] = []
        self.data["data"] = []
        self.images:List[cv2.Mat] = []
    
    @classmethod
    @overload
    # @override
    def load(path:str) -> Self:
        return JSONAccuracyData.load(Path(path))
    
    @classmethod
    @overload
    # @override
    def load(path:Path) -> Self:
        res = JSONAccuracyData()
        res.data = json.load(path)
        
        i = 0
        for d in res.data:
            for dd in d["warps"]:
                res.images[i] = cv2.imread(dd["img"])
                i += 1
        
        return res
    
    @overload
    # @override
    def save(self, path:str):
        self.save(Path(path))
    
    @overload
    # @override
    def save(self, path:Path):
        warpsPath = path.with_name(path.stem + "_warps")
        warpsPath.mkdir(parents=True, exist_ok=True)
        
        imgPaths = []
        for i, img in enumerate(self.getImages()):
            p = str(warpsPath / f"{i}.png")
            imgPaths.append(p)
            cv2.imwrite(p, img)
            
    @property
    def names(self) -> Collection[str]:
        return self.data["names"]
    
    # @override
    def updateNames(self):
        names = set(self.names)
        for d in self.data["data"]:
            for dd in d["warps"]:
                results = set(dd["results"])
                expected = set(dd["expected"])
                
                # Grab names that aren't in the data
                adds1 = names - (names & results)
                adds2 = names - (names & expected)

                # Add the missing keys
                for a, s in [(adds1, dd["results"]), (adds2, dd["expected"])]:
                    s[a] = 0.0
                
                # Grab names that aren't in the list of names
                rems1 = results - names
                rems2 = expected - names
                
                # Remove the excess keys
                for r, s in [(rems1, dd["results"]), (rems2, dd["expected"])]:
                    s.pop(r)
    
    # @override
    def addName(self, name:str):
        if name not in self.names:
            self.data["names"].append(name)
        
        self.updateNames()
        
    # @override
    def removeName(self, name:str):
        if name in self.names:
            self.data["names"].remove(name)

        self.updateNames()
        
    def getScores(self, _key:str) -> np.ndarray:
        return np.array([[ddd["results"][_key] for ddd in dd] for dd in [d["warps"] for d in self.data["data"]]]).flatten()
    
    def setScores(self, _key:str, scores:np.ndarray):
        self.addName(_key)
        i = 0
        for d in self.data["data"]:
            for dd in d["warps"]:
                dd["results"][_key] = scores[i]
                i+=1
    
    @overload
    def getScore(self, _key:str, _index: SupportsIndex) -> float:
        imgIndex = _index // 4
        warpIndex = _index % 4
        return self.data["data"][index(imgIndex)]["warps"][index(warpIndex)]["results"][_key]
    
    @overload
    def setScore(self, _key:str, _index: SupportsIndex, score:float):
        self.addName(_key)
        imgIndex = _index // 4
        warpIndex = _index % 4
        self.data["data"][index(imgIndex)]["warps"][index(warpIndex)]["results"][_key] = score
    
    def getExpecteds(self, _key:str) -> np.ndarray:
        return np.array([[ddd["expected"][_key] for ddd in dd] for dd in [d["warps"] for d in self.data["data"]]]).flatten()
    
    def setExpecteds(self, _key:str, expected:np.ndarray):
        self.addName(_key)
    
    @overload
    def getExpected(self, _key:str, _index: SupportsIndex) -> np.ndarray:
        imgIndex = _index // 4
        warpIndex = _index % 4
        return self.data["data"][index(imgIndex)]["warps"][index(warpIndex)]["expected"][_key]
    
    @overload
    def setExpected(self, _key:str, _index: SupportsIndex, expected:np.ndarray):
        self.addName(_key)
        imgIndex = _index // 4
        warpIndex = _index % 4
        self.data["data"][index(imgIndex)]["warps"][index(warpIndex)]["results"][_key] = expected
    
    def getImages(self) -> List[cv2.Mat]:
        return self.images
    