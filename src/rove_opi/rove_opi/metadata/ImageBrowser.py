from typing import Callable, List
import cv2
import numpy as np

from common import KEY_DOWN, KEY_ESC, KEY_LEFT, KEY_RIGHT, KEY_UP
from metadata import ImageData


class ImageBrowser:
    def __init__(self, imgs: List[cv2.Mat]):
        self.imgs = imgs
        self.data = list(ImageData() for _ in imgs)
        self.index = 0
        self.processingMethods:List[Callable[[cv2.Mat, ImageData], cv2.Mat]] = []
        self.methodIndex = 0
        self.colors = []
        self.img = imgs[0]
        
        cv2.namedWindow("main")
        cv2.namedWindow("disp")
        cv2.setMouseCallback('disp', self.mouseEvent)
        cv2.imshow('disp', self.img)
        
        
        
    def mouseEvent(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDBLCLK:
            self.colors.append(self.img[y,x])
            arr = np.array(self.colors)
            print(arr.mean(0))

    def loop(self):
        k = cv2.waitKey(10)
        if k == -1:
            return
        
        if k == KEY_LEFT:
            self.index = len(self.imgs) - 1 if self.index == 0 else self.index - 1
        if k == KEY_RIGHT:
            self.index = (self.index + 1) % len(self.imgs)
        if k == KEY_DOWN:
            self.methodIndex = len(self.processingMethods) - 1 if self.methodIndex == 0 else self.methodIndex - 1
        if k == KEY_UP:
            self.methodIndex = (self.methodIndex + 1) % len(self.processingMethods)
        if k == KEY_ESC:
            exit(0)
        
        data = self.data[self.index]
        data.lastKey = k
        method = self.processingMethods[self.methodIndex]
        img = self.imgs[self.index]
        self.img = method(img, data)
        cv2.imshow('disp', self.img)
        cv2.imshow('main', img)