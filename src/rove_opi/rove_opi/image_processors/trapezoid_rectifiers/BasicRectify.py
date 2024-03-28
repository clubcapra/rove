from typing import List
import cv2
import numpy as np
from image_processors.trapezoid_rectifiers.TrapezoidRectifier import TrapezoidRectifier


class BasicRectify(TrapezoidRectifier):
    def __init__(self):
        super().__init__()
        
    def __call__(self, img: cv2.Mat, trapezoids:np.ndarray[np.float_]) -> List[cv2.Mat]:
        if trapezoids is None:
            return None
        warps = []
        for i, trapezoid in enumerate(trapezoids):
            # now that we have our rectangle of points, let's compute
            # the width of our new image
            (tl, tr, br, bl) = trapezoid
            widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
            widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
            # ...and now for the height of our new image
            heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
            heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
            # take the maximum of the width and height values to reach
            # our final dimensions
            maxWidth = max(int(widthA), int(widthB))
            maxHeight = max(int(heightA), int(heightB))
            # construct our destination points which will be used to
            # map the screen to a top-down, "birds eye" view
            dst = np.array([
                [0, 0],
                [maxWidth - 1, 0],
                [maxWidth - 1, maxHeight - 1],
                [0, maxHeight - 1]], dtype = "float32")
            # calculate the perspective transform matrix and warp
            # the perspective to grab the screen
            M = cv2.getPerspectiveTransform(np.float32(trapezoid), dst)
            warp = cv2.warpPerspective(img, M, (maxWidth, maxHeight))
            if self.debug:
                cv2.imshow(f'test{i}', warp)
            warps.append(warp)
        
        return warps