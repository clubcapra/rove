import math
from typing import Dict, List, Tuple
import cv2
import numpy as np
from image_processors.contrasters.Contraster import Contraster
from image_processors.shape_identifiers.ShapeIdentifier import ShapeIdentifier
import pytesseract


class BasicShapeIdentifier(ShapeIdentifier):
    def __init__(self, lineWidth:float, lineBorder:float, contraster: Contraster):
        super().__init__()
        self.lineWidth = lineWidth
        self.lineBorder = lineBorder
        self.contraster = contraster
        
    def __call__(self, img: cv2.Mat, warps: List[cv2.Mat], rectAreas:np.ndarray[np.float_]) -> Tuple[List[Dict[str, float]], List[cv2.Mat]]:
        if warps is None:
            return None, None
        # List for valid results
        results = []
        scores = []
        
        # Iterate over the warped (straightned) images
        for i, warp in enumerate(warps):
            # Get the red channel normalized
            contrast = self.contraster(warp)
            
            
            # Iterate over orientations
            for orientation in range(4):
                # Rotate the image
                rotated = np.rot90(contrast, orientation)
                h,w = rotated.shape
                
                # Split top and bottom half
                split = [rotated[:h//2],rotated[h//2:]]
                
                
                score = {
                    'lineBorder' : 0.0,
                    'noLineBorder' : 0.0,
                    'topBottomBorders' : 0.0,
                    'noTopBottomBorders' : 0.0,
                    'bordersLighter' : 0.0,
                    'noBordersLighter' : 0.0,
                    'topOCR' : 0.0,
                    'bottomOCR' : 0.0,
                }
                
                # Update line values to warped resolution
                lineWidth = max(int(h*self.lineWidth), 2)
                lineBorder = max(int(h*self.lineBorder), 2)
                
                # Prepare middle line checks
                half = h//2
                lower = half-lineWidth//2
                upper = half+lineWidth//2 + 1
                line = rotated[lower:upper]
                border1 = rotated[lower-lineBorder:lower]
                border2 = rotated[upper:upper+lineBorder]
                lm = line.mean(dtype=np.float32)
                bm1 = border1.mean(dtype=np.float32)
                bm2 = border2.mean(dtype=np.float32)
                bm = (bm1+bm2)/2
                
                if self.debug:
                    dbg = cv2.drawContours(cv2.cvtColor(rotated.copy(), cv2.COLOR_GRAY2BGR), [np.array([[0,lower-lineBorder],[w, lower-lineBorder],[w,lower],[0,lower]])], 0, (0,255,0), 1)
                    dbg = cv2.drawContours(dbg, [np.array([[0,upper+lineBorder],[w, upper+lineBorder],[w,upper],[0,upper]])], 0, (0,255,0), 1)
                    dbg = cv2.drawContours(dbg, [np.array([[0,lower],[w, lower],[w,upper],[0,upper]])], 0, (255,0,0), 1)
                    self.debugImg(dbg)
                
                # If the line and the area around the line are close, decrease score. Otherwise, use the difference as a base score
                if not math.isclose(bm,lm, rel_tol=0.15, abs_tol=4):
                    score['lineBorder'] = (bm-lm)/255
                else:
                    score['noLineBorder'] = 1
                
                # Check if the top border is similar to bottom border
                if math.isclose(bm1,bm2, rel_tol=0.05, abs_tol=5):
                    score['topBottomBorders'] = 1
                else:
                    score['noTopBottomBorders'] = abs(bm1-bm2)/255
                    
                # Ensure borders are lighter than the line
                if lm < bm1 and lm < bm2:
                    score['bordersLighter'] = 1
                else:
                    score['noBordersLighter'] = 1
                
                # # If score is very low, don't bother with OCR, saves time
                # if score < 0:
                #     continue
                
                # If score is low, verify with OCR
                ocrCount = 0
                for ii, s, k in zip(range(2), split, ['topOCR', 'bottomOCR']):
                    txt = pytesseract.image_to_string(s, lang='eng',config='--psm 10 --oem 3 -c tessedit_char_whitelist=0123456789')
                    txt:str
                    txt.rstrip('\n')
                    score[k] = len(txt)
                    if txt == '':
                        continue
                    ocrCount+=1

        #     # if not b.debug:
        #     #     b.destroyWindow()
        #     while True:
        #         self.stepIndex = len(self.behaviors.choices) - 1 if self.stepIndex == 0 else self.stepIndex - 1
        #         b = self.behaviors.getStepNumber(self.stepIndex)
        #         if b is not None:
        #             break
        # if k == KEY_UP:
        #     # if not b.debug:
        #     #     b.destroyWindow()
        #     while True:
        #         self.stepIndex = (self.stepIndex + 1) % len(self.behaviors.choices)
        #         b = self.behaviors.getStepNumber(self.stepIndex)
        #         if b is not None:
        #             break
                    # cv2.imshow(f'test{i},{orientation},{ii}', s)
                    if self.debug:
                        print(f"Found [{i},{orientation},{ii}] '{txt}'")
                results.append(rotated)
                scores.append(score)
        if self.debug:
            for s, r in zip(scores, results):
                self.debugScore(s)
                self.debugImg(r)
                while cv2.waitKey(100) == -1:
                    pass
        return scores, results