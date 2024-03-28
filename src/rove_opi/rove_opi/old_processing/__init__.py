from itertools import chain
import math
import cv2
import numpy as np
import pytesseract
import imutils

from common import ORANGE


from metadata import ImageData


def noProcess(img:cv2.Mat, data:ImageData):
    return img

def normalize(img: cv2.Mat, data: ImageData):
    mn = img.min((0,1))
    mx = img.max((0,1))
    dt = mx-mn
    fimg = img.astype(np.float32)
    return ((fimg[:,:]-mn)/dt*255).astype(np.uint8)

def gray(img: cv2.Mat, data: ImageData, preview=True):
    v = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    res = normalize(v, data)
    
    if preview:
        for i, col in enumerate(['blue','green','red']):
            cv2.imshow(col, normalize(img[:,:,i], data))
    return res

def red(img: cv2.Mat, data: ImageData, preview=True):
    r = normalize(img[:,:,2], data)
    return r

def redAdaptiveThresh(img: cv2.Mat, data: ImageData, preview=True):
    r:cv2.Mat = red(img, data, False)
    
    
    return r
    

def thresh(img: cv2.Mat, data: ImageData, preview=True):
    v = normalize(img, data)
    r = cv2.threshold(v, data.thresh, 255, cv2.THRESH_BINARY)[1]
    if preview:
        return cv2.cvtColor(r, cv2.COLOR_BGR2GRAY)
    return r

def orangeness1(img: cv2.Mat, data: ImageData, preview=True):
    v = normalize(img, data)
    # v = img
    dist = np.abs((v.astype(np.int16) - ORANGE))
    nn = dist.mean(2).astype(np.uint16)
    if preview:
        n = normalize(nn, data)
        return cv2.cvtColor(n, cv2.COLOR_GRAY2BGR)
    return nn

def orangeness1Thresh(img: cv2.Mat, data: ImageData, preview=True):
    v = orangeness1(img, data, False).astype(np.uint8)
    if preview:
        if data.lastKey == ord('w'):
            data.othresh += 1
        if data.lastKey == ord('s'):
            data.othresh -= 1
        print(f'othresh = {data.othresh}')
    r = cv2.threshold(v, data.othresh, 255, cv2.THRESH_BINARY)[1]
    if preview:
        return cv2.cvtColor(r, cv2.COLOR_GRAY2BGR)
    return r

def orangeness2(img: cv2.Mat, data: ImageData, preview=True):
    # v = normalize(img, data)
    v = img
    
    hsv = cv2.cvtColor(v, cv2.COLOR_BGR2HSV)
    orange = cv2.cvtColor(ORANGE[np.newaxis, np.newaxis].astype(np.uint8), cv2.COLOR_BGR2HSV)
    hue = hsv[:,:,0].astype(np.int16)
    cv2.imshow('test1', hsv[:,:,0])
    cv2.imshow('test2', hsv[:,:,1])
    cv2.imshow('test3', hsv[:,:,2])
    ohue = orange[0,0,0].astype(np.int16)
    orangeness = (np.abs(hue-ohue)*2)
    cv2.imshow('test4', ((orangeness * hsv[:,:,1])/2).astype(np.uint8))
    orangeness[orangeness == 256] -= 1
    orangeness.astype(np.uint8)
    return orangeness

def filterThresh(img: cv2.Mat, data: ImageData, preview=True):
    if preview:
        if data.lastKey == ord('w'):
            data.grow += 0.005
        if data.lastKey == ord('s'):
            data.grow -= 0.005
    # Get orange sections
    v = orangeness1Thresh(img, data, False)
    h,w = v.shape[:2]
    
    # Find the contours
    cnts = cv2.findContours(v.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    
    # Get absolute min and max area
    minArea = data.minArea * h*w
    maxArea = data.maxArea * h*w
    
    margin = data.margin * h
    
    mask = np.zeros((h,w), np.uint8)
    candidates = []
    res = img.copy()
    
    def filterContour(contour:np.ndarray):
        # Filter by area
        a = cv2.contourArea(contour)
        if a <= minArea:
            return False, None, None
        if a > maxArea:
            return False, None, None
        # Get angled rect
        r = cv2.minAreaRect(contour)
        (xx, yy), (ww, hh), rr = r
        
        if ww*hh <= minArea:
            return False, None, None
        if ww*hh > maxArea:
            return False, None, None
        box = cv2.boxPoints(r)
        
        if np.any(np.logical_or(box <= (margin, margin), box >= (w-margin, h-margin))):
            return False, None, None
        
        box = np.int_(box)
        return True, box, r
    
    for c in cnts:
        # Filter contours
        select, box, r = filterContour(c)
        
        # Skip rejected
        if not select:
            continue
        
        # Get rect components
        (xx,yy),(ww,hh),rr = r
        
        # Add the candidate to the list
        candidates.append(c)
        
        # Draw some shapes
        if preview:
            res = cv2.drawContours(res, [c], -1, (0,255,0), 1)
            res = cv2.drawContours(res, [box], 0, (255,0,0),1)
        
        # Expand the boxes to allow 2 seperated shapes to become one
        expanded = cv2.boxPoints(((xx,yy),(ww*data.grow, hh*data.grow),rr))
        expanded = np.int_(expanded)
        mask = cv2.drawContours(mask, [expanded], 0, (255),-1)
    
    # Grab the contours of the expanded mask
    mcnts = cv2.findContours(mask.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    mcnts = imutils.grab_contours(mcnts)
    
    # Convert candidates into np array
    candidates = np.array([vvv for vvv in chain(*[values for values in [vv for vv in candidates]])]).squeeze()
    trapezoids = []
    for c in mcnts:
        # Filter contours
        select, box, r = filterContour(c)
        
        # Skip rejected
        if not select:
            continue
        # Get rect components
        (xx,yy),(ww,hh),rr = r
        
        # Draw the rough outline
        if preview:
            res = cv2.drawContours(res, [box], 0, (0,0,255),2)
        
        # Find the trapezoid contained in this expanded shape
        trapezoid = np.zeros((4,2), np.int_)
        
        # Get the clossest contour point for each corner
        for i, corner in enumerate(box):
            # Get all distances
            m = np.linalg.norm(candidates[:] - corner, axis=1, keepdims=True)
            # Find minimum
            idx = np.where(m == m.min())
            # Grab minimum's contour coordinates
            if idx[0].size > 1:
                trapezoid[i] = candidates[idx[0][0]]
            else:
                trapezoid[i] = candidates[idx[0]]
        
        # Draw the resulting trapezoid
        if preview:
            res = cv2.drawContours(res, [trapezoid], 0, (255,255,0),3)
        trapezoids.append(trapezoid)
    
    if preview:
        return res
    return res, trapezoids
    
def straighten(img: cv2.Mat, data: ImageData, preview=True):
    prev, trapezoids = filterThresh(img, data, preview=False)
    if preview:
        cv2.imshow('test', prev)
    
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
        if preview:
            cv2.imshow(f'test{i}', warp)
        warps.append(warp)
    
    if preview:
        return warp
    return warps, trapezoids
    
def ocr(img: cv2.Mat, data: ImageData, preview=True):
    warps, trapezoids = straighten(img,data,preview=False)
    hh,ww,_ = img.shape
    
    # Get absolute variables
    minArea = hh*ww*data.ocrMinArea
    maxArea = hh*ww*data.ocrMaxArea
    lineWidth = int(hh*data.lineWidth)
    lineBorder = int(hh*data.lineBorder)
    if data.lastKey != -1:
        print(f"Min ocr: {data.ocrMinArea*100}% ({minArea}) | Max ocr: {data.ocrMaxArea*100}% ({maxArea})\nWidth: {data.lineWidth*100}% ({lineWidth}) | Border: {data.lineBorder*100}% ({lineBorder})")
    
    # List for valid results
    results = []
    
    # Iterate over the warped (straightned) images
    for i, warp in enumerate(warps):
        # Get the red channel normalized
        contrast = red(normalize(warp,data), data, preview=False)
        if preview:
            cv2.imshow('test', contrast)
        
        # Iterate over orientations
        for orientation in range(4):
            # Rotate the image
            rotated = np.rot90(contrast, orientation)
            h,w = rotated.shape
            
            # Split top and bottom half
            split = [rotated[:h//2],rotated[h//2:]]
            
            
            score = 0
            
            # Update line values to warped resolution
            lineWidth = max(int(h*data.lineWidth), 2)
            lineBorder = max(int(h*data.lineBorder), 2)
            
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
            
            if preview:
                dbg = cv2.drawContours(cv2.cvtColor(rotated, cv2.COLOR_GRAY2BGR), [np.array([[0,lower-lineBorder],[w, lower-lineBorder],[w,lower],[0,lower]])], 0, (0,255,0), 1)
                dbg = cv2.drawContours(dbg, [np.array([[0,upper+lineBorder],[w, upper+lineBorder],[w,upper],[0,upper]])], 0, (0,255,0), 1)
                dbg = cv2.drawContours(dbg, [np.array([[0,lower],[w, lower],[w,upper],[0,upper]])], 0, (255,0,0), 1)
                cv2.imshow(f'test {i}', dbg)
            
            # If the line and the area around the line are close, decrease score. Otherwise, use the difference as a base score
            if not math.isclose(bm,lm, rel_tol=0.15, abs_tol=4):
                score += bm-lm
            else:
                score -= 30
            
            # Check if the top border is similar to bottom border
            if math.isclose(bm1,bm2, rel_tol=0.05, abs_tol=5):
                score += 10
            else:
                score -= abs(bm1-bm2)*1.5
                
            # Ensure borders are lighter than the line
            if lm < bm1 and lm < bm2:
                score += 20
            else:
                score -= 40
            
            # If score is very low, don't bother with OCR, saves time
            if score < 0:
                continue
            
            # If score is low, verify with OCR
            if score < 50:
                
                found = True
                for ii, s in enumerate(split):
                    txt = pytesseract.image_to_string(s, lang='eng',config='--psm 10 --oem 3 -c tessedit_char_whitelist=0123456789')
                    if txt == '':
                        score -= 20
                        found = False
                        break
                    score += 10

                    # cv2.imshow(f'test{i},{orientation},{ii}', s)
                    print(f"Found [{i},{orientation},{ii}] '{txt}'")
                if found:
                    score += 20
            print(f"score [{i},{orientation}]: {score}")
            if score > 50:
                if not preview:
                    return True, trapezoids[i]
                results.append(rotated)
    if preview:
        for i, result in enumerate(results):
            cv2.imshow(f'result {i}', result)
    else:
        return False, None
    return warp
    
def canny(img: cv2.Mat, data: ImageData, preview=True):
    v = orangeness1Thresh(img, data, False)
    return cv2.Canny(v,0,1)

def final(img: cv2.Mat, data: ImageData, preview=True):
    success, trapezoid = ocr(img, data, preview=False)
    res = img.copy()
    if success:
        res = cv2.drawContours(res, [trapezoid], 0, (255,0,0), 3)
    return res