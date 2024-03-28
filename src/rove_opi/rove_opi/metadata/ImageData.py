class ImageData:
    def __init__(self):
        self.thresh = 100
        self.othresh = 60
        self.lastKey = -1
        self.minArea = 0.008
        self.maxArea = 0.85
        self.erode = 0.001
        self.maxMergeDistance = 20 / 1080
        self.grow = 1.03
        self.margin = 0.001
        self.ocrMinArea = 20 / 1080
        self.ocrMaxArea = 0.15
        self.lineWidth = 0.035
        self.lineBorder = 0.04