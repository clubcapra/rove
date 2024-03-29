import json
from typing import List
import cv2
import numpy as np
import rclpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Polygon, Point32
from cv_bridge import CvBridge
from .lib.common import CONVERTED_PATH, KEY_ESC, KEY_LEFT, KEY_RIGHT, STATS_PATH, AccuracyStatsDict
from .image_processors import OPIFinder, contrasters, edge_detectors, edge_filters, normalizers, shape_identifiers, shape_postprocessors, shape_selectors, trapezoid_finders, trapezoid_rectifiers, thresholders
from .lib.utils import convert, ensureExists

class OPIProcessing(rclpy.Node):
  
    def __init__(self, finder:OPIFinder):
        super().__init__('opi_processing')
        self.subscriber = self.create_subscription(Image, 'image', self.receive_img, 1)
        self.publisher = self.create_publisher(Polygon, 'trapezoid', 10)
        self.bridge = CvBridge()
        self.finder = finder

    def receive_img(self, imgmsg:Image):
        try:
            img = self.bridge.imgmsg_to_cv2(imgmsg)
            cv2.imshow('OPI', img)
            best = None          
            _, (indices, trapezoids, scores) = self.finder.find2(img)
            if len(trapezoids) > 0:
                selectedScores = np.array([scores[i] for i in indices])
                
                bestIdx = np.where(selectedScores == selectedScores.max())
                print(trapezoids)
                best = trapezoids[bestIdx]
                points = []
                for p in best:
                    points.append(Point32(x = p[0], y = p[1]))
                polygon = Polygon()
                polygon.points = points
                self.publisher.publish(polygon)
        except Exception as e:
            self.get_logger().error('Error publishing PNG image: %s' % str(e))


def main(args=None):
    rclpy.init(args=args)
    
    ensureExists()
    convert()
    
    imgs = [cv2.imread(str(p)) for p in CONVERTED_PATH.iterdir()]
    
    finder = OPIFinder()
    finder.steps['normalize']['simple'] = normalizers.SimpleNormalizer()
    
    finder.steps['orangeness']['meanRGBOffset'] = contrasters.MeanRGBOffset()
    
    finder.steps['threshold']['simple'] = thresholders.SimpleThreshold(50)
    
    finder.steps['edgeDetect']['simple'] = edge_detectors.SimpleEdgeDetect()
    
    finder.steps['edgeFilter']['simple'] = edge_filters.SimpleEdgeFilter(0.008, 0.9, 0.001)
    
    finder.steps['postProcessShapes']['fillExpand'] = shape_postprocessors.FillPostProcess(finder.steps['edgeDetect']['simple'], finder.steps['edgeFilter']['simple'], 1.1)
    finder.steps['postProcessShapes']['rectMerge'] = shape_postprocessors.RectMergePostProcess(finder.steps['edgeFilter']['simple'], 1.1)
    
    finder.steps['findTrapezoids']['simple'] = trapezoid_finders.NearestContour()
    
    finder.steps['rectifyTrapezoids']['simple'] = trapezoid_rectifiers.BasicRectify()
    
    redContraster = contrasters.Red(normalizers.SimpleNormalizer())
    
    finder.steps['scoreShapes']['simple'] = shape_identifiers.BasicShapeIdentifier(0.035, 0.04, redContraster)
    
    # finder.steps['selectShapes']['logistic'] = LogisticRegressionSelector()
    
    factors1 = {
        'lineBorder' : 0.4,
        'noLineBorder' : 0.2,
        'topBottomBorders' : 0.1,
        'noTopBottomBorders' : 0.1,
        'bordersLighter' : 0.1,
        'noBordersLighter' : 0.1,
        'topOCR' : 0.5,
        'bottomOCR' : 0.8,
        'valid' : 1.51,
    }
    
    factors2 = {
        'lineBorder' : 0.5,
        'noLineBorder' : 0.1,
        'topBottomBorders' : 0.4,
        'noTopBottomBorders' : 0.1,
        'bordersLighter' : 0.2,
        'noBordersLighter' : 0.1,
        'topOCR' : 0.2,
        'bottomOCR' : 0.4,
        'valid' : 1.51,
    }
    
    finder.steps['selectShapes']['hardCoded'] = shape_selectors.HardCodedSelector(factors2, 1.51)
    finder.steps['selectShapes']['hardCoded'] = shape_selectors.HardCodedSelector(factors1, 1.51)
    with (STATS_PATH / 'accuracy.json').open('r') as rd:
        try:
            results: AccuracyStatsDict = json.load(rd)
            weights = {(k): v['f1_score'] for k, v in results['results'].items()}
            finder.steps['selectShapes']['aggressive'] = shape_selectors.AggressiveLowFalsePos(0.1, weights, 1.51)
            # finder.steps['selectShapes']['aggressive'].debug = True
            # vv = cinput("Calibration desired precision (defailt = 0.2): ]0 - 1]  ", float, lambda v: 0 < v <= 1)
            finder.steps['selectShapes']['aggressive'].calibrate(results, .5)
            
        except (AttributeError, KeyError):
            finder.steps['selectShapes']['aggressive'] = shape_selectors.AggressiveLowFalsePos(0.2, 1, 0.1)
            # finder.steps['selectShapes']['aggressive'].debug = True

    # rclpy.spin(publisher)
    opiNode = OPIProcessing(finder)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    # publisher.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
