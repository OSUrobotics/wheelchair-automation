#!/usr/bin/env python
import roslib
roslib.load_manifest('wheelchair_follower')
import rospy, message_filters
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Joy, Image
import numpy as np
import std_msgs.msg
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge, CvBridgeError
import cv2

class vfolllower:
    def __init__(self):
        self.bridge = CvBridge()
        self.camera = rospy.Subscriber('/img_throttle',Image,self.cam_callback)
    def cam_callback(self,stream):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(stream, "bgr8")
            height = np.size(cv_image, 0)
            width = np.size(cv_image, 1)

            bgdModel = np.zeros((1,65),np.float64)
            fgdModel = np.zeros((1,65),np.float64)

            #Mask Creation
            # a = np.ones((480,70))
            # a = a*cv2.GC_PR_BGD
            # b = np.ones((480,500))
            # b = b*cv2.GC_PR_FGD
            # c = np.ones((480,70))
            # c = c*cv2.GC_PR_BGD

            # output = np.concatenate((a,b),axis=1)
            # mask = np.concatenate((output,c),axis=1)

            # print mask.shape
            mask = np.ones(cv_image.shape[:2],np.uint8)
            rect = (120,1,450,479)
            # rect = (161,79,150,150)
            cv2.grabCut(cv_image,mask,rect,bgdModel,fgdModel,2,cv2.GC_INIT_WITH_RECT)
            mask2 = np.where((mask==2)|(mask==0),0,1).astype('uint8')
            img_cut = cv_image*mask2[:,:,np.newaxis]


        except CvBridgeError as e:
            print(e)

        # CvPoint mid_bottom, mid_top
        mid_bottom_x = np.size(cv_image, 1)/2
        mid_bottom_y = 0
        mid_top_x = np.size(cv_image, 1)/2
        mid_top_y = np.size(cv_image, 0)
        # cv2.rectangle(cv_image, (120,480),(450,0),(255,0,0), 2)
        cv2.rectangle(cv_image, (120,1),(450,479),(255,0,0), 3)
        cv2.line(cv_image, (mid_bottom_x,mid_bottom_y), (mid_top_x,mid_top_y), (0,0,255), 2);
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)

if __name__ == '__main__':
	rospy.init_node('vfolllower',log_level=rospy.DEBUG)
	vfolllower = vfolllower()
	rospy.spin()
