#!/usr/bin/env python

import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class sift_detector:
    def __init__(self):
        self.image_pub = rospy.Publisher("/sift/detected_features",Image, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.callback)

        self.img_template = cv2.imread('m200_template_hoz.png')
        self.gray_template = cv2.cvtColor(self.img_template, cv2.COLOR_BGR2GRAY)

        self.sift = cv2.xfeatures2d.SIFT_create()
        self.kp_template, self.des_template = self.sift.detectAndCompute(self.gray_template, None)
        self.bf = cv2.BFMatcher(cv2.NORM_L1, crossCheck=True)

    def callback(self,data):
        try:
            bgr_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        (rows,cols,channels) = bgr_img.shape

        img = bgr_img.copy()
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

        kp, des = self.sift.detectAndCompute(gray, None)
        matches = self.bf.match(self.des_template, des)
        matches = sorted(matches, key = lambda x:x.distance)

        keypoints = []
        for i,match in enumerate(matches):
            keypoints.append(kp[match.trainIdx])

        img_with_keypoints = cv2.drawKeypoints(gray, keypoints, img, flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        #cv2.imshow("Sift Features", bgr_img)
        #cv2.waitKey(1)
        
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(img_with_keypoints, "bgr8"))
        except CvBridgeError as e:
            print(e)
         
def main(args):
    rospy.init_node('sift_detector', disable_signals=True)
    ic = sift_detector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

if __name__ == '__main__':
   main(sys.argv)