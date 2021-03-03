#!/usr/bin/env python
from tkinter import*
from PIL import ImageTk, Image
import sys
import rospy
import cv2
import numpy as np
import time
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class InterfaceGui:
    def __init__(self):
        self.bridge = CvBridge()
        #self.saved_frame = np.zeros((1080,1920),dtype='uint8')
        #self.saved_frame = cv2.cvtColor(self.saved_frame, cv2.COLOR_GRAY2BGR)
        self.image_sub = rospy.Subscriber("/aruco/detected_roi",Image,self.callback)
        self.root = Tk()
        self.root.title('ARUCO GUI')
        #root.iconbitmap('~/git/3DVisionMobileManipulation/interface_gui/novo_logo.ico')
        self.button_quit = Button(self.root, text="Exit", command=self.root.quit)
        self.button_quit.pack()
        self.root.mainloop()

    def callback(self,data):
        
        try:
            bgr_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        #my_img = ImageTk.PhotoImage(Image.open("test_detected_roi.png"))
        button_img = Button(self.root, text="Display Image", command=lambda: self.display_image(bgr_img))
        button_img.pack()

    def display_image(self, img):
        my_label = Label(image=img)
        my_label.pack()



def main(args):
    rospy.init_node('gui_interface')
    gui = InterfaceGui()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)