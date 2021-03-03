#!/usr/bin/env python
import sys
import rospy
import cv2
import numpy as np
import time
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import Tkinter as tk # for python2.7
from PIL import Image, ImageTk

def callback(data):
    bridge = CvBridge()
    try:
        img = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)

rospy.init_node('gui_interface')
image_sub = rospy.Subscriber("/camera/color/image_raw",Image, callback)
#/aruco/detected_roi

gui = tk.Tk()
gui.title('ARUCO GUI')
gui.geometry('400x600')

bgr_img = ImageTk.PhotoImage(Image.open("detected_roi.png"))
my_label = tk.Label(image=bgr_img)
my_label.pack()
#button_img = Button(gui, text="Display Image", command=lambda: display_image(bgr_img))
#button_img.pack()
button_quit = tk.Button(gui, text="Exit", command=gui.quit)
button_quit.pack()

try:
    rospy.spin()
    gui.mainloop()
    
except KeyboardInterrupt:
    gui.destroy()
    print("Shutting down")
