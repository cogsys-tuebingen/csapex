#!/usr/bin/env python


import os
import roslib; roslib.load_manifest('robot_detection')
import rospy
import sys
import cv
import cv2
import numpy

rospy.init_node('antifilter_test_batch', anonymous=True)

def batch(keypoint, descriptor, bg, path):
    print path
    for frame in os.listdir(path):
        arg = ["rosrun robot_detection antifilter_test", keypoint, descriptor, bg, path + "/" + frame]
        os.system(' '.join(arg))

if __name__ == '__main__':
    if len(sys.argv) != 5:
        raise RuntimeError("usage: " + sys.argv[0] + " <keypoint-type> <descriptor-type> <background-img> <path-to-frames>")
    batch(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4])