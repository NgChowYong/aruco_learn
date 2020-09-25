#!/usr/bin/env python2

# general python package
import rospy
import socket
import threading
import time
import math
import numpy as np

# ros package
from localization.msg import Camera_Data
import tf

import sys

global tag_no
global tag_data_pos
global tag_data_ori
tag_no = []
tag_data_pos = []
tag_data_ori = []


# callback function for rospy to used
def callback(data):
    global tag_no, tag_data_pos, tag_data_ori

    for i in range(len(data.Obstacle_Pose.poses)):
        flag = 0
        for j in range(len(tag_no)):
            if data.Obstacle_ID[i] == tag_no[j]:
               flag = 1
               tag_data_pos[j].append(data.Obstacle_Pose.poses[i].position)
               tag_data_ori[j].append(data.Obstacle_Pose.poses[i].orientation)
        if flag == 0:
            tag_no.append(data.Obstacle_ID[i])
            tag_data_pos.append([])
            tag_data_ori.append([])


if __name__ == '__main__':

    rospy.init_node('SendDataToSTM', anonymous=True)
    # start collect data
    rospy.Subscriber("Camera_Data", Camera_Data, callback)

    # file to save path data
    global sense_file
    sense_file = "/home/icmems/WALLE_project/catkin_ws/src/localization/err_100cm.txt"
    sense_file_ = open(sense_file,"w")

    while not rospy.is_shutdown():
        if len(tag_no) != 0 and len(tag_data_pos[0]) > 200:
            break
        pass

    print(len(tag_data_pos[0]))
    for i in range(len(tag_no)):
        for j in range(len(tag_data_pos[i])):
            str_ = str(tag_data_pos[i][j].x) + "," + str(tag_data_pos[i][j].y) +","+ str(tag_data_pos[i][j].z) \
                    + "," + str(tag_data_ori[i][j].x) +","+ str(tag_data_ori[i][j].y) \
                    + "," + str(tag_data_ori[i][j].z) +","+ str(tag_data_ori[i][j].w) +"\n"
            #print(tag_data_pos)
            sense_file_.write(str_)

    sense_file_.close()
    # flag for closing main code
    end_flag = 1
    # rospy.spin()
    print(sys.argv)
