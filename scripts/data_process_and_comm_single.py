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
from visualization_msgs.msg import Marker
import tf
from localization.srv import *

# for reading map only # not sure can be used or not
import cv2

# HOST = '192.168.43.77'  # Standard loopback interface address (localhost)
HOST = '192.168.0.199'  # Standard loopback interface address (localhost)
PORT = 11223  # Port to listen on (non-privileged ports are > 1023)


# callback function for rospy to used
def callback(data):
    global data_receive, data_receive_flag

    # data process
    global cali_tag
    global cali_tag_no
    global cali_flag
    global do_cali
    global robot_connection

    if cali_flag == 0:
        if do_cali == 1:
            # do data collection
            for i in range(len(data.Obstacle_Pose.poses)):
                for j in range(len(cali_tag_no)):
                    if data.Obstacle_ID[i] == cali_tag_no[j]:
                        cali_tag[j].append(data.Obstacle_Pose.poses[i].position)
    else:
        pass


def plane_calibration():
    global do_cali
    global cali_tag
    global cali_tag_no
    global cali_flag
    global Matrix
    global end_flag
    global correction_matrix_file
    global plane_file, plane_measure_file, correction_file

    # collecting data
    cali_flag = 1

    print('start plane calibration process')

    if do_cali == 1:
        while True:
            break_flag = 0
            for i in range(len(cali_tag_no)):
                if len(cali_tag[i]) < 150:
                    break_flag = 1
                    print('collecting...')
                    print(cali_tag_no[i],',',len(cali_tag[i]))

            # small amount of data, remain collecting
            if break_flag == 1:
                cali_flag = 0

            # enough amount of data, stop collecting data
            else:
                cali_flag = 1
                # can check variance ?
                break

            if end_flag == 1:
                return 0

        # save data for cali
        f = open(plane_measure_file,'a')
        for i in range(len(cali_tag_no)):
            for j in range(len(cali_tag[i])):
                str_ = str(cali_tag[i][j].x) + "," + \
                       str(cali_tag[i][j].y) + "," + \
                       str(cali_tag[i][j].z) + "\n"
                f.write(str_)
        f.close()

        print('start calculating ')

        Measure_Data = []
        # calc mean of each data
        for i in range(len(cali_tag_no)):
            meanx = 0
            meany = 0
            meanz = 0
            l = len(cali_tag[i])
            for j in range(l):
                meanx += cali_tag[i][j].x
                meany += cali_tag[i][j].y
                meanz += cali_tag[i][j].z
            meanx = meanx/l
            meany = meany/l
            meanz = meanz/l
            Measure_Data.append(meanx)
            Measure_Data.append(meany)
            Measure_Data.append(meanz)

        # get service request
        req = correction_serviceRequest()
        req.Measure = Measure_Data
        req.Size = 4
        Actual = []
        global coordinate_file
        f = open(coordinate_file)
        for i in f:
            if i[-1] == '\n':
                i = i[:-1]
            i = i.split(',')
            for j in cali_tag_no:
                #print(i)
                #print(j)
                if str(j) == i[0]:
                    Actual.append(float(i[1]))
                    Actual.append(float(i[2]))
                    Actual.append(float(i[3]))
        req.Actual = Actual
        f.close()
        print('get actual data from file done, start call service')

        # do calibration service to get calibration matrix
        rospy.wait_for_service('correction_service')
        try:
            correction_service_ = rospy.ServiceProxy('correction_service', correction_service)
            resp = correction_service_(req)
            resp = resp.Matrix
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            resp = np.zeros([12,1])

        # resize to 3x4 matrix
        Matrix = np.zeros([3, 4])
        for i in range(3):
            for j in range(4):
                Matrix[i][j] = resp[i*4+j]

        # save to file
        f = open(correction_matrix_file,"w")
        for i in range(3):
            str_ = ""
            for j in range(4):
                str_ = str_ + str(Matrix[i][j]) + ","
            str_ = str_[:-1]
            str_ = str_ + "\n"
            f.write(str_)
        f.close()

        # check for error of matrix
        Error_list = []
        temp_data_1 = []
        temp_data_2 = []
        for i in range(len(cali_tag_no)):
            x = np.zeros([4,1])
            x[0][0] = Measure_Data[i*3]
            x[1][0] = Measure_Data[i*3+1]
            x[2][0] = Measure_Data[i*3+2]
            x[3][0] = 1
            temp_data_1.append([x[0][0],x[1][0],x[2][0]])
            y = Matrix.dot(x)
            temp_data_2.append([y[0][0],y[1][0],y[2][0]])
            dx = y[0][0] - Actual[i*3]
            dy = y[1][0] - Actual[i*3+1]
            dz = y[2][0] - Actual[i*3+2]
            sqrtxy  = math.sqrt(dx*dx+dy*dy)
            sqrtxyz = math.sqrt(dx*dx+dy*dy+dz*dz)
            Error_list.append([sqrtxy,sqrtxyz])
        # print to file
        f = open(plane_file,"w")
        for i in temp_data_1:
            str_ = str(i[0]) + "," + str(i[1]) + "," + str(i[2]) +"\n"
        for i in temp_data_2:
            str_ = str(i[0]) + "," + str(i[1]) + "," + str(i[2]) +"\n"
        f.close()

        # display error
        avg_error_xy = 0
        avg_error_xyz = 0
        print('error of each point after calibration:')
        for i in range(len(Error_list)):
            no = cali_tag_no[i]
            avg_error_xy += Error_list[i][0]
            avg_error_xyz += Error_list[i][1]
            print('err number, xy err, xyz err: ',no,' , ',Error_list[i][0],' , ',Error_list[i][1])
        avg_error_xy = avg_error_xy/len(Error_list)
        avg_error_xyz = avg_error_xyz/len(Error_list)
        print('average error after calibration:')
        print('err xy: ', avg_error_xy)
        print('err xy: ', avg_error_xyz)
        #print(M)
        cali_flag = 2
        print('done correction calculation')

    elif do_cali == 0:
        Matrix = np.zeros([3, 4])
        # direct reading file data/ corrected matrix
        f = open(correction_matrix_file,"r")
        count_i = 0;
        for i in f:
            s = i.split(',')
            if s[0] != '\n':
                for j in range(4):
                    s[j] = float(s[j])
                    Matrix[count_i][j] = s[j]
            count_i = count_i + 1;

        f.close()

        cali_flag = 2
        print('done correction calculation')

    end_flag = 1
    print('end measure data')



if __name__ == '__main__':

    rospy.init_node('SendDataToSTM', anonymous=True)

    global coordinate_file
    global do_cali
    global cali_tag
    global cali_tag_no
    global cali_flag
    global robot_connection
    robot_connection = 0

    # read data from file
    coordinate_file = "/home/icmems/WALLE_project/catkin_ws/src/localization/scripts/coordinate.txt"
    f = open(coordinate_file,"r")
    do_cali = 1
    cali_flag = 0
    cali_tag_no = []
    cali_tag = []

    for i in f:
        s = i.split(',')
        if s[0] != '\n':
            s[0] = int(s[0])
            cali_tag_no.append(s[0])
            cali_tag.append([])
    f.close()

    # run main code
    global data_receive_flag
    data_receive_flag = 0
    global end_flag
    end_flag = 0

    # start collect data
    rospy.Subscriber("Camera_Data", Camera_Data, callback)

    # file to save path data
    global plane_file, plane_measure_file, correction_file, correction_matrix_file
    plane_file = "/home/icmems/WALLE_project/catkin_ws/src/localization/plane_.txt"
    plane_measure_file = "/home/icmems/WALLE_project/catkin_ws/src/localization/plane_measure.txt"
    correction_file = "/home/icmems/WALLE_project/catkin_ws/src/localization/robot_corr_0904_1.txt"
    correction_matrix_file = "/home/icmems/WALLE_project/catkin_ws/src/localization/scripts/Correction_Matrix.txt"
    correction_file_ = open(correction_file,"w")
    plane_file_ = open(plane_file,"w")
    plane_measure_file_ = open(plane_measure_file,"w")
    correction_file_.close()
    plane_file_.close()
    plane_measure_file_.close()

    # run and wait for calibration first for localization
    plane_calibration()

    # flag for closing main code
    end_flag = 1
    # rospy.spin()

