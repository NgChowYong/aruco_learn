#!/usr/bin/env python2

# general python package
import rospy
import time
import math
import numpy as np

# ros package
from localization.msg import Camera_Data
import tf
from localization.srv import *


def Data_Correction(data):
    global Matrix
    global correction_file
    Data_Measure = np.zeros([4,1])
    correction_file_ = open(correction_file,"a")
    # obstacle position correction
    for i in range(len(data.Obstacle_Pose.poses)):
        correction_file_.write("b,"+str(data.Obstacle_ID[i])+",")
        Data_Measure[0][0] = data.Obstacle_Pose.poses[i].position.x
        Data_Measure[1][0] = data.Obstacle_Pose.poses[i].position.y
        Data_Measure[2][0] = data.Obstacle_Pose.poses[i].position.z
        correction_file_.write(str(Data_Measure[0][0])+",")
        correction_file_.write(str(Data_Measure[1][0])+",")
        correction_file_.write(str(Data_Measure[2][0])+"\n")
        Data_Measure[3][0] = 1
        result = Matrix.dot(Data_Measure)
        data.Obstacle_Pose.poses[i].position.x = result[0][0]
        data.Obstacle_Pose.poses[i].position.y = result[1][0]
        data.Obstacle_Pose.poses[i].position.z = result[2][0]
        correction_file_.write("a,"+str(data.Obstacle_ID[i])+",")
        correction_file_.write(str(result[0][0])+",")
        correction_file_.write(str(result[1][0])+",")
        correction_file_.write(str(result[2][0])+"\n")

    # robot position correction
    if len(data.Robot_Pose.poses) != 0:
        correction_file_.write("b,"+str(data.Robot_ID)+",")
        Data_Measure[0][0] = data.Robot_Pose.poses[0].position.x
        Data_Measure[1][0] = data.Robot_Pose.poses[0].position.y
        Data_Measure[2][0] = data.Robot_Pose.poses[0].position.z
        correction_file_.write(str(Data_Measure[0][0])+",")
        correction_file_.write(str(Data_Measure[1][0])+",")
        correction_file_.write(str(Data_Measure[2][0])+"\n")
        Data_Measure[3][0] = 1
        result = Matrix.dot(Data_Measure)
        data.Robot_Pose.poses[0].position.x = result[0][0]
        data.Robot_Pose.poses[0].position.y = result[1][0]
        data.Robot_Pose.poses[0].position.z = result[2][0]
        correction_file_.write("a,"+str(data.Robot_ID)+",")
        correction_file_.write(str(result[0][0])+",")
        correction_file_.write(str(result[1][0])+",")
        correction_file_.write(str(result[2][0])+"\n")

    correction_file_.close()

    return data

# callback function for rospy to used
def callback(data):

    global cali_tag
    global cali_tag_no
    global cali_flag
    global do_cali
    global pub

    if cali_flag == 0:
        if do_cali == 1:
            # do data collection
            for i in range(len(data.Obstacle_Pose.poses)):
                for j in range(len(cali_tag_no)):
                    if data.Obstacle_ID[i] == cali_tag_no[j]:
                        cali_tag[j].append(data.Obstacle_Pose.poses[i].position)
    elif cali_flag == 1:
        # do Matrix calculation n stop collecting data
        pass

    elif cali_flag == 2:
        # TODO: do correction to all data
        data = Data_Correction(data)
        pub.publish(data)


def plane_calibration():
    global do_cali
    global cali_tag
    global cali_tag_no
    global cali_flag
    global Matrix
    global end_flag
    global correction_matrix_file
    # collecting data
    cali_flag = 1

    if do_cali == 1:
        while True:
            break_flag = 0
            for i in range(len(cali_tag_no)):
                if len(cali_tag[i]) < 100:
                    break_flag = 1
                    print('collecting...')
                    print(cali_tag_no[i],',',len(cali_tag[i]))

            # small amount of data, remain collecting
            if break_flag == 1:
                cali_flag = 0

            # enough amount of data, stop collecting data
            elif  break_flag == 0:
                cali_flag = 1
                # can check variance ?
                break

            if end_flag == 1:
                return 0

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

        Matrix = np.zeros([3, 4])
        for i in range(3):
            for j in range(4):
                Matrix[i][j] = resp[i*4+j]

        f = open(correction_matrix_file,"w")
        for i in range(3):
            str_ = ""
            for j in range(4):
                str_ = str_ + str(Matrix[i][j]) + ","

            str_ = str_[:-1]
            str_ = str_ + "\n"
            f.write(str_)
        f.close()

        #print(M)
        cali_flag = 2
        print('done correction calculation')
    elif do_cali == 0:
        Matrix = np.zeros([3, 4])
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

if __name__ == '__main__':
    rospy.init_node('DataProcess', anonymous=True)

    global coordinate_file
    coordinate_file = "/home/icmems/WALLE_project/catkin_ws/src/localization/scripts/coordinate.txt"

    global do_cali
    global cali_tag
    global cali_tag_no
    global cali_flag
    global robot_connection
    robot_connection = 0
    f = open(coordinate_file,"r")
    do_cali = 0
    cali_flag = 0
    cali_tag_no = []
    cali_tag = []

    for i in f:
        s = i.split(',')
        if s[0] != '\n':
            s[0] = int(s[0])
            cali_tag_no.append(s[0])
            cali_tag.append([])

    # run main code
    global end_flag, main_code
    global data_receive_flag
    data_receive_flag = 0
    end_flag = 0
    main_code = 0
    main_code = MainThread(0.1)
    main_code.start()

    # start collect data
    rospy.Subscriber("Camera_Data", Camera_Data, callback)
    global pub
    pub = rospy.Publisher('Camera_Data_Calibrated', Camera_Data, queue_size=2)

    # file to save path data
    global path_file, sense_file, correction_file, correction_matrix_file
    path_file = "/home/icmems/WALLE_project/catkin_ws/src/localization/robot_path.txt"
    sense_file = "/home/icmems/WALLE_project/catkin_ws/src/localization/robot_sense.txt"
    correction_file = "/home/icmems/WALLE_project/catkin_ws/src/localization/robot_corr_0904_1.txt"
    correction_matrix_file = "/home/icmems/WALLE_project/catkin_ws/src/localization/scripts/Correction_Matrix.txt"
    correction_file_ = open(correction_file,"w")
    correction_file_.close()
    path_file_ = open(path_file,"w")
    path_file_.close()
    sense_file_ = open(sense_file,"w")
    sense_file_.close()

    # run and wait for calibration first for localization
    plane_calibration()

    while not rospy.is_shutdown():
        pass
    
    # flag for closing main code
    end_flag = 1
    # rospy.spin()

