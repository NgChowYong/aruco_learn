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


# TODO: need to calculate distance and angle in 2D
def Robot_Data_Process(data, Camera_Pose, ret):
    # calculate length and angle of robot
    # x,y, theta
    x = 0
    y = 0
    z = 0
    theta = 0
    l = len(data.poses)
    if l > 1:
        # TODO : Reserved for many same tag detected
        pass

    elif l == 1:
        x = data.poses[0].position.x
        y = data.poses[0].position.y
        z = data.poses[0].position.z

        # angle already rotate
        theta = math.atan2(data.poses[0].orientation.y, data.poses[0].orientation.x)

    else:
        return [ret, None]

    theta = math.atan2(data.poses[0].orientation.y, data.poses[0].orientation.x)
    camx = Camera_Pose.position.x
    camy = Camera_Pose.position.y
    camz = Camera_Pose.position.z
    distance = math.sqrt(x*x + y*y + z*z) + math.sqrt(camx*camx + camy*camy + camz*camz) 
    pose_update = [x, y, theta]
    # return [(ret + "R," + str(distance) + "," + str(theta) + ","), pose_update]
    #return [(ret + "R," + str(round(x*1000)) + "," + str(round(y*1000)) + "," + str(round(theta*1000)) + "," + str(round(distance*1000)) + ","), pose_update]
    return [(ret + "R," + str(int(x*1000)) + "," + str(int(y*1000)) + "," + str(int(theta*1000)) + "," + str(int(distance*1000)) + ","), pose_update]


def Cam_Data_Process(data, ret):
    # get camera position in x y z
    # data.position.x # poses
    # distance = math.sqrt(data.position.x*data.position.x + data.position.y*data.position.y + data.position.z*data.position.z )
    # theta = math.atan2(data.poses[0].orientation.y, data.poses[0].orientation.x)

    #return ret + "C," + str(round(data.position.x*1000)) + "," + str(round(data.position.y*1000)) + ","
    return ret + "C," + str(int(data.position.x*1000)) + "," + str(int(data.position.y*1000)) + ","


def Path_Data_Process(data):
    return ret + "P,2,3,3,4,1234"

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
    elif cali_flag == 1:
        # do Matrix calculation n stop collecting data
        return

    elif cali_flag == 2:
        # TODO: do correction to all data
        data = Data_Correction(data)

    # initial word sending
    ret = "PC,"

    # robot
    ret, pose_update = Robot_Data_Process(data.Robot_Pose, data.Camera_Pose, ret)
    #main_code.update_pose(pose_update)

    # camera pose
    ret = Cam_Data_Process(data.Camera_Pose, ret)

    # obstacle and path
    #main_code.update_map(data)
    #ret = main_code.update_path(ret,robot_connection,data_receive_flag)

    #data_receive = 'PC,R,1,-2,C,11,23,P,2,3,4,5,6,7,1234,E'
    ret = 'PC,R,0,0,P,2,3,4,5,6,7,1234,'
    data_receive = ret + "E"
    data_receive_flag = 1
    # print(data_receive)
    
    global end_flag
    end_flag = 1
    # send once then close
    

# Send data to STM
# haven't done update data from STM part
def wifi_communication():
    global data_receive, data_receive_flag
    global end_flag, main_code
    global path_file, sense_file
    global robot_connection
    global ros_pub
    print('start wifi')
    count = 0

    try:
        # for python 2.7 used !!
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)# stream using TCP, afinet is using ipv4
        s.bind((HOST, PORT))
        # creating listening port
        print('listen to port')
        r = s.listen(5)
        print('r: ',r)
        if r == None:
            while r != None:
                r = s.listen(5)
                if end_flag == 1:
                    break
        # accept from client request/connect
        conn, addr = s.accept()

        print('Connected by', addr)
        while True:
            # get data from client
            print('collect wifi data')
            data = conn.recv(1024)
            data = data.decode("utf-8")
            if data.find('DK2') == 0:
                print('receive from DK2')
                print(data)
                # ros pub
                f = open(path_file,"a")
                f.write(data)
                f.write("\n")
                f.close()
                # print('sending data : PC,1,2,3')
                # if count >= 1:
                #     data = "END"
                #     data = data.encode("utf-8")
                #     # send data to client
                #     conn.sendall(data)
                # else:

                # TODO: future used for location update from robot
                # main_code.update_pose(pose_update)

            if data_receive_flag == 1:
                data_receive_flag = 0
                # data_receive = 'PC,R,1,-2,C,11,23,P,2,3,4,5,6,7,8,9,12,13,E'

                f = open(sense_file,"a")
                f.write(data_receive)
                f.write("\n")
                f.close()
                robot_connection = 1
                data_receive_ = data_receive.encode("utf-8")
                # send data to client
                conn.sendall(data_receive_)
                print('send: ',data_receive_)
                # count = count + 1
            else:
                # send data to client
                bypass_data = 'PC,E'
                bypass_data = bypass_data.encode("utf-8")
                conn.sendall(bypass_data)

            # client wil send close message then close
            data = 10
            if not data or rospy.is_shutdown() or end_flag == 1:
                break

    finally:
        end_flag = 1
        data = "END"
        data = data.encode("utf-8")
        # send data to client
        try:
            conn.sendall(data)
        except:
            pass

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

    print('start plane calibration process')

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
        for i in range(len(cali_tag_no)):
            x = np.zeros([4,1])
            x[0][0] = Measure_Data[i*3]
            x[1][0] = Measure_Data[i*3+1]
            x[2][0] = Measure_Data[i*3+2]
            x[3][0] = 1
            y = Matrix.dot(x)
            dx = y[0][0] - Actual[i*3]
            dy = y[1][0] - Actual[i*3+1]
            dz = y[2][0] - Actual[i*3+2]
            sqrtxy  = sqrt(dx*dx+dy*dy)
            sqrtxyz = sqrt(dx*dx+dy*dy+dz*dz)
            Error_list.append([sqrtxy,sqrtxyz])
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
    f.close()
    
    # run main code
    global data_receive_flag
    data_receive_flag = 0
    global end_flag
    end_flag = 0

    # start collect data
    rospy.Subscriber("Camera_Data", Camera_Data, callback)
    global ros_pub
    ros_pub = rospy.Publisher('Visual_Data', Marker, queue_size=1)
    # ros_pub.publish(hello_str)
    pub_data = Marker()
    pub_data.id = 0
    pub_data.type = 0
    pub_data.action = 0
    pub_data.pose.position.x = 1
    pub_data.pose.position.y = 1
    pub_data.pose.position.z = 1
    pub_data.pose.orientation.w = 1
    pub_data.color.r = 1
    pub_data.lifetime = 0
    pub_data.header.frame_id = '/map'
    pub_data.header.stamp = rospy.get_rostime()
    ros_pub.publish(pub_data)

    # file to save path data
    global path_file, sense_file, correction_file, correction_matrix_file
    path_file = "/home/icmems/WALLE_project/catkin_ws/src/localization/robot_path.txt"
    sense_file = "/home/icmems/WALLE_project/catkin_ws/src/localization/robot_sense.txt"
    correction_file = "/home/icmems/WALLE_project/catkin_ws/src/localization/robot_corr_0904_1.txt"
    correction_matrix_file = "/home/icmems/WALLE_project/catkin_ws/src/localization/scripts/Correction_Matrix.txt"
    correction_file_ = open(correction_file,"w")
    path_file_ = open(path_file,"w")
    sense_file_ = open(sense_file,"w")
    correction_file_.close()
    path_file_.close()
    sense_file_.close()

    # run and wait for calibration first for localization
    plane_calibration()

    # run continuous communication with DK2
    wifi_communication()

    # flag for closing main code
    end_flag = 1
    # rospy.spin()

