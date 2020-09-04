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
from localization.srv import *

# for reading map only # not sure can be used or not
import cv2

# HOST = '192.168.43.77'  # Standard loopback interface address (localhost)
HOST = '192.168.0.199'  # Standard loopback interface address (localhost)
PORT = 11223  # Port to listen on (non-privileged ports are > 1023)


class MainThread(threading.Thread):
    def __init__(self, num):
        threading.Thread.__init__(self)
        self.num = num
        self.new_path_flag = 0
        self.update_path_flag = 0

        img = cv2.imread('/home/icmems/WALLE_project/catkin_ws/src/localization/simple_version_small_small.jpg', cv2.IMREAD_GRAYSCALE)
        self.width = img.shape[1]
        self.height = img.shape[0]
        for i in range(self.width):
            for j in range(self.height):
                if img[j][i] == 0:
                    img[j][i] = -1
                else:
                    img[j][i] = 0
                    # print('strange' ,j , i)
        self.map = img  # 0 == wall 255 = free

        self.robot_pose = [100, 100, 0]  # x y theta
        self.task1 = [200, 100, 0]  # x y theta
        self.task2 = [100, 200, 0]  # x y theta

        self.old_path = []
        self.new_path = []

    def run(self):
        global end_flag
        print("thread start")
        # initialize map and all parameter....
        # can read a yaml file ?

        # wait for start cmd
        # after start cmd

        start_time = time.time()
        while 1:

            # do smth
            time.sleep(self.num)

            # update of path ?
            if self.new_path_flag == 1:
                self.new_path_flag = 0
                # do a star and check for new path
                self.Astar()
                # if new path then set flag for update_path
                if self.old_path != self.new_path:
                    self.update_path_flag = 1

            # trigger end of code
            if time.time() - start_time > 1800:
                # trigger end of code
                break
                pass

            # end of code
            if (end_flag == 1) or rospy.is_shutdown():
                end_flag = 1
                break
        print("thread end")

    def update_pose(self, pose_update):
        #print("test")
        # update pose of robot, camera pose is not needed in calculation
        if pose_update is not None:
            self.robot_pose = pose_update

    def update_map(self, data):
        # update map
        for i in range(len(data.Obstacle_Pose.poses)):
            self.map[int(data.Obstacle_Pose.poses[i].position.x)][int(data.Obstacle_Pose.poses[i].position.y)] = 1
        cv2.imshow('image', self.map)

        # do cehcking for path update
        self.new_path_flag = 1

    def update_path(self, ret):
        if self.update_path_flag == 0:
            self.update_path_flag = 1
            temp = "P,"
            for i in range(int(len(self.new_path) / 2)):
                temp = temp + self.new_path[i * 2], "," + self.new_path[i * 2 + 1] + ","
            return ret + temp
        else:
            # round( ? *1000)
            return ret

    def Astar(self):
        pass


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
        # TODO : modify this code here to use multiple tag
##        for i in range(l):
##            # do find center
##            # do estimation
##            x += data.poses[i].position.x
##            y += data.poses[i].position.y
##            # do rotation getting out
##        x = x / len(data)
##        y = y / len(data)
##        # x = x/len(data)
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
    global main_code
    # data process

    global cali_tag
    global cali_tag_no
    global cali_flag

    if cali_flag == 0:
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

    # initial word sending
    ret = "PC,"

    # robot
    ret, pose_update = Robot_Data_Process(data.Robot_Pose, data.Camera_Pose, ret)
    main_code.update_pose(pose_update)

    # camera pose
    ret = Cam_Data_Process(data.Camera_Pose, ret)

    # obstacle and path
    main_code.update_map(data)
    ret = main_code.update_path(ret)

    # data_receive = 'PC,R,1,-2,C,11,23,P,2,3,4,5,6,7,1234,E'
    data_receive = ret + "E"
    data_receive_flag = 1
    # print(data_receive)

# Send data to STM
# haven't done update data from STM part
def wifi_communication():
    global data_receive, data_receive_flag
    global end_flag, main_code
    global path_file, sense_file
    print('start wifi')
    count = 0

    try:
##        # for python 2.7 used !!
##        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)# stream using TCP, afinet is using ipv4
##        s.bind((HOST, PORT))
##        # creating listening port
##        print('listen to port')
##        r = s.listen(5)
##        print('r: ',r)
##        if r == None:
##            while r != None:
##                r = s.listen(5)
##                if end_flag == 1:
##                    break
##        # accept from client request/connect
##        conn, addr = s.accept()
##
##        print('Connected by', addr)
        while True:
            pass
##            # get data from client
##            print('collect wifi data')
##            data = conn.recv(1024)
##            data = data.decode("utf-8")
##            if data.find('DK2') == 0:
##                print('receive from DK2')
##                print(data)
##                path_file.write(data)
##                path_file.write("\n")
##
##                # print('sending data : PC,1,2,3')
##                # if count >= 1:
##                #     data = "END"
##                #     data = data.encode("utf-8")
##                #     # send data to client
##                #     conn.sendall(data)
##                # else:
##
##                # TODO: future used for location update from robot
##                # main_code.update_pose(pose_update)
##
##            if data_receive_flag == 1:
##                data_receive_flag = 0
##                # data_receive = 'PC,R,1,-2,C,11,23,P,2,3,4,5,6,7,8,9,12,13,E'
##                sense_file.write(data_receive)
##                data_receive_ = data_receive.encode("utf-8")
##
##                # send data to client
##                conn.sendall(data_receive_)
##                print('send: ',data_receive_)
##                # count = count + 1
##            else:
##                # send data to client
##                bypass_data = 'PC,E'
##                bypass_data = bypass_data.encode("utf-8")
##                conn.sendall(bypass_data)

            # client wil send close message then close
            data = 10
            if not data or rospy.is_shutdown() or end_flag == 1:
                break
            
        # for python 3 used !!
        # with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:  # stream using TCP, afinet is using ipv4
        #     s.bind((HOST, PORT))
        #     # creating listening port
        #     print('listen to port')
        #     s.listen()
        #     # accept from client request/connect
        #     conn, addr = s.accept()
        #     with conn:
        #         print('Connected by', addr)
        #         while True:
        #             # get data from client
        #             data = conn.recv(1024)
        #             data = data.decode("utf-8")
        #             print(data)
        #             if data.find('DK2') == 0:
        #                 print('receive from DK2')
        #                 # print('sending data : PC,1,2,3')
        #                 # if count >= 1:
        #                 #     data = "END"
        #                 #     data = data.encode("utf-8")
        #                 #     # send data to client
        #                 #     conn.sendall(data)
        #                 # else:
        #
        #                 # TODO: future used for location update from robot
        #                 # main_code.update_pose(pose_update)
        #
        #             if data_receive_flag == 1:
        #                 data_receive_flag = 0
        #                 # data_receive = 'PC,R,1,-2,C,11,23,P,2,3,4,5,6,7,8,9,12,13,E'
        #                 data_receive_ = data_receive.encode("utf-8")
        #
        #                 # send data to client
        #                 conn.sendall(data_receive_)
        #                 print('send: ',data_receive_)
        #                 # count = count + 1
        #
        #             # client wil send close message then close
        #             if not data or rospy.is_shutdown() or end_flag == 1:
        #                 break
    finally:
        end_flag = 1
        path_file.close()
        sense_file.close()
        data = "END"
        data = data.encode("utf-8")
        # send data to client
        # conn.sendall(data)

def plane_calibration():
    global cali_tag
    global cali_tag_no
    global cali_flag
    global Matrix
    global end_flag
    # collecting data
    cali_flag = 0

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
    f = open("/home/icmems/WALLE_project/catkin_ws/src/localization/scripts/coordinate.txt")
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
    #print(M)
    cali_flag = 2
    print('done correction calculation')

if __name__ == '__main__':
    rospy.init_node('SendDataToSTM', anonymous=True)
    rospy.Subscriber("Camera_Data", Camera_Data, callback)

    global cali_tag
    global cali_tag_no
    global cali_flag
    cali_flag = 0
    cali_tag_no = []
    cali_tag_no.append(13)
    cali_tag_no.append(14)
    cali_tag_no.append(15)
    cali_tag_no.append(16)
    cali_tag_no.append(17)
    cali_tag_no.append(18)
    cali_tag_no.append(19)
    cali_tag_no.append(20)
    cali_tag = []
    cali_tag.append([])
    cali_tag.append([])
    cali_tag.append([])
    cali_tag.append([])
    cali_tag.append([])
    cali_tag.append([])
    cali_tag.append([])
    cali_tag.append([])


    # run main code
    global end_flag, main_code
    end_flag = 0
    main_code = 0
    main_code = MainThread(1)
    main_code.start()

    # file to save path data
    global path_file, sense_file,correction_file
    path_file = open("/home/icmems/WALLE_project/catkin_ws/src/localization/robot_path.txt","w")
    sense_file = open("/home/icmems/WALLE_project/catkin_ws/src/localization/robot_sense.txt","w")
    correction_file = "/home/icmems/WALLE_project/catkin_ws/src/localization/robot_corr_0904_1.txt"
    correction_file_ = open(correction_file,"w")
    correction_file_.close()

    # run and wait for calibration first for localization
    plane_calibration()

    # run continuous communication with DK2
    wifi_communication()

    # flag for closing main code
    end_flag = 1
    # rospy.spin()

