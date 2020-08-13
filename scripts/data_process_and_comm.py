#!/usr/bin/env python2

# general python package
import rospy
import socket
import threading
import time
import math

# ros package
from localization.msg import Camera_Data
import tf
# from tf.transformations import euler_from_quaternion, quaternion_from_euler

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
            if time.time() - start_time > 100:
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
            return ret

    def Astar(self):
        pass


# TODO: need to calculate distance and angle in 2D
def Robot_Data_Process(data, ret):
    # calculate length and angle of robot
    # x,y, theta
    x = 0
    y = 0
    z = 0
    theta = 0
    l = len(data.poses)
    if l > 1:
        # TODO : modify this code here to use multiple tag
        for i in range(l):
            # do find center
            # do estimation
            x += data.poses[i].position.x
            y += data.poses[i].position.y
            # do rotation getting out
        x = x / len(data)
        y = y / len(data)
        # x = x/len(data)

    elif l == 1:
        x = data.poses[0].position.x
        y = data.poses[0].position.y
        z = data.poses[0].position.z

        # angle already rotate
        theta = math.atan2(data.poses[0].orientation.y, data.poses[0].orientation.x)

    else:
        return [ret, None]

    theta = math.atan2(data.poses[0].orientation.y, data.poses[0].orientation.x)
    distance = math.sqrt(x*x + y*y + z*z)
    pose_update = [x, y, theta]
    # return [(ret + "R," + str(distance) + "," + str(theta) + ","), pose_update]
    return [(ret + "R," + str(x) + "," + str(y) + "," + str(theta) + "," + str(distance) + ","), pose_update]


def Cam_Data_Process(data, ret):
    # get camera position in x y z
    # data.position.x # poses
    # distance = math.sqrt(data.position.x*data.position.x + data.position.y*data.position.y + data.position.z*data.position.z )
    # theta = math.atan2(data.poses[0].orientation.y, data.poses[0].orientation.x)

    return ret + "C," + str(data.position.x) + "," + str(data.position.y) + ","


def Path_Data_Process(data):
    return ret + "P,2,3,3,4,"


# callback function for rospy to used
def callback(data):
    global data_receive, data_receive_flag
    global main_code
    # data process
    # data update and send
    # initial word sending
    ret = "PC,"

    # robot
    ret, pose_update = Robot_Data_Process(data.Robot_Pose, ret)
    main_code.update_pose(pose_update)

    # camera pose
    ret = Cam_Data_Process(data.Camera_Pose, ret)

    # obstacle and path
    main_code.update_map(data)
    ret = main_code.update_path(ret)

    # data_receive = 'PC,R,1,-2,C,11,23,P,2,3,4,5,6,7,E'
    data_receive = ret + "E"
    data_receive_flag = 1
    # print(data_receive)

# Send data to STM
# haven't done update data from STM part
def wifi_communication():
    global data_receive, data_receive_flag
    global end_flag, main_code
    print('start wifi')
    count = 0

    try:
        # for python 2.7 used !!
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)# stream using TCP, afinet is using ipv4
        s.bind((HOST, PORT))
        # creating listening port
        print('listen to port')
        s.listen(5)
        # accept from client request/connect
        conn, addr = s.accept()
        print('Connected by', addr)
        while True:
            # get data from client
            data = conn.recv(1024)
            data = data.decode("utf-8")
            print(data)
            if data.find('DK2') == 0:
                print('receive from DK2')
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
                data_receive_ = data_receive.encode("utf-8")

                # send data to client
                conn.sendall(data_receive_)
                print('send: ',data_receive_)
                # count = count + 1

            # client wil send close message then close
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

if __name__ == '__main__':
    rospy.init_node('SendDataToSTM', anonymous=True)
    rospy.Subscriber("Camera_Data", Camera_Data, callback)

    # run main code
    global end_flag, main_code
    end_flag = 0
    main_code = MainThread(1)
    main_code.start()
    # run continuous communication with DK2
    wifi_communication()

    # flag for closing main code
    # TODO: end flag trigger for 3 threads
    end_flag = 1
    # rospy.spin()
