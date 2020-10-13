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
from localization.msg import Status_Data
from geometry_msgs.msg import PoseArray
import tf

# for reading map only # not sure can be used or not
import cv2

# HOST = '192.168.43.77'  # Standard loopback interface address (localhost)
# HOST = '192.168.0.199'  # Standard loopback interface address (localhost)
HOST = '10.42.0.1'  # Standard loopback interface address (localhost)
PORT = 11223  # Port to listen on (non-privileged ports are > 1023)

class Comm_Data:
    # data passing to comm
    path = PoseArray()
    path_update_flag = 0
    camdata = Camera_Data()
    camdata_update_flag = 0
    # data output
    data_in_str = "PC,E"

    def camdata_store(self,data):
        self.camdata = data
        self.camdata_update_flag = 1

    def pathdata_store(self,data):
        self.path = data
        self.path_update_flag = 1

    def check_update(self):
        if self.camdata_update_flag == 1 or self.path_update_flag == 1:
            return True
        else:
            return False

    def return_data_string(self):
        return self.data_in_str

    def data_combine(self):
        # initial word sending
        ret = "PC,"

        # update when new data capture
        if self.camdata_update_flag == 1:
            # robot
            ret = self.Robot_Data_Process(self.camdata.Robot_Pose, self.camdata.Camera_Pose, ret)
            # camera pose
            ret = self.Cam_Data_Process(self.camdata.Camera_Pose, ret)
            camdata_update_flag = 0
        if self.path_update_flag == 1:
            # path
            ret = self.Path_Data_Process(self.path.poses, ret)

        # output
        ret = ret + "E"
        self.data_in_str = ret


    # TODO: need to calculate distance and angle in 2D
    def Robot_Data_Process(self,data, Camera_Pose, ret):
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

        elif l >= 1:
            x = data.poses[0].position.x
            y = data.poses[0].position.y
            z = data.poses[0].position.z

            # angle already rotate
            theta = math.atan2(data.poses[0].orientation.y, data.poses[0].orientation.x)

        else:
            return ret

        theta = math.atan2(data.poses[0].orientation.y, data.poses[0].orientation.x)
        camx = Camera_Pose.position.x
        camy = Camera_Pose.position.y
        camz = Camera_Pose.position.z
        distance = math.sqrt(x*x + y*y + z*z) + math.sqrt(camx*camx + camy*camy + camz*camz) 
        return (ret + "R," + str(int(x*1000)) + "," + str(int(y*1000)) + "," + str(int(theta*1000)) + "," + str(int(distance*1000)) + ",")

    def Cam_Data_Process(self,data, ret):
        print(ret)
        return ret + "C," + str(int(data.position.x*1000)) + "," + str(int(data.position.y*1000)) + ","

    def Path_Data_Process(self,data, ret):
        str_ = ""
        for i in range(len(data)-1):
            str_ = str_ + str(data[i].position.x)
            str_ = str_ + ","
            str_ = str_ + str(data[i].position.y)
            str_ = str_ + ","
        i = i + 1
        str_ = str_ + str(data[i].position.x)
        str_ = str_ + ","
        str_ = str_ + str(data[i].position.y)
        str_ = str_ + ","
        str_ = str_ + str(data[i].orientation.w)
        return ret + "P," + str_ + ","

    def receive_DK2(self,data):
        if data == 2 or data == 3:
            self.path_update_flag = 0

# callback function for rospy to used
def callback_camdata(data):
    global comm_data
    comm_data.camdata_store(data)


# callback function for rospy to used
def callback_path(data):
    global comm_data
    comm_data.pathdata_store(data)

# Send data to STM
# haven't done update data from STM part
def wifi_communication():
    global comm_data
    global pub
    global end_flag
    print('start wifi')
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
                str_ = data.split(',')
                if str_[1] != 'E':
                    # save data to comm
                    comm_data.receive_DK2(int(str_[1]))

                    # publish data
                    robot_status = Status_Data()
                    robot_status.Robot_Status = int(float(str_[2]))
                    robot_status.Robot_State.position.x = float(str_[3])
                    robot_status.Robot_State.position.y = float(str_[4])
                    robot_status.Robot_State.orientation.w = float(str_[5])
                    pub.publish(robot_status)

            if comm_data.check_update() == True:
                comm_data.data_combine()
                data_receive = comm_data.return_data_string()
                data_receive_ = data_receive.encode("utf-8")
                # send data to client
                conn.sendall(data_receive_)
                print('send: ',data_receive_)
            else:
                # send data to client
                bypass_data = 'PC,E'
                bypass_data = bypass_data.encode("utf-8")
                conn.sendall(bypass_data)

            # client wil send close message then close
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


if __name__ == '__main__':
    rospy.init_node('SendDataToSTM', anonymous=True)

    # run main code
    global end_flag
    end_flag = 0

    global comm_data
    comm_data = Comm_Data()

    # start collect data
    rospy.Subscriber("Camera_Data_Calibrated", Camera_Data, callback_camdata)
    rospy.Subscriber("Target_Path", PoseArray, callback_path)
    global pub
    pub = rospy.Publisher('Robot_Status', Status_Data, queue_size=1)

    # run continuous communication with DK2
    wifi_communication()

    # flag for closing main code
    end_flag = 1
    # rospy.spin()

