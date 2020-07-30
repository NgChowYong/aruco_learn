#!/usr/bin/env python3

# turn into ros version

import rospy
import socket
from localization_node.msg import Camera_Data

HOST = '192.168.43.77'  # Standard loopback interface address (localhost)
PORT = 11223        # Port to listen on (non-privileged ports are > 1023)

class MainThread(threading.Thread):
    def __init__(self, num):
        threading.Thread.__init__(self)
        self.num = num
        self.new_path = 0

    def run(self):
        global end_flag
        print("thread start")
        # initialize map and all parameter....
        # can read a yaml file ?

        # wait for start cmd
        # after start cmd

        start_time = time.time()
        while (1):

            # do smth
            time.sleep(self.num)
            if self.new_path == 1:
                self.new_path = 0
                # do a star and check for new path
                # if new path then set flag for update_path

            # trigger end of code
            if (time.time() - start_time > 100):
                # trigger end of code
                # break
                pass

            # end of code
            if(end_flag == 1):
                break
        print("thread end")

    def update_data(self, data, ret):
        print("test")
        # do map update
        self.update_map(data.Obstacle_Pose)
        # do cehcking for path update
        self.new_path = 1

    def update_map(data):
        # update map
        pass

    def update_path(self, ret):
        # return ret + "P,1,2,3,4,5,6"
        return ret


def Robot_Data_Process(data, ret):
    # calculate length and angle of robot
    return ret + "R,2,3,"

def Cam_Data_Process(data):
    return ret + "C,2,3,"

def Path_Data_Process(data):
    return ret + "P,2,3,3,4"

def callback(data):
    global data_receive, data_receive_flag
    global main_code
    # data process
    # data update and send
    # initial word sending
    ret = "PC,"

    # robot
    ret   = Robot_Data_Process(data.Robot_Pose,ret)

    # camera pose
    ret   = Cam_Data_Process(data.Camera_Pose ,ret)

    # obstacle and path
    # ret    = Path_Data_Process(data.Obstacle_Pose,ret)
    main_code.update_data(data)
    ret   = main_code.update_path(ret)

    # data_receive = 'PC,R,1,-2,C,11,23,P,2,3,4,5,6,7,E'
    data_receive = ret + "E"
    data_receive_flag = 1

if __name__ == '__main__':
    global data_receive, data_receive_flag
    rospy.init_node('SendDataToSTM', anonymous=True)
    rospy.Subscriber("Camera_Data", Camera_Data, callback)

    global end_flag, main_code
    end_flag = 0
    main_code = MainThread(1)
    main_code.run()

    count = 0
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s: # stream using TCP, afinet is using ipv4
        s.bind((HOST, PORT))
        # creating listening port
        s.listen()
        # accept from client request/connect
        conn, addr = s.accept()
        with conn:
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

                    # need to update data from stm ~

                if data_receive_flag == 1
                    data_receive_flag = 0
                    # data_receive = 'PC,R,1,-2,C,11,23,P,2,3,4,5,6,7,8,9,12,13,E'
                    data_receive = data_receive.encode("utf-8")
                    # send data to client
                    conn.sendall(data_receive)

                    # count = count + 1

                # client wil send close message then close
                if not data:
                    break
    end_flag = 1
    # rospy.spin()
