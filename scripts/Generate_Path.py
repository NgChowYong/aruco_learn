#!/usr/bin/env python2

# general python package
import rospy
import time
import math
import numpy as np

# ros package
from localization.msg import Status_Data
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
import tf

# for reading map only # not sure can be used or not
import cv2


##class MainThread(threading.Thread):
##    def __init__(self, num):
##        threading.Thread.__init__(self)
##        self.num = num
##        self.new_path_flag = 0
##        self.update_path_flag = 0
##
##        img = cv2.imread('/home/icmems/WALLE_project/catkin_ws/src/localization/simple_version_small_small.jpg', cv2.IMREAD_GRAYSCALE)
##        self.width = img.shape[1]
##        self.height = img.shape[0]
##        for i in range(self.width):
##            for j in range(self.height):
##                if img[j][i] == 0:
##                    img[j][i] = -1
##                else:
##                    img[j][i] = 0
##                    # print('strange' ,j , i)
##        self.map = img  # 0 == wall 255 = free
##
##        self.robot_pose = [0, -300, 0]  # x y theta
##        self.task1 = [200, 100, 0]  # x y theta
##        self.task2 = [100, 200, 0]  # x y theta
##
##        self.old_path = []
##        self.new_path = [[400,-400,90],[400,400,-180],[-400,400,-90],[-400,-400,0]]
##        self.count = 0
##
##    def run(self):
##        global end_flag
##        print("thread start")
##        # initialize map and all parameter....
##        # can read a yaml file ?
##
##        # wait for start cmd
##        # after start cmd
##
##        start_time = time.time()
##        while 1:
##
##            # do smth
##            time.sleep(self.num)
##
##            # update of path ?
##            if self.new_path_flag == 1:
##                self.new_path_flag = 0
##                # do a star and check for new path
##                #self.Astar()
##                # if new path then set flag for update_path
##                # print("start run path")
##                xx = self.robot_pose[0] - self.new_path[-1][0]
##                yy = self.robot_pose[1] - self.new_path[-1][1]
##                if math.sqrt(xx*xx + yy*yy) < 100:
##                    self.old_path = []
##                    self.count = 0
##                if self.old_path != self.new_path:
##                    self.update_path_flag = 1
##                    print("start update path")
##
##            # trigger end of code
##            if time.time() - start_time > 1800:
##                # trigger end of code
##                break
##                pass
##
##            # end of code
##            if (end_flag == 1) or rospy.is_shutdown():
##                end_flag = 1
##                break
##        print("thread end")

class MainFunction():
    def __init__(self):
        # self.smth = 0
        self.constant_path = [[500,300,0],[500,1000,0],[-500,1000,0],[-500,300,0]] # in mm
        self.robot_status = 0
        self.robot_state = [0,0,0] # initial state
        self.reaching_distance = 30 # in mm
        self.reaching = 0
	self.starting = 1

    def recorded_path(self,repeat_no):
        global pub
        rate = rospy.Rate(1) # 1hz
        rate.sleep()
        count = 0
        print("start generate path")
        while not rospy.is_shutdown() and count < repeat_no:
            if self.check_reaching_last_target(self.constant_path) == True or self.starting == 1:
                self.starting = 0
                pub_data = PoseArray()
                for i in self.constant_path:
                    pose = Pose()
                    pose.position.x = i[0]
                    pose.position.y = i[1]
                    pose.orientation.w = i[2]
                    pub_data.poses.append(pose)
                pub.publish(pub_data)
                count = count + 1
                print("generate no: "+str(count))
                print("end generate route")
                break;
            rate.sleep()
        print("generate path end")

    def check_reaching_last_target(self,path):
        dx = path[-1][0] - self.robot_state[0]
        dy = path[-1][1] - self.robot_state[1]
        if math.sqrt(dx*dx +dy*dy) > self.reaching_distance:
            self.reaching = 0
            #print("large dist")
            return False
        else:
            if self.reaching == 0:
                self.reaching = 1
                #print("short dist")
                return True
            else:
                #print("not leaving yet")
                return False


    def update_robot_status(self,data):
        self.robot_status = data.Robot_Status
        self.robot_state = [data.Robot_State.position.x, \
                            data.Robot_State.position.y, \
                            data.Robot_State.orientation.w]
        #print("update pose")


def callback(data):
    global main_code
    main_code.update_robot_status(data)

if __name__ == '__main__':
    rospy.init_node('PathGeneration', anonymous=True)

    global main_code
    main_code = MainFunction()

    global pub
    pub = rospy.Publisher('Target_Path', PoseArray, queue_size=0)
    rospy.Subscriber("Robot_Status", Status_Data, callback)

    main_code.recorded_path(5)

