#!/usr/bin/env python2

# general python package
import rospy
import time
import math

# ros package
from localization.msg import Camera_Data_error
from sensor_msgs.msg import Imu
import tf
# from tf.transformations import euler_from_quaternion, quaternion_from_euler

file1 = "measure_obs_t9.txt"
file2 = "measure_cam_t9.txt"
file3 = "measure_acc_t9.txt"

f=open(file1,"w")
f.close()
f=open(file2,"w")
f.close()

# callback function for rospy to used
def callback(data):
    f=open(file2,"a")
    str_ = ""
    str_ = str_ + str(data.Camera_Pose.position.x)
    str_ = str_ + ","
    str_ = str_ + str(data.Camera_Pose.position.y)
    str_ = str_ + ","
    str_ = str_ + str(data.Camera_Pose.position.z)
    str_ = str_ + ","
    str_ = str_ + str(data.Camera_Pose.orientation.x)
    str_ = str_ + ","
    str_ = str_ + str(data.Camera_Pose.orientation.y)
    str_ = str_ + ","
    str_ = str_ + str(data.Camera_Pose.orientation.z)
    str_ = str_ + ","
    str_ = str_ + str(data.Camera_Pose.orientation.w)
    str_ = str_ + "\n"
    f.write(str_)
    f.close()
    f=open(file1,"a")
    str_ = ""
    for i in range(len(data.Obstacle_Pose.poses)):
        str_ = str_ + str(data.Obstacle_ID[i])
        str_ = str_ + ","
        str_ = str_ + str(data.Obstacle_Pose.poses[i].position.x)
        str_ = str_ + ","
        str_ = str_ + str(data.Obstacle_Pose.poses[i].position.y)
        str_ = str_ + ","
        str_ = str_ + str(data.Obstacle_Pose.poses[i].position.z)
        str_ = str_ + ","
        str_ = str_ + str(data.Obstacle_Pose.poses[i].orientation.x)
        str_ = str_ + ","
        str_ = str_ + str(data.Obstacle_Pose.poses[i].orientation.y)
        str_ = str_ + ","
        str_ = str_ + str(data.Obstacle_Pose.poses[i].orientation.z)
        str_ = str_ + ","
        str_ = str_ + str(data.Obstacle_Pose.poses[i].orientation.w)
        str_ = str_ + ","
        str_ = str_ + str(data.Obstacle_Pose_ori.poses[i].position.x)
        str_ = str_ + ","
        str_ = str_ + str(data.Obstacle_Pose_ori.poses[i].position.y)
        str_ = str_ + ","
        str_ = str_ + str(data.Obstacle_Pose_ori.poses[i].position.z)
        str_ = str_ + ","
        str_ = str_ + str(data.Obstacle_Pose_ori.poses[i].orientation.x)
        str_ = str_ + ","
        str_ = str_ + str(data.Obstacle_Pose_ori.poses[i].orientation.y)
        str_ = str_ + ","
        str_ = str_ + str(data.Obstacle_Pose_ori.poses[i].orientation.z)
        str_ = str_ + ","
        str_ = str_ + str(data.Obstacle_Pose_ori.poses[i].orientation.w)
        str_ = str_ + "\n"

    print(str_)
    f.write(str_)
    f.close()

def callback2(data):
    f=open(file3,"a")
    str_2 = str(data.linear_acceleration.x) + " " + str(data.linear_acceleration.y) + " " + str(data.linear_acceleration.z) + " \n"
    f.write(str_2)
    f.close()

if __name__ == '__main__':
    f=open(file1,"a")
    f.write("ID,xp,yp,zp,xo,yo,zo,wo,xori,yori,zori,xoori,yoori,zoori,woori\n")
    f.close()
    rospy.init_node('SendDataToSTM', anonymous=True)
    rospy.Subscriber("Camera_Data", Camera_Data_error, callback)
    rospy.Subscriber("camera/accel/sample", Imu, callback2)
    rospy.spin()
