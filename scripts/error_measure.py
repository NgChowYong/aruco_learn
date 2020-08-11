#!/usr/bin/env python2

# general python package
import rospy
import time
import math

# ros package
from localization.msg import Camera_Data
import tf
# from tf.transformations import euler_from_quaternion, quaternion_from_euler

f=open("measure_test_1.txt","w")
f.close()

# callback function for rospy to used
def callback(data):
    f=open("measure_test_1.txt","a")
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
        str_ = str_ + "\n"

    print(str_)
    f.write(str_)
    f.close()


if __name__ == '__main__':
    f=open("measure_test_1.txt","a")
    f.write("ID,x,y,z(position),x,y,z,w(orientation)\n")
    f.close()
    rospy.init_node('SendDataToSTM', anonymous=True)
    rospy.Subscriber("Camera_Data", Camera_Data, callback)
    rospy.spin()
