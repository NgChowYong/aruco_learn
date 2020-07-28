#include "ros/ros.h"
#include "std_msgs/String.h"
#include "fiducial_msgs/FiducialTransformArray.h"
#include <tf/transform_broadcaster.h> // publish pose to tf
#include <tf/transform_listener.h> // listen pose to tf
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <string>

// global variable or handler
ros::Publisher                          pub;
ros::Subscriber                         sub;
tf::StampedTransform                    cam2base;
fiducial_msgs::FiducialTransformArray   aruco_;
std_msgs::String                        string_output;

// initial tag variable
int origin = 1;
int robot  = 2;
int range  = 3;


std::string float_to_string(float f,std::string n){ 
  // float to mm then to string
  return std::to_string(static_cast<int>(f*1000)) + n;
}


void MarkerCallback(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg)
{
  //ROS_INFO("I hear marker");
  aruco_ = *msg;

  string_output = "empty string";
  pub.publish(string_output);

  /* 
  * for all tag recognize
  * see whether it is origin
  * if it is send origin
  * see whether it is robot 
  * if it is transform robot coordinate to length and angle relative to origin then send
  * see whether it is obstacle
  * if it is store obstacle location
  * send the obs location
  */
  for(int i = 0;i < aruco_.transforms.size();i++){
    if(aruco_.transforms[i].fiducial_id == origin){
      ROS_INFO("found origin !");
      tf::TransformBroadcaster br;
      tf::Transform cam2pos;
      cam2pos.setOrigin(  tf::Vector3(aruco_.transforms[i].transform.translation.x,aruco_.transforms[i].transform.translation.y,aruco_.transforms[i].transform.translation.z));
      cam2pos.setRotation( tf::Quaternion(aruco_.transforms[i].transform.rotation.x,aruco_.transforms[i].transform.rotation.y,aruco_.transforms[i].transform.rotation.z ,aruco_.transforms[i].transform.rotation.w));

      // from pos to camera then camera to base => pos to base, let the origin pose be map
      // it become map to base link
      tf::Transform pos2base;
      pos2base = cam2pos.inverse() * cam2base;
      br.sendTransform(tf::StampedTransform(pos2base, ros::Time::now(), "map","base_link"));
    }else if(aruco_.transforms[i].fiducial_id == robot){
        // do update of camera
        // especially for multiple tag read
        tf::TransformListener listener;
        tf::StampedTransform transform;

        // find from camera frame to origin
        try {
            std::string ss1 = "/camera_color_optical_frame";
            std::string ss2 = "/fiducial_";
            ss2.append(std::to_string(origin));
            // from cam to origin
            listener.waitForTransform(ss2, ss1, ros::Time(0), ros::Duration(2.0));
            listener.lookupTransform(ss2, ss1, ros::Time(0), transform);
        } catch (tf::TransformException ex) {
            std::cout << "\nNOT FOUND --" << ex.what() << "\n";
        }

        // rotation from ??? 
        tf::Quaternion q = transform.getRotation();
        tf::Matrix3x3 mr(q);
        float xx = aruco_.transforms[i].transform.translation.x;
        float yy = aruco_.transforms[i].transform.translation.y;
        float zz = aruco_.transforms[i].transform.translation.z;

        // do rotation to get 'x y z' of tag rel to 
        float x = mr[0][0] * xx + mr[0][1] * yy + mr[0][2] * zz + transform.getOrigin().x();
        float y = mr[1][0] * xx + mr[1][1] * yy + mr[1][2] * zz + transform.getOrigin().y();
        float z = mr[2][0] * xx + mr[2][1] * yy + mr[2][2] * zz + transform.getOrigin().z();

        std::cout << "xyz:" << x << " " << y << " " << z << "\n";
    }else{

    }
    //std::cout <<"\noutput:"<< output;

  }

  // if not fond origin of map just give a coordinate
  if (found_flag == 0){
      tf::TransformBroadcaster br;
      tf::Transform original;
      original.setOrigin(  tf::Vector3(0,0,1));
      original.setRotation( tf::Quaternion(0,0,0,1));
      br.sendTransform(tf::StampedTransform(original, ros::Time::now(), "map","base_link"));
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cam_tf");
  ros::NodeHandle n;
  if(ros::param::has("/origin_id")) {
    ros::param::get("/origin_id",origin);
  }else{
    origin = 1;
  }
  std::cout << "origin: "<< origin <<"\n";

  if(n.hasParam("robot_id")) {
    n.getParam("robot_id",robot);
  }else{
    robot = 2;
  }
  std::cout << "robot: "<< robot <<"\n";

  pub = n.advertise<std_msgs::String>("Camera_Data", 10);
  sub = n.subscribe("fiducial_transforms", 2, MarkerCallback);

  // find transformation from camera lens to base
  tf::TransformListener Listener;
  Listener.waitForTransform("/camera_color_optical_frame","/base_link",ros::Time(0),ros::Duration(5.0));
  Listener.lookupTransform("/camera_color_optical_frame","/base_link",ros::Time(0),cam2base);

  ros::spin();

  return 0;
}
