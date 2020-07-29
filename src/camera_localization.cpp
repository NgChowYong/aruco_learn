#include "ros/ros.h"
#include "fiducial_msgs/FiducialTransformArray.h"
#include "localization/Camera_Data.h"
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
localization::Camera_Data               camdata;
tf::StampedTransform 			transform;
// initial tag variable
int origin = 1;
int robot  = 2;

void MarkerCallback(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg)
{
  //ROS_INFO("I hear marker");
  aruco_ = *msg;

  camdata.Obstacle_Pose.poses.erase(camdata.Obstacle_Pose.poses.begin(),camdata.Obstacle_Pose.poses.end());
  camdata.Robot_Pose.poses.erase(camdata.Robot_Pose.poses.begin(),camdata.Robot_Pose.poses.end());

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
      cam2pos = cam2pos.inverse();
      pos2base = cam2pos * cam2base;
      br.sendTransform(tf::StampedTransform(pos2base, ros::Time::now(), "map","base_link"));
  
      camdata.Camera_Pose.position.x = cam2pos.getOrigin().x();
      camdata.Camera_Pose.position.y = cam2pos.getOrigin().y();
      camdata.Camera_Pose.position.z = cam2pos.getOrigin().z();

    }else{
        // do update of camera
        // especially for multiple tag read
        tf::TransformListener listener;

        // find from camera frame to origin
	// do once is enough ?
        try {
            std::string ss1 = "/camera_color_optical_frame";
            std::string ss2 = "/fiducial_";
            ss2.append(std::to_string(origin));
            // from cam to origin
            // listener.waitForTransform(ss2, ss1, ros::Time(0), ros::Duration(0.001));
            listener.lookupTransform(ss2, ss1, ros::Time(0), transform);
        } catch (tf::TransformException ex) {
            std::cout << "\nNOT FOUND --" << ex.what() << "\n";
        }

        // rotation from rotation and translation
        tf::Quaternion q = transform.getRotation();
        tf::Matrix3x3 mr(q);
        float xx = aruco_.transforms[i].transform.translation.x;
        float yy = aruco_.transforms[i].transform.translation.y;
        float zz = aruco_.transforms[i].transform.translation.z;

        // do rotation and translation to get 'x y z' wrt origin 
        float x = mr[0][0] * xx + mr[0][1] * yy + mr[0][2] * zz + transform.getOrigin().x();
        float y = mr[1][0] * xx + mr[1][1] * yy + mr[1][2] * zz + transform.getOrigin().y();
        float z = mr[2][0] * xx + mr[2][1] * yy + mr[2][2] * zz + transform.getOrigin().z();

        std::cout << "xyz:" << x << " " << y << " " << z << "\n";
        
        geometry_msgs::Pose temp;
        //temp.position.x = x;
        //temp.position.y = y;
        //temp.position.z = z;
        temp.position.x = transform.getOrigin().x();
        temp.position.y = transform.getOrigin().y();
        temp.position.z = transform.getOrigin().z();
        //temp.orientation = 1;
	// from xyz to length and angle
        if(aruco_.transforms[i].fiducial_id == robot){
          camdata.Robot_Pose.poses.push_back(temp);
        }else{
          camdata.Obstacle_Pose.poses.push_back(temp);
        }
    }
  }

  // if not fond origin of map just give a coordinate
  /*
  if (found_flag == 0){
      tf::TransformBroadcaster br;
      tf::Transform original;
      original.setOrigin(  tf::Vector3(0,0,1));
      original.setRotation( tf::Quaternion(0,0,0,1));
      br.sendTransform(tf::StampedTransform(original, ros::Time::now(), "map","base_link"));
  }
  */

  pub.publish(camdata);
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

  pub = n.advertise<localization::Camera_Data>("Camera_Data", 10);
  sub = n.subscribe("fiducial_transforms", 2, MarkerCallback);

  // find transformation from camera lens to base
  tf::TransformListener Listener;
  Listener.waitForTransform("/camera_color_optical_frame","/base_link",ros::Time(0),ros::Duration(5.0));
  Listener.lookupTransform("/camera_color_optical_frame","/base_link",ros::Time(0),cam2base);

  tf::TransformBroadcaster br;
  tf::Transform original;
  original.setOrigin(  tf::Vector3(0,0,1));
  original.setRotation( tf::Quaternion(0,0,0,1));
  br.sendTransform(tf::StampedTransform(original, ros::Time::now(), "map","base_link"));
 

  ros::spin();

  return 0;
}
