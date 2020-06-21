#include "ros/ros.h"
#include "std_msgs/String.h"
#include "fiducial_msgs/FiducialTransformArray.h"
#include <tf/transform_broadcaster.h> // publish pose to tf
#include <tf/transform_listener.h> // listen pose to tf
#include <iostream>

tf::StampedTransform cam2base;
fiducial_msgs::FiducialTransformArray aruco_;
std::string origin;

void MarkerCallback(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg)
{
  //ROS_INFO("I hear marker");
  aruco_ = *msg;

  int found_flag = 0;

  for(int i = 0;i < aruco_.transforms.size();i++){
    
    
    if(aruco_.transforms[i].fiducial_id == std::stoi(origin)){
      tf::TransformBroadcaster br;
      tf::Transform cam2pos;
      cam2pos.setOrigin(  tf::Vector3(aruco_.transforms[i].transform.translation.x,aruco_.transforms[i].transform.translation.y,aruco_.transforms[i].transform.translation.z));
    cam2pos.setRotation( tf::Quaternion(aruco_.transforms[i].transform.rotation.x,aruco_.transforms[i].transform.rotation.y,aruco_.transforms[i].transform.rotation.z ,aruco_.transforms[i].transform.rotation.w));

      tf::Transform pos2base;
      pos2base = cam2pos.inverse() * cam2base;
      br.sendTransform(tf::StampedTransform(pos2base, ros::Time::now(), "map","base_link"));

      found_flag = 1;
      ROS_INFO("found origin !");
    }
  }

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
  ros::Subscriber sub = n.subscribe("fiducial_transforms", 2, MarkerCallback);

  tf::TransformListener Listener;
  Listener.waitForTransform("/camera_color_optical_frame","/base_link",ros::Time(0),ros::Duration(5.0));
  Listener.lookupTransform("/camera_color_optical_frame","/base_link",ros::Time(0),cam2base);

  if(n.hasParam("origin_id")) {
    n.getParam("origin_id",origin);
  }else{
    origin = "10";
  }
  ros::spin();

  return 0;
}
