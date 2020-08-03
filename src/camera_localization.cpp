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
fiducial_msgs::FiducialTransformArray   aruco_;
localization::Camera_Data               camdata;
tf::StampedTransform                    map2base;
tf::StampedTransform                    cam2base;
tf::Transform                           cam2pos;//  from origin to camera

// initial tag variable
int origin = 1;
int robot  = 2;

void MarkerCallback(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg){
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

      cam2pos.setOrigin(  tf::Vector3(aruco_.transforms[i].transform.translation.x,aruco_.transforms[i].transform.translation.y,aruco_.transforms[i].transform.translation.z));
      cam2pos.setRotation( tf::Quaternion(aruco_.transforms[i].transform.rotation.x,aruco_.transforms[i].transform.rotation.y,aruco_.transforms[i].transform.rotation.z ,aruco_.transforms[i].transform.rotation.w));
      // from pos to camera then camera to base => pos to base => map origin to base link, let the origin pose be map
      // it become map to base link
      cam2pos = cam2pos.inverse(); // become from origin to camera
      //map2base = cam2pos * cam2base;
      map2base.setData(cam2pos * cam2base);
      // publish at main while loop

      camdata.Camera_Pose.position.x = cam2pos.getOrigin().x();
      camdata.Camera_Pose.position.y = cam2pos.getOrigin().y();
      camdata.Camera_Pose.position.z = cam2pos.getOrigin().z();

    }else{
        // do update of camera
        // especially for multiple tag read
        tf::TransformListener listener;

        // find from camera frame to origin => 
        // become from origin to camera
        tf::Quaternion q = cam2pos.getRotation();
        tf::Matrix3x3 mr(q);

        //try {
        //    std::string ss1 = "/camera_color_optical_frame";
        //    std::string ss2 = "/fiducial_";
        //    ss2.append(std::to_string(origin));
        //    // from cam to origin
        //    // listener.waitForTransform(ss2, ss1, ros::Time(0), ros::Duration(0.001));
        //    listener.lookupTransform(ss2, ss1, ros::Time(0), transform);
        //} catch (tf::TransformException ex) {
        //    std::cout << "\nNOT FOUND --" << ex.what() << "\n";
        //}

        //// rotation from rotation and translation
        //tf::Quaternion q = transform.getRotation();
        //tf::Matrix3x3 mr(q);

        double xx = aruco_.transforms[i].transform.translation.x;
        double yy = aruco_.transforms[i].transform.translation.y;
        double zz = aruco_.transforms[i].transform.translation.z;

        //// do rotation and translation to get 'x y z' wrt origin 
        double x = mr[0][0] * xx + mr[0][1] * yy + mr[0][2] * zz + cam2pos.getOrigin().x();
        double y = mr[1][0] * xx + mr[1][1] * yy + mr[1][2] * zz + cam2pos.getOrigin().y();
        double z = mr[2][0] * xx + mr[2][1] * yy + mr[2][2] * zz + cam2pos.getOrigin().z();

        std::cout << "xyz:" << cam2pos.getOrigin().x() << " " << cam2pos.getOrigin().y() << " " << cam2pos.getOrigin().z() << "\n";
        std::cout << "xyz:" << xx << " " << yy << " " << zz << "\n";
        std::cout << "xyz:" << x << " " << y << " " << z << "\n";

        geometry_msgs::Pose temp;
        temp.position.x = x;
        temp.position.y = y;
        temp.position.z = z;

        // TODO : need to improve relative rotation here !!! 
        temp.orientation.x = aruco_.transforms[i].transform.rotation.x;
        temp.orientation.y = aruco_.transforms[i].transform.rotation.y;
        temp.orientation.z = aruco_.transforms[i].transform.rotation.z;
        temp.orientation.w = aruco_.transforms[i].transform.rotation.w;
        //temp.position.x = transform.getOrigin().x();
        //temp.position.y = transform.getOrigin().y();
        //temp.position.z = transform.getOrigin().z();
        //temp.orientation = 1;
	    // from xyz to length and angle
        if(aruco_.transforms[i].fiducial_id == robot){
          camdata.Robot_Pose.poses.push_back(temp);
        }else{
          camdata.Obstacle_Pose.poses.push_back(temp);
        }
    }
  }
  pub.publish(camdata);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cam_localization");
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
  ros::Subscriber                         sub;
  sub = n.subscribe("fiducial_transforms", 2, MarkerCallback);

  // find transformation from camera lens to base
  tf::TransformListener Listener;
  Listener.waitForTransform("/camera_color_optical_frame","/base_link",ros::Time(0),ros::Duration(5.0));
  Listener.lookupTransform("/camera_color_optical_frame","/base_link",ros::Time(0),cam2base);

  // initialize map 2 base link
  map2base.setOrigin(tf::Vector3(0, 0, 1));
  map2base.setRotation(tf::Quaternion(0, 0, 0, 1));
  ros::Rate loop_rate(100);

  // keep broadcast map 2 base tf
  tf::TransformBroadcaster                broadcast;
  while (ros::ok()){
      broadcast.sendTransform(tf::StampedTransform(map2base, ros::Time::now(), "map","base_link"));
      ros::spinOnce();
      loop_rate.sleep();
  }
  return 0;
}
