#include <ros/ros.h>
#include <aruco_mapping.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <cstdio>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

//custom messages
#include <cm_aruco_msgs/Aruco_marker.h>

using namespace std;


class cm_Marker
{
public:
  tf::TransformBroadcaster br;
	tf::Transform transform;
  ros::Subscriber marker_info_sub;
  ros::Subscriber amcl_info_sub;

  geometry_msgs::Pose marker_pose;
  // geometry_msgs::Point marker_position;
  // geometry_msgs::Quaternion marker_orientation;
  string s;
  double x;
  double y;
  
  cm_Marker(ros::NodeHandle *nh)
  {
    marker_info_sub = nh -> subscribe("/aruco_poses",1, &cm_Marker::sub_marker_MsgCallback, this);
  }
  void sub_marker_MsgCallback(const cm_aruco_msgs::Aruco_marker::ConstPtr& msg);
  void cm_navigation();
  void tf_publisher();
};

void cm_Marker::sub_marker_MsgCallback(const cm_aruco_msgs::Aruco_marker::ConstPtr& msg)
{
  marker_pose.position.x = msg -> global_camera_pose.position.x;
  marker_pose.position.y = msg -> global_camera_pose.position.y;
  marker_pose.position.z = msg -> global_camera_pose.position.z;
  marker_pose.orientation.x = msg -> global_camera_pose.orientation.x;
  marker_pose.orientation.y = msg -> global_camera_pose.orientation.y;
  marker_pose.orientation.z = msg -> global_camera_pose.orientation.z;
  marker_pose.orientation.w = msg -> global_camera_pose.orientation.w;
}

void cm_Marker::tf_publisher()
{

  // transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
  // transform.setRotation(tf::createQuaternionFromRPY(0.0, 0.0, 0.0));
  // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "odom"));

  // ROS_INFO("YESSSSSSSSSSSSSSSSSS");
  transform.setOrigin( tf::Vector3(0.3, 0.0, 0.2) );
  transform.setRotation(tf::createQuaternionFromRPY(-90.0*M_PI/180.0, 0.0, -90.0*M_PI/180.0));
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "camera_0"));

  // transform.setOrigin( tf::Vector3(marker_pose.position.x, marker_pose.position.y, marker_pose.position.z) );
  // tf::Quaternion MarkerQ(marker_pose.orientation.x,marker_pose.orientation.y,marker_pose.orientation.z,marker_pose.orientation.w); // yaw pitch roll
  // transform.setRotation(MarkerQ);
  // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_0", "marker"));

  transform.setOrigin( tf::Vector3(0.0, 0.0, 0.6 ));
  transform.setRotation(tf::createQuaternionFromRPY(0.0,90.0*M_PI/180,0.0));
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "marker", "safe_link"));

  // transform.setOrigin( tf::Vector3(marker_pose.position.x-0.5, marker_pose.position.y-0.5, marker_pose.position.z) );
  // // transform.setRotation(MarkerQ);
  // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_link", "safe_link"));

}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_navigation_goals");
  ros::NodeHandle nh("~");

  cm_Marker cm_Markers(&nh);
  
 
  ros::Rate loop_rate(10);
  while(ros::ok())
  {
    cm_Markers.tf_publisher();
    // cm_Markers.cm_navigation();
    ros::spinOnce();
    loop_rate.sleep();
  }
  

  return 0;
}
