#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <aruco_mapping.h>
#include <tf_conversions/tf_eigen.h>
#include <nav_msgs/Odometry.h>
#include <cm_aruco_msgs/Aruco_marker.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

class cm_pose_checker
{
public:
    tf::TransformListener listener;
    tf::StampedTransform transform;

    tf::TransformListener listener2;
    tf::StampedTransform transform2;

    ros::Subscriber marker_pose_info_sub;
    ros::Subscriber odom_sub;
    ros::Publisher vel_pub;

    geometry_msgs::Pose marker_pose;
    geometry_msgs::Twist create_vel;
    nav_msgs::Odometry odom_pose;

    double distance;
    double R,P,Y;
    double Roll, Pitch, Yaw;

    // we need to optimize the following value
    double linear_vel_alpha = 1.1;    
    double angular_vel_r = 0.3;

    cm_pose_checker(ros::NodeHandle *nh)
    {
        marker_pose_info_sub = nh -> subscribe("/aruco_poses",1, &cm_pose_checker::sub_marker_MsgCallback, this);
        odom_sub = nh -> subscribe("/odom",1,&cm_pose_checker::sub_odom_MsgCallback,this);
        vel_pub = nh -> advertise<geometry_msgs::Twist>("/cmd_vel",1);
    }
public:    
    void sub_marker_MsgCallback(const cm_aruco_msgs::Aruco_marker::ConstPtr& msg);
    void sub_odom_MsgCallback(const nav_msgs::Odometry::ConstPtr&msg);
    void cal_distance();
    void cal_navigation();

};

void cm_pose_checker::cal_navigation()
{

}
void cm_pose_checker::sub_odom_MsgCallback(const nav_msgs::Odometry::ConstPtr&msg)
{
    odom_pose.pose.pose.position.x = msg -> pose.pose.position.x;
    odom_pose.pose.pose.position.y = msg -> pose.pose.position.y;
    odom_pose.pose.pose.orientation.x = msg -> pose.pose.orientation.x;
    odom_pose.pose.pose.orientation.y = msg -> pose.pose.orientation.y;
    odom_pose.pose.pose.orientation.z = msg -> pose.pose.orientation.z;
    odom_pose.pose.pose.orientation.w = msg -> pose.pose.orientation.w;
    // ROS_INFO("Yaw from robot is %.5f", odom_pose.pose.pose.orientation.z);
}

void cm_pose_checker::cal_distance()
{
    try
    {
        // we meed fix from map to base_link because we do not need a map
        // listener.lookupTransform("/map", "/base_link",ros::Time(0), transform);
        listener2.lookupTransform("/base_link", "/safe_link",ros::Time(0), transform2);
        // ROS_INFO("xx %.3f yy %.3f zz %.3f", transform.getOrigin().x(),transform.getOrigin().y(),transform.getOrigin().z());
        
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        // continue;
    }
    // distance = sqrt(pow(transform.getOrigin().x() - transform2.getOrigin().x(), 2) +
    //  pow(transform.getOrigin().y() - transform2.getOrigin().y(), 2) );

    // from base_link to marker_position
    distance = sqrt(pow(transform2.getOrigin().x(), 2) + pow(transform2.getOrigin().y(), 2) );

    ROS_INFO("=========================================");
    // ROS_INFO_STREAM("======================================");
    // ROS_INFO("Base_link position from the map, X is %.3f Y is %.3f",transform.getOrigin().x(),transform.getOrigin().y());
    ROS_INFO("Marker position from the base_link, X is %.3f Y is %.3f",transform2.getOrigin().x(),transform2.getOrigin().y());
    
    ROS_INFO("distance from robot_base_link to safe_link(marker.z + 0.6) is %.3f", distance);

    transform.setOrigin( tf::Vector3(transform2.getOrigin().x(),transform2.getOrigin().y(),transform2.getOrigin().z()) );
    tf::Quaternion MarkerQ(transform2.getRotation().x(),transform2.getRotation().y(),transform2.getRotation().z(),transform2.getRotation().w()); // yaw pitch roll

    tf::Matrix3x3 m(MarkerQ);

    m.getRPY(R,P,Y);

    transform.setOrigin( tf::Vector3(odom_pose.pose.pose.position.x, odom_pose.pose.pose.position.y, odom_pose.pose.pose.position.z ));
    tf::Quaternion aMarkerQ(odom_pose.pose.pose.orientation.x, odom_pose.pose.pose.orientation.y, odom_pose.pose.pose.orientation.z, odom_pose.pose.pose.orientation.w);
    tf::Matrix3x3 n(aMarkerQ);

    n.getRPY(Roll,Pitch,Yaw);

    // -180 < Odom_Yaw & Safe_link_Y < 180 
    // if (Y < 0) Y = Y * 180.0 / M_PI + 360;
    ROS_INFO("goal Degree is %.3f", Y * 180.0 / M_PI);
    
    // if (Yaw < 0) Yaw = Yaw * 180.0 / M_PI + 360;
    ROS_INFO("odom Yaw Degree is %.3f", Yaw * 180.0 / M_PI);
    
    double yaw_error = Yaw - Y;
    ROS_INFO("Error Degree is %.3f", yaw_error* 180.0 / M_PI);

    create_vel.linear.x =  linear_vel_alpha * (0.3 * distance - 0.15); 
    if (create_vel.linear.x >0.4) create_vel.linear.x = 0.4;

    // if(abs(Yaw) > abs(Y)) create_vel.angular.z = 1 * (angular_vel_r *yaw_error);
    // else create_vel.angular.z = -1 * (angular_vel_r * yaw_error);

    // version 1 should fix
    if (Y > 0 && Y < 90 ) 
        if( Yaw > Y ) create_vel.angular.z = -1 * (angular_vel_r * yaw_error);
        else create_vel.angular.z = 1 * (angular_vel_r * yaw_error);

    else
        if(Yaw < Y) create_vel.angular.z = 1 * (angular_vel_r * yaw_error);
        else create_vel.angular.z = -1 * (angular_vel_r * yaw_error); 

    // version 2 else should fix
    // if (Yaw < 180 && Y < 180)  create_vel.angular.z = angular_vel_r * (-(Yaw- Y));
    // else if (Yaw < 180 && Y > 180) create_vel.angular.z = angular_vel_r * (-(Yaw + 360 - Y));
    // else if (Yaw > 180 && Y < 180) create_vel.angular.z = angular_vel_r * ((-Yaw + 360 + Y));
    // else create_vel.angular.z = 0.1; 

    //version 3 Python Robotics
    /* 
    xdiff is the position of target with respect to current robot position in x direction : transform2.getOrigin().x()
    ydiff is the position of target with respect to current robot position in y direction : transform2.getOrigin().y()
    theta is the current heading angle of robot with respect to x axis                    : Yaw
    theta_goal is the target angle of robot with respect to x axis                        : Y
    */
    double alpha = (atan2(transform2.getOrigin().y(),transform2.getOrigin().x()) - Yaw + M_PI);
    if (alpha < -180) alpha = 360 + alpha - M_PI;
    else if (alpha > 180) alpha = alpha - 360 - M_PI;

    double beta = Y - Yaw - alpha + M_PI ; // how to chnage -pi ~ pi mod;



    // create_vel.angular.z = r * (yaw_error);

    // if (distance < 0.5) create_vel.linear.x = 0.0;
    // else create_vel.linear.x = 0.3;
    vel_pub.publish(create_vel);

}

void cm_pose_checker::sub_marker_MsgCallback(const cm_aruco_msgs::Aruco_marker::ConstPtr& msg)
{
  marker_pose.position.x = msg -> global_camera_pose.position.x;
  marker_pose.position.y = msg -> global_camera_pose.position.y;
  marker_pose.position.z = msg -> global_camera_pose.position.z;
  marker_pose.orientation.x = msg -> global_camera_pose.orientation.x;
  marker_pose.orientation.y = msg -> global_camera_pose.orientation.y;
  marker_pose.orientation.z = msg -> global_camera_pose.orientation.z;
  marker_pose.orientation.w = msg -> global_camera_pose.orientation.w;

//   ROS_INFO("Marker Pose X is %.3f Y is %.3f Z is %.3f", marker_pose.position.x, marker_pose.position.y, marker_pose.position.z);
//   ROS_INFO("Marker Yaw is %.3f",marker_pose.orientation.z);
}



int main(int argc, char** argv){
    ros::init(argc, argv, "pose_checker");
    ros::NodeHandle nh("~");
    cm_pose_checker cm_pose_checkers(&nh);
    ros::Rate rate(2.0);

    while (ros::ok())
    {
        cm_pose_checkers.cal_distance();
        cm_pose_checkers.cal_navigation();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
};