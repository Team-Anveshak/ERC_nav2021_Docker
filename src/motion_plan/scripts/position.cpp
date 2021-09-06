#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Odometry.h"
#include <iostream>
#include "tf/transform_datatypes.h"
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/transform_listener.h>
#include "ar_track_alvar_msgs/AlvarMarkers.h"
#include "ar_track_alvar_msgs/AlvarMarker.h"
#include "nav_msgs/Odometry.h"


geometry_msgs::TwistWithCovarianceStamped twist_cov;
geometry_msgs::PoseStamped raw_land_pose;
nav_msgs::Odometry filtered_odom;
nav_msgs::Odometry true_odom;
// double marker_actual[15][2] = {{9.8, 0.0},
//                               {9.8, 3.5},
//                               {34.00, 1.50},
//                               {(23.63,-4.62)},
//                               {(10.27,9.76)},
//                               {(10.10,-21.38)},
//                               {(5.44,-15.17)},
//                               {(31.00,-9.13)},
//                               {(18.37,11.00)},
//                               {(1.36,9.60)},
//                               {(17.00,-22.46)},
//                               {(19.63,-0.02)},
//                               {(18.29,-13.90)},
//                               {0.0, 0.0},
//                               {(3.02,-17.34)}};
double marker_actual[15][2] = {{5.702769, 0.0},
                              {4.503994, 6.826416},
                              {14.329459 -0.428800},
                              {29.215900, 5.300520},
                              {12.103476, -11.714824},
                              {18.821705, 12.167322},
                              {18.397116, 3.251184},
                              {18.566999 -18.526400},
                              {12.228898, 7.785070},
                              {20.629101, -8.191796},
                              {29.695601, -16.672436},
                              {28.375999, 11.429800},
                              {30.889601, -5.985371},
                              {1.717059, -10.908800},
                              {5.813930, 14.724000}};
nav_msgs::Odometry landmark_pose;
bool publish_land = false;

void calculate_pose(int id, geometry_msgs::PoseStamped transformed_pose)
{

  double x_mul, y_mul, hypot_mul, x = transformed_pose.pose.position.x, y = transformed_pose.pose.position.y;
  hypot_mul = sqrt((x - filtered_odom.pose.pose.position.x)*(x - filtered_odom.pose.pose.position.x) + 
                    (y - filtered_odom.pose.pose.position.y)*(y - filtered_odom.pose.pose.position.y));
  x_mul = (x - filtered_odom.pose.pose.position.x)/hypot_mul;
  y_mul = (y - filtered_odom.pose.pose.position.y)/hypot_mul;

  landmark_pose.pose.pose.position.x = filtered_odom.pose.pose.position.x - (x+0.125*x_mul - marker_actual[id-1][0]);
  landmark_pose.pose.pose.position.y = filtered_odom.pose.pose.position.y - (y+0.125*y_mul - marker_actual[id-1][1]);
  std::cout<<id<<" Marker Error in x,y: "<<(landmark_pose.pose.pose.position.x-true_odom.pose.pose.position.x)<<" "<<(landmark_pose.pose.pose.position.y-true_odom.pose.pose.position.y)<<"\n";
  std::cout<<x<<" "<<y<<"\n";
}


void clk_wheel_odom(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
  twist_cov.header.frame_id = msg->header.frame_id;
  twist_cov.header.seq = msg->header.seq;
  twist_cov.header.stamp = msg->header.stamp;
  twist_cov.twist.twist = msg->twist;
}

void clk_filtered_odom(const nav_msgs::Odometry::ConstPtr& msg)
{
  if(msg->header.frame_id == "map")
    filtered_odom = *msg;
}

void clk_true_odom(const nav_msgs::Odometry::ConstPtr& msg)
{
  true_odom = *msg;
}

void clk_marker(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg)
{
  static tf::TransformListener listener(ros::Duration(10));
  for(int i = 0; i < msg->markers.size(); i++)
  {
    if(msg->markers[i].id >= 1 && msg->markers[i].id <= 15 && msg->markers[i].id != 14)
    {
      landmark_pose.header.seq = msg->header.seq;
      landmark_pose.header.stamp = msg->header.stamp;
      raw_land_pose = msg->markers[i].pose;
      raw_land_pose.header.frame_id = msg->markers[i].header.frame_id;

      geometry_msgs::PoseStamped transformed_pose;
      try {
        listener.transformPose("map", raw_land_pose, transformed_pose);
      }
      catch(tf::TransformException& ex){
        ROS_ERROR("Received an exception trying to transform a point from base_footprint to %s: %s", raw_land_pose.header.frame_id.c_str(), ex.what());
      }

      calculate_pose(msg->markers[i].id, transformed_pose);
      publish_land = true;
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "position");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<geometry_msgs::TwistWithCovarianceStamped>("/wheel_odom_cov", 1);
  ros::Publisher pub1 = n.advertise<nav_msgs::Odometry>("/landmark/odom", 1);
  ros::Subscriber sub = n.subscribe("/wheel_odom", 1, clk_wheel_odom);
  ros::Subscriber sub1 = n.subscribe("/odometry/filtered", 1, clk_filtered_odom);
  ros::Subscriber sub2 = n.subscribe("/ground_truth", 1, clk_true_odom);
  ros::Subscriber sub3 = n.subscribe("/ar_pose_marker", 1, clk_marker);


  double roll, pitch, yaw_true, yaw_filtered, marker_est_x, marker_est_y;
  tf::Quaternion quat;

  landmark_pose.child_frame_id = "map";
  landmark_pose.header.frame_id = "map";
  landmark_pose.pose.covariance = { 0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.1};

  twist_cov.twist.covariance = {1e-4, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 1e-4, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 1e-4, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 1e-4, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 1e-4, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 1e-4};

  ros::Rate loop_rate(2);
  loop_rate.sleep();
  double x_mul, y_mul, hypot_mul;
  while (ros::ok())
  {
    tf::quaternionMsgToTF(filtered_odom.pose.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw_filtered);
    tf::quaternionMsgToTF(true_odom.pose.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw_true);

    std::cout<<"Error in x,y,yaw: "<<(filtered_odom.pose.pose.position.x-true_odom.pose.pose.position.x)<<" "
              <<(filtered_odom.pose.pose.position.y-true_odom.pose.pose.position.y)<<" "<<(yaw_filtered-yaw_true)<<"\n";
    

    pub.publish(twist_cov);
    if(publish_land == true)
    {
      publish_land = false;
      pub1.publish(landmark_pose);
    }

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}