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
double marker_actual[15][2] = {{9.8, 0.0},
                              {9.8, 3.5},
                              {34.00, 1.50},
                              {(23.63,-4.62)},
                              {(10.27,9.76)},
                              {(10.10,-21.38)},
                              {(5.44,-15.17)},
                              {(31.00,-9.13)},
                              {(18.37,11.00)},
                              {(1.36,9.60)},
                              {(17.00,-22.46)},
                              {(19.63,-0.02)},
                              {(18.29,-13.90)},
                              {0.0, 0.0},
                              {(3.02,-17.34)}};
nav_msgs::Odometry landmark_pose;
bool publish_land = false;

void calculate_pose(int id, geometry_msgs::PoseStamped transformed_pose, geometry_msgs::PoseStamped raw_pose)
{

  double x_mul, y_mul, hypot_mul, x = transformed_pose.pose.position.x, y = transformed_pose.pose.position.y;
  hypot_mul = sqrt((x - filtered_odom.pose.pose.position.x)*(x - filtered_odom.pose.pose.position.x) + 
                    (y - filtered_odom.pose.pose.position.y)*(y - filtered_odom.pose.pose.position.y));
  x_mul = (x - filtered_odom.pose.pose.position.x)/hypot_mul;
  y_mul = (y - filtered_odom.pose.pose.position.y)/hypot_mul;

  double dist, dist_t, dist_avg, t, x_slope, y_slope, mark_angle, x_shift, y_shift, x_avg, y_avg;
  dist_t = sqrt(raw_pose.pose.position.x*raw_pose.pose.position.x + raw_pose.pose.position.y*raw_pose.pose.position.y);
  dist = sqrt((marker_actual[id-1][0] - filtered_odom.pose.pose.position.x)*(marker_actual[id-1][0] - filtered_odom.pose.pose.position.x) + 
              (marker_actual[id-1][1] - filtered_odom.pose.pose.position.y)*(marker_actual[id-1][1] - filtered_odom.pose.pose.position.y));
  t = dist_t/dist;
  x_slope = t * filtered_odom.pose.pose.position.x + (1 - t) * marker_actual[id-1][0];
  y_slope = t * filtered_odom.pose.pose.position.y + (1 - t) * marker_actual[id-1][1];

  mark_angle = atan2(raw_pose.pose.position.y, raw_pose.pose.position.x);

  x_shift = filtered_odom.pose.pose.position.x - (x+0.125*x_mul - marker_actual[id-1][0]);
  y_shift = filtered_odom.pose.pose.position.y - (y+0.125*y_mul - marker_actual[id-1][1]);

  x_avg = (x_slope + x_shift)/2;
  y_avg = (y_slope + y_shift)/2;
  dist = sqrt((marker_actual[id-1][0] - x_avg)*(marker_actual[id-1][0] - x_avg) + 
              (marker_actual[id-1][1] - y_avg)*(marker_actual[id-1][1] - y_avg));
  t = dist_t/dist;

  landmark_pose.pose.pose.position.x = t * x_avg + (1 - t) * marker_actual[id-1][0];
  landmark_pose.pose.pose.position.y = t * y_avg + (1 - t) * marker_actual[id-1][1];

  double roll, pitch, yaw, land_angle;
  land_angle = atan2(marker_actual[id-1][1] - landmark_pose.pose.pose.position.y, marker_actual[id-1][0] - landmark_pose.pose.pose.position.x);
  
  tf::Quaternion quat, quat2;
  tf::quaternionMsgToTF(filtered_odom.pose.pose.orientation, quat);
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  yaw = yaw + land_angle - mark_angle;

  quat2.setRPY(roll, pitch, yaw);

  tf::quaternionTFToMsg(quat2, landmark_pose.pose.pose.orientation);

  std::cout<<id<<" Marker x,y: "<<landmark_pose.pose.pose.position.x<<" "<<landmark_pose.pose.pose.position.y<<"\n";
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

      calculate_pose(msg->markers[i].id, transformed_pose, raw_land_pose);
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
  ros::Subscriber sub3 = n.subscribe("/ar_pose_marker", 1, clk_marker);


  double roll, pitch, yaw_filtered, marker_est_x, marker_est_y;
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

    std::cout<<"x,y,yaw: "<<(filtered_odom.pose.pose.position.x)<<" "
              <<(filtered_odom.pose.pose.position.y)<<" "<<(yaw_filtered)<<"\n";
    

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