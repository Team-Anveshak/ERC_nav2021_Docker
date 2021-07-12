#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/Path.h"
#include <utility>
#include <stack>
#include <set>

#define WIDTH 400
#define HEIGHT 400
#define RESOLUTION 0.05

//(0,0) is bottom right
//Required Inflation: 0.35

nav_msgs::OccupancyGrid map;
geometry_msgs::Point goal;
geometry_msgs::Point rover_position;

void chatterCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  map = *msg;
}

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  goal.x = msg->pose.position.x;
  goal.y = msg->pose.position.y;
  goal.z = msg->pose.position.z;
}

void positionCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  rover_position.x = msg->pose.pose.position.x;
  rover_position.y = msg->pose.pose.position.y;
  rover_position.z = msg->pose.pose.position.z;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "global_mapper");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<nav_msgs::OccupancyGrid>("/global_costmap", 1);
  ros::Publisher pub_plan = n.advertise<nav_msgs::Path>("/global_plan", 1);
  ros::Subscriber sub = n.subscribe("/grid_map_visualization/traversability_grid", 1, chatterCallback);
  ros::Subscriber sub_goal = n.subscribe("/move_base_simple/goal", 1, goalCallback);
  ros::Subscriber sub_pose = n.subscribe("/odometry/filtered/pose", 1, positionCallback);

  nav_msgs::OccupancyGrid global_map;

  global_map.header.frame_id = "/map";
  global_map.info.height = HEIGHT;
  global_map.info.width = WIDTH;
  global_map.info.resolution = RESOLUTION;

  geometry_msgs::Pose global_map_origin;
  global_map_origin.position.x = -10;
  global_map_origin.position.y = -10;
  global_map_origin.position.z = 0;

  global_map_origin.orientation.w = 1;
  global_map_origin.orientation.x = 0;
  global_map_origin.orientation.y = 0;
  global_map_origin.orientation.z = 0;

  global_map.info.origin = global_map_origin;

  std::vector<int8_t> vec(HEIGHT*WIDTH, -1);
  global_map.data =  vec;

  ros::Rate loop_rate(1);
  int i, j, x, y;
  while (ros::ok())
  {
    global_map.header.stamp = map.header.stamp;
    global_map.header.seq = map.header.seq;
    global_map.info.map_load_time = map.info.map_load_time;


    for(i = 5; i < map.info.height; i++)
        for(j = 5; j < map.info.width; j++)
        {

            if(i >= map.info.width - 5 || j >= map.info.height - 5)
                continue;
            if(map.data[j + i*map.info.width] > 50)
            {
              x = int((map.info.origin.position.y - global_map.info.origin.position.y)/global_map.info.resolution + i*map.info.resolution/global_map.info.resolution);
              y = int((map.info.origin.position.x - global_map.info.origin.position.x)/global_map.info.resolution + j*map.info.resolution/global_map.info.resolution);
              global_map.data[y + x*global_map.info.width] = 0;
            }
        }
    for(i = 8; i < map.info.height; i++)
        for(j = 8; j < map.info.width; j++)
        {
            if(i >= map.info.width - 10 || j >= map.info.height - 10)
                continue;
            if(map.data[j + i*map.info.width] <= 50)
            {
              x = int((map.info.origin.position.y - global_map.info.origin.position.y)/global_map.info.resolution + i*map.info.resolution/global_map.info.resolution);
              y = int((map.info.origin.position.x - global_map.info.origin.position.x)/global_map.info.resolution + j*map.info.resolution/global_map.info.resolution);
            //   for(int i1 = 0; i1 < 7; i1++)
            //     for(int i2 = 0; i2 < 7; i2++)
            //     {
            //       if(i1*i1 + i2*i2 < 49)
            //       {
                     global_map.data[y + 0 + (x + 0)*global_map.info.width] = 100;  //y+i1
            //       }
            //     }
            }
        }

    pub.publish(global_map);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}