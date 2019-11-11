#include <ros/ros.h>
#include "src/mapgen.h"

std::vector<Eigen::Matrix4f> vec_poses;
std::vector<double> vec_poses_time;
std::vector<Eigen::Matrix4Xf> vec_lasers;
std::vector<double>vec_lasers_time;
Eigen::Matrix4f tf_mocap2pose;

mapgen mapgenerator;
void callback_laser(const sensor_msgs::LaserScan::ConstPtr& msg);
void callback_pose(const geometry_msgs::PoseStamped::ConstPtr& msg);
void check_data();

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rs_mapgen");
  ros::NodeHandle nh;
  tf_mocap2pose<< -0.9961947,  0.0871557, 0,0,
                  0.0871557,   0.9961947, 0,0,
                  0,                   0,-1,0,
                  0,                   0, 0,1;
  ros::Subscriber subscribe_laser = nh.subscribe<sensor_msgs::LaserScan>("/scan",100,callback_laser);
  ros::Subscriber subscribe_pose = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/turtleBot/pose",100,callback_pose);

  ros::spin();

  return 0;
}

void check_data()
{
  while((vec_poses.size()!=0 && vec_lasers.size()!=0))
  {
    // fabs: float abs
    if(fabs(vec_poses_time[0] - vec_lasers_time[0])>0.1)
    {
      if(vec_poses_time[0]>vec_lasers_time[0])
      {
        vec_lasers.erase(vec_lasers.begin());
        vec_lasers_time.erase(vec_lasers_time.begin());
      }
      else
      {
        vec_poses.erase(vec_poses.begin());
        vec_poses_time.erase(vec_poses_time.begin());
      }
    }
    else
    {
      mapgenerator.updateMap(vec_poses[0],vec_lasers[0]);
      vec_lasers.erase(vec_lasers.begin());
      vec_lasers_time.erase(vec_lasers_time.begin());
      vec_poses.erase(vec_poses.begin());
      vec_poses_time.erase(vec_poses_time.begin());
    }
  }
}

void callback_laser(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  int scanQuantity =((msg->angle_max)-(msg->angle_min))/(msg->angle_increment)+1;
  Eigen::Matrix4Xf eigenLaser = Eigen::Matrix4Xf::Ones(4, 1);
  int scanEffective = 0;
  for(int i=0;i<scanQuantity;i++){
    float dist = msg->ranges[i];
    if(dist > 1 && dist < 10)
    {
      scanEffective++;
      eigenLaser.conservativeResize(4,scanEffective);
      eigenLaser(0,scanEffective-1) =  dist * cos(msg->angle_min + ( msg->angle_increment * i));
      eigenLaser(1,scanEffective-1) =  dist * sin(msg->angle_min + ( msg->angle_increment * i));
      eigenLaser(2,scanEffective-1) =  0;
      eigenLaser(3,scanEffective-1) =  1;
    }
  }
  vec_lasers.push_back(eigenLaser);
  vec_lasers_time.push_back(msg->header.stamp.toSec());
  check_data();
}

void callback_pose(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  Eigen::Matrix4f eigenPose;
  tf::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
  tf::Matrix3x3 m(q);
  eigenPose<< m[0][0], m[0][1], m[0][2], msg->pose.position.x,
              m[1][0], m[1][1], m[1][2], msg->pose.position.y,
              m[2][0], m[2][1], m[2][2], msg->pose.position.z,
              0,0,0,1;
  eigenPose = eigenPose * tf_mocap2pose;
  vec_poses.push_back(eigenPose);
  vec_poses_time.push_back(msg->header.stamp.toSec());
  check_data();
}
