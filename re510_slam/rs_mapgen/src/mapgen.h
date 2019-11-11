#ifndef MAPGEN_H
#define MAPGEN_H

#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <opencv2/opencv.hpp>
#include <tf/tf.h>

class mapgen
{
private:
  float imageResolution;
  float mapCenterX;
  float mapCenterY;
  cv::Mat gridMap;
  Eigen::Matrix4f tf_laser2robot;
  float occupancyIncrease;
  float occupancyDecrease;

public:
  mapgen();
  ~mapgen();
  void updateMap(Eigen::Matrix4f pose, Eigen::Matrix4Xf laser);
};

#endif

