#ifndef MCL_H
#define MCL_H

#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <opencv2/opencv.hpp>
#include <tf/tf.h>
#include <random>
#include "tool.h"
#include <cmath>

class mcl
{
  struct particle{
    Eigen::Matrix4f pose;
    float score;
    Eigen::Matrix4Xf scan; // Only for maximum probability particle.
  };

private:
  int m_sync_count;

  std::random_device rd;  //Will be used to obtain a seed for the random number engine
  std::mt19937 gen; //Standard mersenne_twister_engine seeded with rd()

  float imageResolution;
  float mapCenterX;
  float mapCenterY;
  float odomCovariance[6];
  int numOfParticle;
  std::vector<particle> particles;
  particle maxProbParticle;
  cv::Mat gridMap; // Gridmap for showing
  cv::Mat gridMapCV; // Gridmap for use (gaussian-blurred)
  Eigen::Matrix4f tf_laser2robot;
  Eigen::Matrix4f odomBefore;
  float minOdomDistance;
  float minOdomAngle;
  int repropagateCountNeeded;

  bool isOdomInitialized;
  int predictionCounter;

  void initializeParticles();
  void prediction(Eigen::Matrix4f diffPose);
  void weightning(Eigen::Matrix4Xf laser);
  void resampling();
  void showInMap();


public:
  mcl();
  ~mcl();
  void updateData(Eigen::Matrix4f pose, Eigen::Matrix4Xf laser);
};



#endif

