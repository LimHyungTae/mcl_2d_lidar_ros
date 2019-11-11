
#include "mapgen.h"

mapgen::mapgen()
{
  gridMap = cv::Mat(300,300,CV_32FC1, cv::Scalar(0.5));

  imageResolution = 0.05; // [m] per [pixel]
  tf_laser2robot << -1, 0, 0,-0.1,
                     0, 1, 0,   0,
                     0, 0,-1,   0,
                     0, 0, 0,   1;
  mapCenterX = 0;
  mapCenterY = -2;
  occupancyIncrease = 0.02;
  occupancyDecrease = 0.01;
}

mapgen::~mapgen()
{

}

void mapgen::updateMap(Eigen::Matrix4f pose, Eigen::Matrix4Xf laser)
{
  //Todo : Convert robot pose in image frame
  //Input :  pose(0,3), pose(1,3) (x,y position in [m])
  //         imageResolution
  //         gridMap.rows , gridMap.cols (size of image)
  //         mapCenterX, mapCenterY (center of map's position)
  //Output : poseX, poseY (pose in pixel value)

  int poseX;
  int poseY;
  poseX = static_cast<int>((pose(0,3) - mapCenterX + (300.0*imageResolution)/2)/imageResolution);
  poseY = static_cast<int>((pose(1,3) - mapCenterY + (300.0*imageResolution)/2)/imageResolution);
//  poseY = static_cast<int>(mapCenterY + (300.0*imageResolution)/2- (pose(1,3))/imageResolution);;

  //---------------------------------------------//

  //Generate new image for visualize (do not touch)
  cv::Mat showMap;
  cv::cvtColor(gridMap,showMap,CV_GRAY2RGB);
  cv::circle(showMap,cv::Point(poseX,poseY),2,cv::Scalar(0,0,255),-1);

  Eigen::Matrix4Xf laserTransformed;
  //Todo : Transform laser data into global frame
  //Input : laser (4 x N matrix of laser points in lidar sensor's frame)
  //        pose (4 x 4 matrix of robot pose)
  //        tf_laser2robot (4 x 4 matrix of transformatino between robot and sensor)
  //Output : laserTransformed (4 x N matrix of laser points in global frame)

  laser = pose * tf_laser2robot* laser;
  laserTransformed = laser; // 'laser' is in lidar sensor's frame.

  //--------------------------------------------//

  //Put laser into Gridmap
  for(int i=0;i<laserTransformed.cols();i++)
  {
    //TODO :  translate each laser point (in [m]) to pixel frame.  (transLaser(0,i) 's unit is [m]) (You will use it in MCL too! remember!)
    //Input : laserTransformed(0,i), laserTransformed(1,i)  (laser point's pose in global frame)
    //         imageResolution
    //         gridMap.rows , gridMap.cols (size of image)
    //         mapCenterX, mapCenterY (center of map's position)
    //Output : laserX, laserY (laser point's pixel position)

    int laserX;
    int laserY;
    laserX = static_cast<int>((laserTransformed(0, i) - mapCenterX + (300.0*imageResolution)/2)/imageResolution);
    laserY = static_cast<int>((laserTransformed(1, i) - mapCenterY + (300.0*imageResolution)/2)/imageResolution);
//    laserY = static_cast<int>(mapCenterY + (300.0*imageResolution)/2- (laserTransformed(1, i))/imageResolution);

    //---------------------------------------------------------------------//

    if(laserX>=0 && laserX<gridMap.cols && laserY>=0 && laserY<gridMap.rows)
    {
       cv::circle(showMap,cv::Point(laserX,laserY),1,cv::Scalar(0,255,255),-1);
       cv::LineIterator it(gridMap, cv::Point(poseX,poseY), cv::Point(laserX,laserY), 8);

       for(int j=0;j<it.count-1;j++) // For all point on laser ray.
       {
         //Todo : Set occupancy of points on laser ray (Using value 'occupancyDecrease')

         gridMap.at<float>(it.pos()) = gridMap.at<float>(it.pos()) +0.02 ; // 0.5 is neural value. how can we make it whiter for free cell?

         //-----------------------------------------//
         it++;
       }

       //Todo : Set occupancy of laser end-point. (Using value 'occupancyIncrease')

       gridMap.at<float>(cv::Point(laserX,laserY)) = gridMap.at<float>(cv::Point(laserX, laserY)) - 0.01; // 0.5 is neural value. how can we make it darker for occupied cell?

       //------------------------------------------//
    }
  }
//  cv::resize(showMap, showMap, cv::Size(), 1, 1);
  cv::imshow("CURRENT",showMap);
  cv::imshow("MAP",gridMap);
//  cv::Mat image_new = gridMap.clone();
//  image_new.convertTo(image_new, CV_8UC3, 255.0);
//  cv::imwrite("map.png", image_new);
  cv::waitKey(1);
}
