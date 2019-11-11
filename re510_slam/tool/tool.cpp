#include "tool.h"

namespace tool
{

cv::Mat cvMaptoMCLMap(cv::Mat mat)
{
  for(int i=0;i<mat.cols;i++)
  {
    for(int j=0;j<mat.rows;j++)
    {
      unsigned char val = mat.at<unsigned char>(j,i);
      if(val>100 && val<150) mat.at<unsigned char>(j,i)=255;
//      else if(val<128) mat.at<unsigned char>(j,i)=255-val;
//      else if(val>128) mat.at<unsigned char>(j,i)=255-val;
    }
  }
  return mat;
}


Eigen::Matrix4f mat2eigen(cv::Mat mat)
{
  Eigen::Matrix4f result =  Eigen::Matrix4f::Identity();

  result(0,0) = mat.at<float>(0,0);
  result(0,1) = mat.at<float>(0,1);
  result(0,2) = mat.at<float>(0,2);
  result(0,3) = mat.at<float>(0,3);

  result(1,0) = mat.at<float>(1,0);
  result(1,1) = mat.at<float>(1,1);
  result(1,2) = mat.at<float>(1,2);
  result(1,3) = mat.at<float>(1,3);

  result(2,0) = mat.at<float>(2,0);
  result(2,1) = mat.at<float>(2,1);
  result(2,2) = mat.at<float>(2,2);
  result(2,3) = mat.at<float>(2,3);

  result(3,0) = mat.at<float>(3,0);
  result(3,1) = mat.at<float>(3,1);
  result(3,2) = mat.at<float>(3,2);
  result(3,3) = mat.at<float>(3,3);

  return result;
}

cv::Mat eigen2mat(Eigen::Matrix4f mat)
{
  cv::Mat result = cv::Mat::zeros(4,4,CV_32FC1);
  result.at<float>(0,0) = mat(0,0);
  result.at<float>(0,1) = mat(0,1);
  result.at<float>(0,2) = mat(0,2);
  result.at<float>(0,3) = mat(0,3);

  result.at<float>(1,0) = mat(1,0);
  result.at<float>(1,1) = mat(1,1);
  result.at<float>(1,2) = mat(1,2);
  result.at<float>(1,3) = mat(1,3);

  result.at<float>(2,0) = mat(2,0);
  result.at<float>(2,1) = mat(2,1);
  result.at<float>(2,2) = mat(2,2);
  result.at<float>(2,3) = mat(2,3);

  result.at<float>(3,0) = mat(3,0);
  result.at<float>(3,1) = mat(3,1);
  result.at<float>(3,2) = mat(3,2);
  result.at<float>(3,3) = mat(3,3);

  return result;
}

cv::Mat xyzrpy2mat(float x, float y, float z, float roll, float pitch, float yaw)
{
  cv::Mat rot_vec = cv::Mat::zeros(3,1,CV_32FC1);
  rot_vec.at<float>(0) = roll;
  rot_vec.at<float>(1) = pitch;
  rot_vec.at<float>(2) = yaw;

  cv::Mat rot_mat;
  cv::Rodrigues(rot_vec,rot_mat);

  cv::Mat result = cv::Mat::zeros(4,4,CV_32FC1);

  rot_mat.copyTo(result(cv::Rect(0,0,3,3)));

  result.at<float>(0,3) = x;
  result.at<float>(1,3) = y;
  result.at<float>(2,3) = z;

  result.at<float>(3,3) = 1;

  return result;
}

void mat2xyzrpy(cv::Mat mat, float *x, float *y, float *z, float *roll, float *pitch, float *yaw)
{
  *x = mat.at<float>(0,3);
  *y = mat.at<float>(1,3);
  *z = mat.at<float>(2,3);

  cv::Mat rot_mat = cv::Mat(mat(cv::Rect(0,0,3,3)));

  cv::Mat rot_vec;
  cv::Rodrigues(rot_mat,rot_vec);
  *roll = rot_vec.at<float>(0);
  *pitch = rot_vec.at<float>(1);
  *yaw = rot_vec.at<float>(2);
}




Eigen::VectorXf eigen2xyzrpy(Eigen::Matrix4f mat)
{
  Eigen::VectorXf result(6);
  mat2xyzrpy(eigen2mat(mat), &result[0], &result[1], &result[2], &result[3], &result[4], &result[5]);
  return result;
}


Eigen::Matrix4f xyzrpy2eigen(float x, float y, float z, float roll, float pitch, float yaw)
{
  Eigen::Matrix4f result =  mat2eigen(xyzrpy2mat(x,y,z,roll,pitch,yaw));
  return result;
}

double GaussianRand()
{
  double u = ((double) rand() / (RAND_MAX)) * 2 - 1;
  double v = ((double) rand() / (RAND_MAX)) * 2 - 1;
  double r = u * u + v * v;
  if (r == 0 || r > 1) return GaussianRand();
  double c = sqrt(-2 * log(r) / r);
  return u * c;
}

}
