#ifndef COORDINATES_HPP
#define COORDINATES_HPP

#include <opencv2/core.hpp>

//{corner0,corner1,corner2,corner3,center}
float ObjTagID1[5][3] = {{-244,170,0},{-112,170,0},{-112,302,0},{-244,302,0},{-178,236,0}};
cv::Mat ObjTagID1_Mat(5,3,CV_32F,ObjTagID1);

float ObjTagID2[5][3] = {{112,170,0},{244,170,0},{244,302,0},{112,302,0},{178,236,0}};
cv::Mat ObjTagID2_Mat{5,3,CV_32F,ObjTagID2};

float ObjTagID3[5][3] = {{-244,-302,0},{-112,-302,0},{-112,-170,0},{-244,-170,0},{-178,-236,0}};
cv::Mat ObjTagID3_Mat{5,3,CV_32F,ObjTagID3};

float ObjTagID4[5][3] = {{112,-302,0},{244,-302,0},{244,-170,0},{112,-170,0},{178,-236,0}};
cv::Mat ObjTagID4_Mat{5,3,CV_32F,ObjTagID4};

struct  tfinfo {
    float  x;
    float  y;
    float  z;
    float  roll;
    float  pitch;
    float  yaw;
};

struct tfinfo TF_LR1_LR2_LiveR1;
struct tfinfo TF_LR1_LR2_LiveR2;
struct tfinfo TF_XF_Guest_Guest;
struct tfinfo TF_XF_Guest_XF;
struct tfinfo TF_XF_LR2_LiveR2;
struct tfinfo TF_XF_LR2_XF;
struct tfinfo TF_XF_LR3_LiveR3;
struct tfinfo TF_XF_LR3_XF;


cv::Mat homography_LR1_LR2_LiveR1;
cv::Mat homography_LR1_LR2_LiveR2;
cv::Mat homography_XF_Guest_Guest;
cv::Mat homography_XF_Guest_XF;
cv::Mat homography_XF_LR2_LiveR2;
cv::Mat homography_XF_LR2_XF;
cv::Mat homography_XF_LR3_LiveR3;
cv::Mat homography_XF_LR3_XF;

#endif