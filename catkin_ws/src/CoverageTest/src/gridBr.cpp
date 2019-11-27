#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"

#include <opencv2/core.hpp>
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"

using namespace cv;
using namespace std;

int main (int argc, char **argv)
{
    ros::init(argc, argv, "gridMap");

    ros::NodeHandle nh;


    ros::Publisher pub = nh.advertise<nav_msgs::OccupancyGrid>("/gridMap", 1);
    nav_msgs::OccupancyGrid map;

    Mat frame, gray;
    frame = cv::imread("/home/tfn/Desktop/ihos2.jpg");
    cvtColor(frame, gray, COLOR_BGR2GRAY);


    map.header.frame_id="world";
    map.header.stamp = ros::Time::now();
    map.info.resolution = 0.01372;         // float32
    map.info.width      = gray.cols;           // uint32
    map.info.height     = gray.rows;           // uint32

    map.info.origin.position.x = -5.15;
    map.info.origin.position.y = -2.178;

    map.info.origin.orientation.x = 0.5;
    map.info.origin.orientation.y = 0.5;

    cout << "width: " << gray.cols << endl;
    cout << "height: " << gray.rows << endl;


    int p[map.info.width*map.info.height] = {-1};   // [0,100]
    for (int i = 0;  i < gray.cols; i++) {
        for (int j = 0; j < gray.rows; j++) {
            p[i * map.info.width + j] = gray.at<int8_t>(i,j);
        }
    }

    std::vector<signed char> a(p, p+ map.info.width*map.info.height);
    map.data = a;

    while (ros::ok())
    {
        pub.publish(map);
    }

    ros::shutdown();
    return 0;
}
