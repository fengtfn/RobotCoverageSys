#include <iostream>
#include <dirent.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "ros/ros.h"
#include "ros/publisher.h"
#include "nav_msgs/Path.h"

#include "CoverageTest/tf-coordinates.hpp"

#include <opencv2/core.hpp>
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

extern "C" {
#include "apriltag.h"
#include "tag36h11.h"
#include "tag25h9.h"
#include "tag16h5.h"
#include "tagCircle21h7.h"
#include "tagCircle49h12.h"
#include "tagCustom48h12.h"
#include "tagStandard41h12.h"
#include "tagStandard52h13.h"
#include "common/getopt.h"
#include "apriltag_pose.h"
}

using namespace cv;
using namespace std;

struct picinfo{
    std::string stamp;
    std::string cameraID;
    Point2f coord;
};

const std::string video2pic = "/home/tfn/git-project/RobotCoverageSys/catkin_ws/src/CoverageTest/pic2/";

vector<struct  picinfo> imgpt;
vector<Point2f> objpt;
vector<Point2f> imgpt2;

vector<string> vecFile;


int aprilTag(const char * tagPic, string stamp, string cameraID)
{
    // Initialize tag detector with options
    apriltag_family_t *tf = NULL;

    tf = tag36h11_create();

    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);
    td->quad_decimate = 2.0;
    td->quad_sigma = 0.0;
    td->nthreads = 1;
    td->debug = 0;
    td->refine_edges = 1;

    Mat frame, gray;

    if (access(tagPic,F_OK) == 0){
        //读取原始图像
        frame = imread(tagPic, IMREAD_UNCHANGED);
        if (frame.empty()) {
            //检查是否读取图像
            cout << "Error! Input image cannot be read...\n";
            return -1;
        }

        cvtColor(frame, gray, COLOR_BGR2GRAY);

        // Make an image_u8_t header for the Mat data
        image_u8_t im = { .width = gray.cols,
                .height = gray.rows,
                .stride = gray.cols,
                .buf = gray.data
        };

        zarray_t *detections = apriltag_detector_detect(td, &im);
        //cout << zarray_size(detections) << " tags detected" << endl;
        if (zarray_size(detections) == 1) {
            // Draw detection outlines
            for (int i = 0; i < zarray_size(detections); i++) {
                apriltag_detection_t *det;
                zarray_get(detections, i, &det);
                //cout << "tag id " << det->id << endl;

                imgpt2.push_back(Point2f(det->c[0],det->c[1]));

                if (det->id == 0) {
                    struct picinfo tmp;
                    tmp.stamp = stamp;
                    tmp.cameraID = cameraID;
                    tmp.coord = Point2f(det->c[0],det->c[1]);
                    imgpt.push_back(tmp);
                }
            }
        } else {
            string cmd = "rm -f " + string(tagPic);
            system(cmd.c_str());
            system("sync");
        }

        apriltag_detections_destroy(detections);
    }

    apriltag_detector_destroy(td);
    tag36h11_destroy(tf);

    return 0;
}

//列出一个目录下所有文件
void scan_one_dir( const char * dir_name)
{

    struct dirent **namelist;
    int  n = scandir(dir_name,&namelist,0,alphasort);

    int index = 0;
    while (index < n) {
        if (strcmp (namelist[index]->d_name, ".") == 0 ||
            strcmp (namelist[index]->d_name, "..") == 0) {
            free(namelist[index]);
            index ++;
            continue;
        }

        char wholePath[256] = {0};
        sprintf(wholePath, "%s%s", dir_name,  namelist[index]->d_name);
        string str(namelist[index]->d_name);

        aprilTag(wholePath,str.substr(0,10),str.substr(str.find("_") + 1, str.rfind(".") - (str.find("_") + 1)) );
        free(namelist[index]);
        index++;
    }
    free(namelist);
}

bool myfunc(struct picinfo p1,  struct picinfo p2)
{
    return p1.stamp == p2.stamp;
}

int main (int argc, char **argv)
{
    scan_one_dir(video2pic.c_str());

    auto pos = unique(imgpt.begin(),imgpt.end(),myfunc);
    imgpt.erase(pos,imgpt.end());

    vector<Point2f> vec;
    vector<Point2f> vecOut;
    for (auto & item : imgpt) {

        vec.push_back(item.coord);


        if (item.cameraID == "xf") {
            cv::perspectiveTransform(Mat(vec),vecOut,homography_XF_LR2_XF);
        }
        if (item.cameraID == "liv3") {
            Mat tag2xf;
            invert(homography_XF_LR3_XF,tag2xf);
            cv::perspectiveTransform(Mat(vec),vecOut,homography_XF_LR2_XF*tag2xf*homography_XF_LR3_LiveR3);
        }

        objpt.push_back(vecOut.at(0));
        vec.clear();
        vecOut.clear();
    }


    ros::init(argc, argv, "path_broadcaster");
    ros::NodeHandle node;

    ros::Publisher pub;
    pub = node.advertise<nav_msgs::Path>("/mypath",100);

    ros::Rate rate(1);
    while (ros::ok()) {
        {
            auto path_array = nav_msgs::Path();
            path_array.header.frame_id = "world";
            path_array.header.stamp = ros::Time::now();
            for (auto item = objpt.begin(); item != objpt.end(); item ++) {
                geometry_msgs::PoseStamped p;
                p.header.frame_id = "world";
                p.header.stamp = ros::Time::now();
                p.pose.position.x = item.base()->x * 1e-3;
                p.pose.position.y = item.base()->y * 1e-3;
                p.pose.orientation.w = 1;
                path_array.poses.push_back(p);
            }
            pub.publish(path_array);
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
