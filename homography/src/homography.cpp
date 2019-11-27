#include "calibration.hpp"
#include "coordinates.hpp"

#include <iostream>
#include <dirent.h>
#include <sys/types.h>
#include <map>


#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

extern "C" {
#include "apriltag.h"
#include "tag36h11.h"
#include "common/getopt.h"
#include "apriltag_pose.h"
}

using namespace std;
using namespace cv;


const string picDir = "./../pic/";
const string picDistDir = "./../pic-dist/";
const string pic1 = picDistDir + "Tag-LR1-LR2-LiveR1.png";
const string pic2 = picDistDir + "Tag-LR1-LR2-LiveR2.png";
const string pic3 = picDistDir + "Tag-XF-Guest-Guest.png";
const string pic4 = picDistDir + "Tag-XF-Guest-XF.jpg";
const string pic5 = picDistDir + "Tag-XF-LR2-LiveR2.png";
const string pic6 = picDistDir + "Tag-XF-LR2-XF.jpeg";
const string pic7 = picDistDir + "Tag-XF-LR3-LiveR3.png";
const string pic8 = picDistDir + "Tag-XF-LR3-XF.jpg";

map<string,int> picmap;
vector<apriltag_detection_t> tagVec;


int undistortPic(std::string picdir, std::string distdir)
{
    DIR *dirptr = nullptr;
    dirptr = opendir(picdir.c_str());
    if (dirptr == nullptr) {
        printf("%s open failed\n",picdir.c_str());
        return -1;
    }

    struct dirent * entry;
    double * camD = nullptr;
    double * distCoeffD = nullptr;

    while ((entry = readdir(dirptr)) != NULL) {

        if(strcmp(entry->d_name,".")==0 || strcmp(entry->d_name,"..")==0)
            continue;

        string pic = picdir + entry->d_name;
        //cout << "pic name is " << pic << endl;

        string camera = pic.substr(pic.rfind("-") + 1);
        camera.erase(camera.rfind("."));
        //cout << "camear name is " << camera << endl;

        switch (picmap.find(camera)->second){
            case 1:
                camD = camD_livR1;
                distCoeffD = distCoeffD_livR1;
                break;
            case 2:
                camD = camD_livR2;
                distCoeffD = distCoeffD_livR2;
                break;
            case 3:
                camD = camD_livR3;
                distCoeffD = distCoeffD_livR3;
                break;
            case 4:
                camD = camD_xf;
                distCoeffD = distCoeffD_xf;
                break;
            case 5:
                camD = camD_guestBedR;
                distCoeffD = distCoeffD_guestBedR;
                break;
            default:
                cout << "not find " << camera << " in " << picdir << endl;
                closedir(dirptr);
                return -1;
                break;
        }

        Mat camera_matrix = Mat(3,3,CV_64FC1,camD);
        Mat distortion_coefficients = Mat(5,1,CV_64FC1,distCoeffD);

        Mat image = imread(pic);
        Mat outImage;
        undistort(image,outImage,camera_matrix,distortion_coefficients);
        imwrite(distdir + entry->d_name,outImage);
        system("sync");
    }

    closedir(dirptr);
    return 0;
}

void coordinateOpt(double * camD , double * distCoeffD, struct tfinfo & TF ,  cv::Mat & homography)
{
    if (tagVec.size() == 0) {
        printf("!!!! not tag !!!!!\n");
        return ;
    }

    Mat imgM , objM;
    vector<Point2f> imgpt;
    imgpt.clear();

    for (auto &item : tagVec) {
        switch (item.id) {
            case 1:
                objM.push_back(ObjTagID1_Mat);
                break;
            case 2:
                objM.push_back(ObjTagID2_Mat);
                break;
            case 3:
                objM.push_back(ObjTagID3_Mat);
                break;
            case 4:
                objM.push_back(ObjTagID4_Mat);
                break;
            default:
                printf("cannot find tag id , ERROR!!!!\n");
                return;
        }

        imgpt.push_back(Point2f(item.p[0][0],item.p[0][1]));
        imgpt.push_back(Point2f(item.p[1][0],item.p[1][1]));
        imgpt.push_back(Point2f(item.p[2][0],item.p[2][1]));
        imgpt.push_back(Point2f(item.p[3][0],item.p[3][1]));
        imgpt.push_back(Point2f(item.c[0],item.c[1]));
    }
    Mat(imgpt).convertTo(imgM,CV_32F);

//    cout << "imgM is " << imgM << endl;
//    cout << "objM is " << objM << endl;

    homography  = cv::findHomography(imgM,objM);
    cout << "camera to world graphy " << homography << endl;


    Mat camera_matrix = Mat(3,3,CV_64FC1,camD);
    Mat distortion_coefficients = Mat(5,1,CV_64FC1,distCoeffD);
    Mat rvec, tvec , rotM , rotT;
    solvePnP(objM, imgM, camera_matrix,distortion_coefficients,rvec, tvec, false);


    Rodrigues(rvec, rotM);  //将旋转向量变换成旋转矩阵

    //根据旋转矩阵求出坐标旋转角
    auto theta_x = atan2(rotM.at<double>(2, 1), rotM.at<double>(2, 2));
    auto theta_y = atan2(-rotM.at<double>(2, 0),
                         sqrt(rotM.at<double>(2, 1)*rotM.at<double>(2, 1) + rotM.at<double>(2, 2)*rotM.at<double>(2, 2)));
    auto theta_z = atan2(rotM.at<double>(1, 0), rotM.at<double>(0, 0));

    TF.x = tvec.at<double>(0,0) * 1e-3;
    TF.y = tvec.at<double>(1,0) * 1e-3;
    TF.z = tvec.at<double>(2,0) * 1e-3;

    TF.roll = theta_x;
    TF.pitch = theta_y;
    TF.yaw = theta_z;

    cout << "x : " << TF.x << endl;
    cout << "y : " << TF.y << endl;
    cout << "z : " << TF.z << endl;
    cout << "roll : " << TF.roll << endl;
    cout << "pitch : " << TF.pitch << endl;
    cout << "yaw : " << TF.yaw << endl;
    cout << "----------------------------------" << endl;

    return;
}

int aprilTag(const string tagPic)
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

    tagVec.clear();
    Mat frame, gray;

    if (access(tagPic.c_str(),F_OK) == 0){
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
        cout << zarray_size(detections) << " tags detected" << endl;

        // Draw detection outlines
        for (int i = 0; i < zarray_size(detections); i++) {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);
            //cout << "tag id " << det->id << endl;

            line(frame, Point(det->p[0][0], det->p[0][1]),
                 Point(det->p[1][0], det->p[1][1]),
                 Scalar(0, 0xff, 0), 2);
            line(frame, Point(det->p[0][0], det->p[0][1]),
                 Point(det->p[3][0], det->p[3][1]),
                 Scalar(0, 0, 0xff), 2);
            line(frame, Point(det->p[1][0], det->p[1][1]),
                 Point(det->p[2][0], det->p[2][1]),
                 Scalar(0xff, 0, 0), 2);
            line(frame, Point(det->p[2][0], det->p[2][1]),
                 Point(det->p[3][0], det->p[3][1]),
                 Scalar(0xff, 0, 0), 2);

            stringstream ss;
            ss << det->id;
            String text = ss.str();
            int fontface = FONT_HERSHEY_SCRIPT_SIMPLEX;
            double fontscale = 1.0;
            int baseline;
            Size textsize = getTextSize(text, fontface, fontscale, 2,
                                        &baseline);
            putText(frame, text, Point(det->c[0]-textsize.width/2,
                                       det->c[1]+textsize.height/2),
                    fontface, fontscale, Scalar(0xff, 0x99, 0), 2);

            tagVec.push_back(*det);
        }

        apriltag_detections_destroy(detections);
/*        namedWindow(tagPic, WINDOW_AUTOSIZE);
        imshow(tagPic, frame);
        waitKey(0);*/
    }

    apriltag_detector_destroy(td);
    tag36h11_destroy(tf);

    return 0;
}


int main (int argc, char **argv)
{
    picmap.insert(pair<string,int>("LiveR1",1));
    picmap.insert(pair<string,int>("LiveR2",2));
    picmap.insert(pair<string,int>("LiveR3",3));
    picmap.insert(pair<string,int>("XF",4));
    picmap.insert(pair<string,int>("Guest",5));

    //去畸变
    {
        string cmd = "rm -rf " + picDistDir + "*";
        system(cmd.c_str());
        int ret = undistortPic(picDir, picDistDir);
        if (ret != 0) {
            cout << endl << "ERROR: pic undistort failed" << endl;
            return -1;
        }
    }


    {
        aprilTag(pic1);
        coordinateOpt(camD_livR1,distCoeffD_livR1,TF_LR1_LR2_LiveR1,homography_LR1_LR2_LiveR1);

        aprilTag(pic2);
        coordinateOpt(camD_livR2,distCoeffD_livR2,TF_LR1_LR2_LiveR2,homography_LR1_LR2_LiveR2);

        aprilTag(pic3);
        coordinateOpt(camD_guestBedR,distCoeffD_guestBedR,TF_XF_Guest_Guest,homography_XF_Guest_Guest);

        aprilTag(pic4);
        coordinateOpt(camD_xf,distCoeffD_xf,TF_XF_Guest_XF,homography_XF_Guest_XF);

        aprilTag(pic5);
        coordinateOpt(camD_livR2,distCoeffD_livR2,TF_XF_LR2_LiveR2,homography_XF_LR2_LiveR2);

        aprilTag(pic6);
        coordinateOpt(camD_xf,distCoeffD_xf,TF_XF_LR2_XF,homography_XF_LR2_XF);

        aprilTag(pic7);
        coordinateOpt(camD_livR3,distCoeffD_livR3,TF_XF_LR3_LiveR3,homography_XF_LR3_LiveR3);

        aprilTag(pic8);
        coordinateOpt(camD_xf,distCoeffD_xf,TF_XF_LR3_XF,homography_XF_LR3_XF);
    }


    return 0;
}