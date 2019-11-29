
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

#include <iostream>
#include <stdio.h>
#include <fstream>
#include <sys/stat.h>
#include <unistd.h>

using namespace cv;
using namespace std;


struct timespec file_create_time(std::string video_path)
{
    struct stat buf;
    stat(video_path.c_str(),&buf);

    cout << "file create time is  " << buf.st_ctim.tv_sec << "s " << endl;
    return buf.st_ctim;
}

void video2Pic(std::string video_path,std::string pic_save_path, std::string camera_id)
{
    if (access(pic_save_path.c_str(),F_OK) != 0) {
        cout << pic_save_path << " not exist !" << endl;
        mkdir(pic_save_path.c_str(),0777);
        system("sync");
        cout << "create it at first!!" << endl;
    } else {
        //std::string cmd = "rm -rf " + pic_save_path + "/*";
        //system(cmd.c_str());
        //system("sync");
    }

    std::string video_startTime = video_path.substr(video_path.rfind('/') + 1,10);
    long int mp4_begin_time =  stol(video_startTime);
    cout << "video start time " << mp4_begin_time << endl;


    cv::VideoCapture cap(video_path,CAP_ANY);
    // check if we succeeded
    if (!cap.isOpened()) {
        cerr << "ERROR! Unable to open video\n";
        return;
    }

    int  fps = 30 ; // cap.get(cv::CAP_PROP_FPS);
    cout << "video fps is " << fps << endl;

    auto width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    cout << "video frame width is " << width << endl;

    auto height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    cout << "video frame height is " << height << endl;


    int frame = 0;
    cv::Mat src;
    while (1) {
        // check if we succeeded
        if (!cap.read(src)) {
            //cerr << "ERROR! blank frame grabbed\n";
            break;
        }

        if (frame % fps == 0) {

            char pic_name[56] = {0};
            long int timet = (long int)(1.0 / fps * frame) + mp4_begin_time;
            snprintf(pic_name, sizeof(pic_name), "%ld-%s.png",timet,camera_id.c_str());

            //cout << "save pic name is " << pic_name << endl;
            bool result = cv::imwrite(pic_save_path + "/" + std::string(pic_name),src);
            if (!result) {
                cout << "write failed!!" << endl;
                break;
            }
        }
        frame ++;
    }
    cap.release();
    return;
}

int main(int argc, char** argv)
{
    //argv[1]: mp4 file
    //argv[2]: picture save path
    //argv[3]: camera ID

    if (argc != 4) {
        cout << "usage: ./video_analyze demo.mp4 ./pic/  1" << endl;
        return -1;
    }

    video2Pic(argv[1],argv[2],argv[3]);
    system("sync");

    return 0;
}
