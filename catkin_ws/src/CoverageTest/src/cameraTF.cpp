#include "CoverageTest/tf-coordinates.hpp"


#include "tf/tf.h"
#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "ros/publisher.h"


using  namespace std;

int main (int argc, char **argv)
{
    cout << "camera TF ......" << endl;
    ros::init(argc, argv, "tf_broadcaster");
    ros::NodeHandle node;

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    ros::Rate rate(1);


    while (ros::ok())
   {

       //输入欧拉角，转化成四元数在终端输出
       q.setRPY(TF_LR1_LR2_LiveR1.roll,TF_LR1_LR2_LiveR1.pitch,TF_LR1_LR2_LiveR1.yaw);
       transform.setOrigin(tf::Vector3(TF_LR1_LR2_LiveR1.x,TF_LR1_LR2_LiveR1.y,TF_LR1_LR2_LiveR1.z));
       transform.setRotation(q);
       br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"LiveR1","world"));


       q.setRPY(TF_LR1_LR2_LiveR2.roll,TF_LR1_LR2_LiveR2.pitch,TF_LR1_LR2_LiveR2.yaw);
       transform.setOrigin(tf::Vector3(TF_LR1_LR2_LiveR2.x,TF_LR1_LR2_LiveR2.y,TF_LR1_LR2_LiveR2.z));
       transform.setRotation(q);
       br.sendTransform(tf::StampedTransform(transform.inverse(),ros::Time::now(),"world","liveR2"));


       q.setRPY(TF_XF_LR2_LiveR2.roll,TF_XF_LR2_LiveR2.pitch,TF_XF_LR2_LiveR2.yaw);
       transform.setOrigin(tf::Vector3(TF_XF_LR2_LiveR2.x,TF_XF_LR2_LiveR2.y,TF_XF_LR2_LiveR2.z));
       transform.setRotation(q);
       br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"liveR2","tag-xf-liver2"));


       q.setRPY(TF_XF_LR2_XF.roll,TF_XF_LR2_XF.pitch,TF_XF_LR2_XF.yaw);
       transform.setOrigin(tf::Vector3(TF_XF_LR2_XF.x,TF_XF_LR2_XF.y,TF_XF_LR2_XF.z));
       transform.setRotation(q);
       br.sendTransform(tf::StampedTransform(transform.inverse(),ros::Time::now(),"tag-xf-liver2","XF"));


       q.setRPY(TF_XF_Guest_XF.roll,TF_XF_Guest_XF.pitch,TF_XF_Guest_XF.yaw);
       transform.setOrigin(tf::Vector3(TF_XF_Guest_XF.x,TF_XF_Guest_XF.y,TF_XF_Guest_XF.z));
       transform.setRotation(q);
       br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"XF","tag-xf-guest"));


       q.setRPY(TF_XF_Guest_Guest.roll,TF_XF_Guest_Guest.pitch,TF_XF_Guest_Guest.yaw);
       transform.setOrigin(tf::Vector3(TF_XF_Guest_Guest.x,TF_XF_Guest_Guest.y,TF_XF_Guest_Guest.z));
       transform.setRotation(q);
       br.sendTransform(tf::StampedTransform(transform.inverse(),ros::Time::now(),"tag-xf-guest","Guest"));


       q.setRPY(TF_XF_LR3_XF.roll,TF_XF_LR3_XF.pitch,TF_XF_LR3_XF.yaw);
       transform.setOrigin(tf::Vector3(TF_XF_LR3_XF.x,TF_XF_LR3_XF.y,TF_XF_LR3_XF.z));
       transform.setRotation(q);
       br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"XF","tag-xf-liver3"));


       q.setRPY(TF_XF_LR3_LiveR3.roll,TF_XF_LR3_LiveR3.pitch,TF_XF_LR3_LiveR3.yaw);
       transform.setOrigin(tf::Vector3(TF_XF_LR3_LiveR3.x,TF_XF_LR3_LiveR3.y,TF_XF_LR3_LiveR3.z));
       transform.setRotation(q);
       br.sendTransform(tf::StampedTransform(transform.inverse(),ros::Time::now(),"tag-xf-liver3","LiveR3"));


       ros::spinOnce();
       rate.sleep();
   }

    return 0;
}