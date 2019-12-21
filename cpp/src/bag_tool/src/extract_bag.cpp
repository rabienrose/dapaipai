#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <cstdio>
#include <iostream>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <map>
#include <opencv2/opencv.hpp>
#include <memory>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <bag_tool/extract_bag.h>
#include <Eigen/Core>
#include <gflags/gflags.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <chrono>
#include <ctime> 
#include "stitch_algo.h"

void extract_bag(std::string out_addr_, std::string bag_addr_){
    std::string bag_addr=bag_addr_;
    std::string out_dir=out_addr_;
    std::string img_dir=out_dir+"/images";
    mkdir(img_dir.c_str(), S_IRWXO|S_IRWXG|S_IRWXU);
    rosbag::Bag bag;
    bag.open(bag_addr,rosbag::bagmode::Read);
    std::vector<std::string> topics;
    topics.push_back("img");
    topics.push_back("dir");
    topics.push_back("rot");
    topics.push_back("acc");
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    rosbag::View::iterator it= view.begin();
    StitchAlgo algo;
    algo.is_paint=true;
    algo.ClearImgSphere();
    int cur_motion_id=-1;
    Eigen::Vector3d cur_rot;
    Eigen::Vector3d cur_acc;
    Eigen::Quaterniond cur_dir;
    bool get_rot=false;
    bool get_acc=false;
    bool get_dir=false;
    bool get_all_motion_data=false;
    int img_count=0;
    for(;it!=view.end();it++){
        rosbag::MessageInstance m =*it;
        sensor_msgs::CompressedImagePtr simg = m.instantiate<sensor_msgs::CompressedImage>();
        if( simg!=NULL ){
            cv_bridge::CvImagePtr cv_ptr;
            try{
                cv_ptr = cv_bridge::toCvCopy(simg, "bgra8");
                //if(img_count==215 || img_count==0 || img_count==1){
                    algo.AddImage(cv_ptr->image, simg->header.stamp.toSec());
                    cv::imshow("chamo", cv_ptr->image);
                    cv::waitKey(-1);
                //}
                img_count++;
                
            }catch (cv_bridge::Exception& e){
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }
        }
        geometry_msgs::QuaternionStampedPtr sdir = m.instantiate<geometry_msgs::QuaternionStamped>();
        if( sdir!=NULL ){
            Eigen::Quaterniond qua;
            qua.x()=sdir->quaternion.x;
            qua.y()=sdir->quaternion.y;
            qua.z()=sdir->quaternion.z;
            qua.w()=sdir->quaternion.w;
            if(get_rot && get_acc){
                algo.AddRot(qua, cur_rot, cur_acc, sdir->header.stamp.toSec());
            }
        }
        geometry_msgs::Vector3StampedPtr srot = m.instantiate<geometry_msgs::Vector3Stamped>();
        if( srot!=NULL && m.getTopic()=="rot"){
            cur_rot.x()=srot->vector.x;
            cur_rot.y()=srot->vector.y;
            cur_rot.z()=srot->vector.z;
            get_rot=true;
        }
        geometry_msgs::Vector3StampedPtr sacc = m.instantiate<geometry_msgs::Vector3Stamped>();
        if( sacc!=NULL && m.getTopic()=="acc"){
            cur_acc.x()=sacc->vector.x;
            cur_acc.y()=sacc->vector.y;
            cur_acc.z()=sacc->vector.z;
            get_acc=true;
        }
    }
};
