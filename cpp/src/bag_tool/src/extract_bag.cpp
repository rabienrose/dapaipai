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
    rosbag::Bag bag;
    bag.open(bag_addr,rosbag::bagmode::Read);
    std::vector<std::string> topics;
    topics.push_back("img");
    topics.push_back("dir");
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    rosbag::View::iterator it= view.begin();
    StitchAlgo algo;
    algo.ClearImgSphere();
    Eigen::Quaterniond cur_dir;
    std::map<int, cv::Mat> img_pool;
    std::map<int, Eigen::Quaterniond> dir_pool;
    int img_count=0;
    for(;it!=view.end();it++){
        rosbag::MessageInstance m =*it;
        sensor_msgs::CompressedImagePtr simg = m.instantiate<sensor_msgs::CompressedImage>();
        if( simg!=NULL ){
            cv::Mat image = cv::imdecode(cv::Mat(simg->data),cv::IMREAD_UNCHANGED );
            img_pool[simg->header.seq]=image;
        }
        geometry_msgs::QuaternionStampedPtr sdir = m.instantiate<geometry_msgs::QuaternionStamped>();
        if( sdir!=NULL ){
            Eigen::Quaterniond qua;
            qua.x()=sdir->quaternion.x;
            qua.y()=sdir->quaternion.y;
            qua.z()=sdir->quaternion.z;
            qua.w()=sdir->quaternion.w;
            dir_pool[sdir->header.seq]=qua;
        }
    }
    for(int i=0; i<100; i++){
        std::map<int, Eigen::Quaterniond>::iterator it_dir=dir_pool.find(i);
        std::map<int, cv::Mat>::iterator it_img=img_pool.find(i);
        if(it_dir!=dir_pool.end() && it_img!=img_pool.end()){
            Eigen::Matrix3d dir_m(it_dir->second);
            algo.AddData(it_img->second, dir_m);
        }
    }
    cv::Mat re_img = algo.FinalImg();
    if(re_img.cols>0){
        cv::flip(re_img, re_img, 0);
        cv::imwrite(out_dir, re_img);
    }
    
};
