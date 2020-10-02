#include <string>
#include <fstream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <math.h>
#include "imu_tools.h"
#include "NavState.h"
#include "IMUConverter.h"

namespace chamo {

std::vector<std::string> split(const std::string& str, const std::string& delim){
    std::vector<std::string> tokens;
    size_t prev = 0, pos = 0;
    do
    {
        pos = str.find(delim, prev);
        if (pos == std::string::npos) pos = str.length();
        std::string token = str.substr(prev, pos-prev);
        if (!token.empty()) tokens.push_back(token);
        prev = pos + delim.length();
    }
    while (pos < str.length() && prev < str.length());
    return tokens;
}

}

Eigen::Vector3d interEigenV(Eigen::Vector3d v1, Eigen::Vector3d v2, double t1, double t2, double t3){
    return v1 + (v2 - v1) * (t3 - t1) / (t2 - t1);
}

void updatePreInt(std::vector<chamo::IMUPreintegrator>& preints, std::vector<std::vector<chamo::IMUData>>& sycn_imu_datas, Eigen::Vector3d ba, Eigen::Vector3d bg){
    for(int i=0; i<sycn_imu_datas.size(); i++){
        chamo::IMUPreintegrator temp_preint;
        temp_preint.reset();
        for(int j=1; j<sycn_imu_datas[i].size(); j++){
            double dt=sycn_imu_datas[i][j]._t-sycn_imu_datas[i][j-1]._t;
            temp_preint.update(sycn_imu_datas[i][j]._g - bg, sycn_imu_datas[i][j]._a - ba, dt);
        }
        preints.push_back(temp_preint);
    }
}

void alignToIMU(std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Vector4d>>& frame_poses, std::vector<Eigen::Matrix<double, 7, 1>>& imu_datas, std::vector<double> frame_times, Eigen::Matrix4d Tbc){
    std::vector<std::vector<chamo::IMUData>> sycn_imu_datas;
    std::vector<cv::Mat> pose_vec_mat;
    for(int i=0; i<frame_poses.size(); i++){
        pose_vec_mat.push_back(chamo::Converter::toCvMat(frame_poses[i]));
        std::vector<chamo::IMUData> imudatas;
        sycn_imu_datas.push_back(imudatas);
    }
    int cur_imu_ind=1;
    for(int i=1; i<frame_poses.size()-1; i++){
        while(true){
            if (cur_imu_ind>=imu_datas.size()){
                break;
            }
            Eigen::Vector3d g;
            Eigen::Vector3d a;
            g(0)=imu_datas[cur_imu_ind](1);
            g(1)=imu_datas[cur_imu_ind](2);
            g(2)=imu_datas[cur_imu_ind](3);
            a(0)=imu_datas[cur_imu_ind](4);
            a(1)=imu_datas[cur_imu_ind](5);
            a(2)=imu_datas[cur_imu_ind](6);
            double cur_t=imu_datas[cur_imu_ind](0);
            double last_time=imu_datas[cur_imu_ind-1](0);
            double img_time=frame_times[i];
            if (cur_t>=img_time && last_time<img_time){
                Eigen::Vector3d last_g;
                Eigen::Vector3d last_a;
                last_g(0)=imu_datas[cur_imu_ind-1](1);
                last_g(1)=imu_datas[cur_imu_ind-1](2);
                last_g(2)=imu_datas[cur_imu_ind-1](3);
                last_a(0)=imu_datas[cur_imu_ind-1](4);
                last_a(1)=imu_datas[cur_imu_ind-1](5);
                last_a(2)=imu_datas[cur_imu_ind-1](6);
                chamo::IMUData imu_data;
                imu_data._g = interEigenV(last_g, g, last_time, cur_t, img_time);
                imu_data._a = interEigenV(last_a, a, last_time, cur_t, img_time);
                imu_data._t = img_time;
                sycn_imu_datas[i].push_back(imu_data);
//                std::cout<<"=================="<<std::endl;
//                std::cout<<frame_times[i]<<std::endl;
//                std::cout<<"=================="<<std::endl;
                break;
            }
            if (cur_t<=img_time){
                
                chamo::IMUData imu_data;
                imu_data._g=g;
                imu_data._a=a;
                imu_data._t=imu_datas[cur_imu_ind](0);
                sycn_imu_datas[i].push_back(imu_data);
//                std::cout<<imu_datas[cur_imu_ind](0)<<std::endl;
                cur_imu_ind++;
            }else{
                break;
            }
            
            
        }
    }
    Eigen::Vector3d bg=Eigen::Vector3d::Zero();
    Eigen::Vector3d ba=Eigen::Vector3d::Zero();
    std::vector<chamo::IMUPreintegrator> preints;
    updatePreInt(preints, sycn_imu_datas, ba, bg);
    Eigen::Vector3d new_bg = OptimizeInitialGyroBias(pose_vec_mat, preints, Tbc);
    std::cout<<new_bg<<std::endl;
    bg=new_bg;
    preints.clear();
    updatePreInt(preints, sycn_imu_datas, ba, bg);
    double sstar;
    cv::Mat gwstar;
    double scale_confi;
    double grav_confi;
    CalGravityAndScale(pose_vec_mat, preints, chamo::Converter::toCvMat(Tbc), sstar, gwstar, scale_confi, grav_confi);
    std::cout<<"sstar: "<<sstar<<std::endl;
    std::cout<<"gwstar: "<<gwstar<<std::endl;
    cv::Mat gI = cv::Mat::zeros(3,1,CV_32F);
    gI.at<float>(2) = 1;
    gwstar=-gwstar;
    cv::Mat gwn = gwstar/cv::norm(gwstar);
    cv::Mat gIxgwn = gI.cross(gwn);
    double normgIxgwn = cv::norm(gIxgwn);
    cv::Mat vhat = gIxgwn/normgIxgwn;
    double theta = std::atan2(normgIxgwn,gI.dot(gwn));
    Eigen::Vector3d vhateig = chamo::Converter::toVector3d(vhat);
    Eigen::Matrix3d Rwi_ = Sophus::SO3::exp(vhateig*theta).matrix();
    Eigen::Vector3d bias_a=Eigen::Vector3d::Zero();
//    CalAccBias(pose_vec_mat, preints, sstar, gwstar, chamo::Converter::toCvMat(Tbc), Rwi_, bias_a);
//    std::cout<<"reafined sstar: "<<sstar<<std::endl;
}

void read_imu_data(std::string imu_addr, std::vector<Eigen::Matrix<double, 7, 1>>& imu_datas){
    std::string line;
    std::ifstream infile_imu(imu_addr.c_str());
    int imu_count=0;
    
    while (true)
    {
        std::getline(infile_imu, line);
        if (line==""){
            break;
        }
        std::vector<std::string> splited = chamo::split(line, ",");
        Eigen::Matrix<double, 7, 1> imu;
        int tmp_int = atoi(splited[0].c_str());
        imu(0)=tmp_int/1000.0;
        imu(1)=atof(splited[1].c_str());
        imu(2)=atof(splited[2].c_str());
        imu(3)=atof(splited[3].c_str());
        imu(4)=atof(splited[4].c_str())*9.8;
        imu(5)=atof(splited[5].c_str())*9.8;
        imu(6)=atof(splited[6].c_str())*9.8;
        imu_datas.push_back(imu);
    }
}

void read_gps_data(std::string gps_addr, std::vector<Eigen::Matrix<double, 4, 1>>& gps_datas){
    std::string line;
    std::ifstream infile_imu(gps_addr.c_str());
    int gps_count=0;
    int last_gps_time=0;
    int first_gps_time=0;
    while (true)
    {
        std::getline(infile_imu, line);
        if (line==""){
            break;
        }
        std::vector<std::string> splited = chamo::split(line, ",");
        int tmp_int = atoi(splited[0].c_str());
        double lat= atof(splited[1].c_str());
        double lon= atof(splited[2].c_str());
        double h= atof(splited[3].c_str());
        if (gps_count==0){
            last_gps_time=tmp_int;
            first_gps_time=tmp_int;
        }else{
            if (last_gps_time==tmp_int){
                continue;
            }
            last_gps_time=tmp_int;
        }
        Eigen::Matrix<double,4, 1> gps;
        gps(0)=(tmp_int-first_gps_time)/1000.0;
        gps(0)=lat;
        gps(0)=lon;
        gps(0)=h;
        gps_count++;
        gps_datas.push_back(gps);
    }
}

void read_cam_info(std::string cam_addr, Eigen::Matrix4d& Tbc){
    std::string line;
    std::ifstream infile_camera(cam_addr.c_str()); 
    std::getline(infile_camera, line);
    std::vector<std::string> splited = chamo::split(line, " ");
    Tbc(0,0)=atof(splited[0].c_str());
    Tbc(0,1)=atof(splited[1].c_str());
    Tbc(0,2)=atof(splited[2].c_str());
    Tbc(0,3)=atof(splited[3].c_str());
    Tbc(1,0)=atof(splited[4].c_str());
    Tbc(1,1)=atof(splited[5].c_str());
    Tbc(1,2)=atof(splited[6].c_str());
    Tbc(1,3)=atof(splited[7].c_str());
    Tbc(2,0)=atof(splited[8].c_str());
    Tbc(2,1)=atof(splited[9].c_str());
    Tbc(2,2)=atof(splited[10].c_str());
    Tbc(2,3)=atof(splited[11].c_str());
//    Eigen::Matrix4d T_rot=Eigen::Matrix4d::Zero();
//    T_rot(0,2)=-1;
//    T_rot(1,1)=1;
//    T_rot(2,0)=1;
//    T_rot(3,3)=1;
//    Tbc=T_rot*Tbc;
}

double read_img_start_time(std::string img_start_file){
    std::string line;
    std::ifstream infile(img_start_file);
    std::getline(infile, line);
    int tmp_int = atoi(line.c_str());
    return tmp_int/1000.0;
}

void read_img_time(std::string img_time_addr, std::vector<double>& img_timess){
    std::string line;
    std::ifstream infile(img_time_addr);
    while (true)
    {
        std::getline(infile, line);
        std::vector<std::string> splited = chamo::split(line, ",");
        if (line==""){
            break;
        }
        int tmp_int = atoi(splited[0].c_str());
        img_timess.push_back(tmp_int/1000.0);
    }
    infile.close();
}

void read_frame_pose(std::string img_pose_addr, std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Vector4d>>& poses, std::vector<int>& frame_ids){
    std::string line;
    std::ifstream infile_camera(img_pose_addr.c_str());
    int step=20;
    int tmp_count=0;
    while (true){
        std::getline(infile_camera, line);
        if (line==""){
            break;
        }
        if (tmp_count%step!=0){
            tmp_count++;
            continue;
        }
        std::vector<std::string> splited = chamo::split(line, " ");
        frame_ids.push_back(atoi(splited[0].c_str()));
        Eigen::Matrix4d pose=Eigen::Matrix4d::Identity();
        pose(0,0)=atof(splited[1].c_str());
        pose(0,1)=atof(splited[2].c_str());
        pose(0,2)=atof(splited[3].c_str());
        pose(0,3)=atof(splited[4].c_str());
        pose(1,0)=atof(splited[5].c_str());
        pose(1,1)=atof(splited[6].c_str());
        pose(1,2)=atof(splited[7].c_str());
        pose(1,3)=atof(splited[8].c_str());
        pose(2,0)=atof(splited[9].c_str());
        pose(2,1)=atof(splited[10].c_str());
        pose(2,2)=atof(splited[11].c_str());
        pose(2,3)=atof(splited[12].c_str());
//        std::cout<<pose(0,3)<<","<<pose(1,3)<<","<<pose(2,3)<<std::endl;
        poses.push_back(pose);
        
        tmp_count++;
    }
}

class IMUData_t{
    public:
        IMUData_t(const double& gx, const double& gy, const double& gz,
                     const double& ax, const double& ay, const double& az,
                     const double& t) :_g(gx,gy,gz), _a(ax,ay,az), _t(t){}
        IMUData_t(){};
        Eigen::Vector3d _g;    //gyr data
        Eigen::Vector3d _a;    //acc data
        double _t;      //timestamp
};

long unsigned int calIdFromGPS(double lat, double lon){
    return floor(lat*100)*floor(lon*100);
}

double get_avg_img_period(std::vector<double>& times){
    double avg_delta_time=0;
    int count=times.size()-1;
    for (int i=int(times.size()/2)-200; i<int(times.size()/2)+200; i++){
        avg_delta_time=avg_delta_time+(times[i]-times[i-1])/400;
    }
    return avg_delta_time;
}

void get_frm_times(std::vector<int>& frm_ids, double first_img_time, double dt, std::vector<double>& frm_times){
    for (int i=0;i<frm_ids.size(); i++){
        frm_times.push_back(frm_ids[i]*dt+first_img_time);
//        std::cout<<frm_ids[i]*dt+first_img_time<<std::endl;
    }
}

void process_data(std::string ws_addr){
    std::string img_pose_addr=ws_addr+"/frame_trajectory.txt";
    std::string img_time_addr=ws_addr+"/ExposureData.txt";
    std::string img_start_file=ws_addr+"/FirstImage.txt";
    std::string imu_addr=ws_addr+"/GyroData.txt";
    std::string cam_imu_addr=ws_addr+"/camera_imu.txt";
    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Vector4d>> poses;
    std::vector<int> frame_ids;
    read_frame_pose(img_pose_addr, poses, frame_ids);
    std::cout<<"poses: "<<poses.size()<<std::endl;
    std::vector<double> img_timess;
    read_img_time(img_time_addr, img_timess);
    std::cout<<"img_timess: "<<img_timess.size()<<std::endl;
    double first_img_time= read_img_start_time(img_start_file);
    std::cout<<"first_img_time: "<<first_img_time<<std::endl;
    double dt = get_avg_img_period(img_timess);
    std::cout<<"dt: "<<dt<<std::endl;
    std::vector<double> frm_times;
    get_frm_times(frame_ids, first_img_time, dt, frm_times);
    std::cout<<"frm_times: "<<frm_times.size()<<std::endl;
    std::vector<Eigen::Matrix<double, 7, 1>> imu_datas;
    read_imu_data(imu_addr, imu_datas);
    std::cout<<"imu_datas: "<<imu_datas.size()<<std::endl;
    Eigen::Matrix4d Tbc=Eigen::Matrix4d::Identity();
    read_cam_info(cam_imu_addr, Tbc);
    std::cout<<Tbc<<std::endl;
    alignToIMU(poses, imu_datas, frm_times, Tbc);
}


int main(int argc, char* argv[]) {
    process_data("/workspace/imu_scale_es");
    return 0;
}
