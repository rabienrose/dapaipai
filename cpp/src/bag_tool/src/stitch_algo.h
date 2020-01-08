#ifndef STITCH_ALGO
#define STITCH_ALGO
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <iostream>
#include <fstream>
#include <Eigen/Eigen>
#include <vector>
#include <string>
#include <unordered_map>
#include <queue>
#include <mutex>
#include <map>
#include <chrono>
#include <ctime>
#include <ros/ros.h>
#include <rosbag/bag.h>
namespace PANO{
    class PanoMap;
    class KeyFrame;
}

struct DataItem{
    cv::Mat img;
    Eigen::Quaterniond dir;
    Eigen::Vector3d rot;
    Eigen::Vector3d acc;
    double timestamp;
};
struct RotItem{
    double time;
    Eigen::Quaterniond dir;
    Eigen::Vector3d rot;
    Eigen::Vector3d acc;
};
struct ImageItem{
    double time;
    cv::Mat img;
};
class StitchAlgo{
public:
    StitchAlgo();
    void AddImage(cv::Mat img, double timestamp);
    void AddImageSimple(cv::Mat img, double timestamp);
    void AddRot(Eigen::Quaterniond rot_angle,Eigen::Vector3d rot_speed, Eigen::Vector3d acc, double timestamp);
    cv::Mat GetParoImage();
    cv::Mat GetRawImage();
    void ClearImgSphere();
    PANO::KeyFrame* CreateNewFrame(cv::Mat img, Eigen::Matrix3d dir);
    void DoOptimize();
    void CalSphereSurfaceByFrame(PANO::KeyFrame* frame);
    void FindNearRay(Eigen::Matrix3d, std::vector<PANO::KeyFrame*>& candi_list, std::vector<PANO::KeyFrame*>& overlay_list);
    cv::Mat FinalImg();
    void Reset();
    void switchPaint();
    void AddData(cv::Mat img, Eigen::Matrix3d dir);
    Eigen::Quaterniond cam_dir;
    std::string bag_root;
private:
    bool is_paint;
    void align_img();
    void update();
    int check_in_slot(Eigen::Matrix3d dir);
    void DrawMarder();
    std::queue<DataItem> raw_data;
    std::vector<RotItem> raw_rots;
    std::vector<ImageItem> raw_imgs;
    //std::vector<std::vector<unsigned char>> sphere_img;
    cv::Mat sphere_img;
    cv::Mat latest_img;
    Eigen::Matrix3d R_b_c;
    Eigen::Matrix3d R_pano_w;
    Eigen::Matrix3d R_unity_w;
    std::vector<Eigen::Vector3d> sphere_pts;
    std::vector<Eigen::Vector2i> sphere_pts_uv;
    cv::Mat K;
    cv::Mat DistCoef;
    std::map<int, PANO::KeyFrame*> frame_pool;
    double width;
    double height;
    double fx;
    double fy;
    double cx;
    double cy;
    PANO::PanoMap* pano_map;
    std::vector<Eigen::Vector3d> slot_posis;
    std::vector<Eigen::Vector2i> slot_uvs;
    std::chrono::time_point<std::chrono::system_clock> last_update_time;
    std::shared_ptr<rosbag::Bag> bag_ptr;
    int image_count;
    
};
#endif
