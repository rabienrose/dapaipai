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
    void AddRot(Eigen::Quaterniond rot_angle,Eigen::Vector3d rot_speed, Eigen::Vector3d acc, double timestamp);
    cv::Mat GetParoImage();
    cv::Mat GetRawImage();
    void ClearImgSphere();
    PANO::KeyFrame* CreateNewFrame(cv::Mat img, Eigen::Matrix3d dir);
    void AddNewFrame(PANO::KeyFrame *new_frame, bool& is_discard, std::vector<PANO::KeyFrame*>& candi_list, std::vector<PANO::KeyFrame*>& overlay_list);
    void DoOptimize();
    void CalSphereSurface(Eigen::Matrix3d cur_dir);
    void FindNearRay(Eigen::Matrix3d, std::vector<PANO::KeyFrame*>& candi_list, std::vector<PANO::KeyFrame*>& overlay_list);
    void FinalImg();
    Eigen::Quaterniond cam_dir;
    bool is_paint;
private:
    void align_img();
    void update();
    //std::vector<Eigen::Vector2d> GetGirdNearby(Eigen::Vector3d rot);
    std::queue<DataItem> raw_data;
    std::vector<RotItem> raw_rots;
    std::vector<ImageItem> raw_imgs;
    //std::vector<std::vector<unsigned char>> sphere_img;
    cv::Mat sphere_img;
    cv::Mat latest_img;
    Eigen::Matrix3d R_b_c;
    Eigen::Matrix3d R_unity_w;
    std::vector<Eigen::Vector3d> sphere_pts;
    std::vector<Eigen::Vector2i> sphere_pts_uv;
    cv::Mat K;
    cv::Mat DistCoef;
    double width;
    double height;
    double fx;
    double fy;
    double cx;
    double cy;
    PANO::PanoMap* pano_map;
    
};
#endif
