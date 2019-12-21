#ifndef KEYFRAME_H
#define KEYFRAME_H
#include <opencv2/core/core.hpp>
#include <Eigen/Eigen>
namespace PANO
{
#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64
class ORBextractor;
class KeyFrame
{
public:
    KeyFrame(cv::Mat img);
    ~KeyFrame();
    std::vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r) const;
    void AssignFeaturesToGrid();
    bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);
    static ORBextractor* mpORBextractor;
    static float fx;
    static float fy;
    static float cx;
    static float cy;
    static float mnMinX;
    static float mnMaxX;
    static float mnMinY;
    static float mnMaxY;
    static float mfGridElementWidthInv;
    static float mfGridElementHeightInv;
    std::vector<cv::KeyPoint> mvKeys;
    cv::Mat mDescriptors;
    cv::Mat color_img;
    Eigen::Matrix3d direction;
    int frame_id;
    int N;
    std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];
};

}

#endif
