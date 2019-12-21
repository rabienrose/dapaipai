#ifndef RAYPOINT_H
#define RAYPOINT_H
#include <opencv2/core/core.hpp>
#include <Eigen/Eigen>
namespace PANO
{
class KeyFrame;
struct TrackItem{
    KeyFrame* frame;
    int kp_index;
};
class RayPoint
{
public:
    RayPoint();
    std::vector<TrackItem> track;
};

}

#endif
