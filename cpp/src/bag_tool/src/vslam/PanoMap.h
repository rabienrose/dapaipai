#ifndef PAROMAP_H
#define PAROMAP_H
#include <opencv2/core/core.hpp>
#include <Eigen/Eigen>
namespace PANO
{
class KeyFrame;
struct GraphNode{
    KeyFrame* frame1;
    KeyFrame* frame2;
    Eigen::Matrix3d rot_1_2;
    int weight;
};
class PanoMap
{
public:
    PanoMap();
    std::vector<KeyFrame*> frames;
    std::vector<GraphNode> rot_graph;
};

}

#endif
