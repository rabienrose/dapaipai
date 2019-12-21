#ifndef ORBMATCHER_H
#define ORBMATCHER_H

#include<vector>
#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>
#include"KeyFrame.h"

namespace PANO
{

class ORBmatcher
{    
public:

    ORBmatcher(float nnratio=0.6, bool checkOri=true);

    // Computes the Hamming distance between two ORB descriptors
    static int DescriptorDistance(const cv::Mat &a, const cv::Mat &b);

    // Matching for the Map Initialization (only used in the monocular case)
    int SearchForInitialization(KeyFrame &F1, KeyFrame &F2, std::vector<int> &vnMatches12, int windowSize=5);
public:
    cv::Mat debug_img_ref;
    cv::Mat debug_img_cur;

    static const int TH_LOW;
    static const int TH_HIGH;
    static const int HISTO_LENGTH;


protected:
    float RadiusByViewingCos(const float &viewCos);

    void ComputeThreeMaxima(std::vector<int>* histo, const int L, int &ind1, int &ind2, int &ind3);

    float mfNNratio;
    bool mbCheckOrientation;
};

}

#endif // ORBMATCHER_H
