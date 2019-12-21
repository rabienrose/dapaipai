#include "ORBmatcher.h"
#include <limits.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <stdint.h>
#include <stdlib.h>
#include <iostream>
using namespace std;

namespace PANO
{

const int ORBmatcher::TH_HIGH = 100;
const int ORBmatcher::TH_LOW = 50;
const int ORBmatcher::HISTO_LENGTH = 30;

ORBmatcher::ORBmatcher(float nnratio, bool checkOri): mfNNratio(nnratio), mbCheckOrientation(checkOri)
{
}

float ORBmatcher::RadiusByViewingCos(const float &viewCos)
{
    if(viewCos>0.998)
        return 2.5;
    else
        return 4.0;
}

int ORBmatcher::SearchForInitialization(KeyFrame &F1, KeyFrame &F2, vector<int> &vnMatches12, int windowSize)
{
    int nmatches=0;
    vnMatches12 = vector<int>(F1.mvKeys.size(),-1);
    vector<int> vMatchedDistance(F2.mvKeys.size(),INT_MAX);
//     cv::Mat ref_dis=debug_img_ref.clone();
//     cv::Mat cur_dis=debug_img_cur.clone();
    float avg_diff=0;
    for(size_t i1=0, iend1=F1.mvKeys.size(); i1<iend1; i1++)
    {
        cv::KeyPoint kp1 = F1.mvKeys[i1];
        double x1=(kp1.pt.x-KeyFrame::cx)/KeyFrame::fx;
        double y1=(kp1.pt.y-KeyFrame::cy)/KeyFrame::fy;
        Eigen::Vector3d v1(x1,y1,1);
        
        Eigen::Vector3d v2=F2.direction.transpose()*F1.direction*v1;
        double predict_u=KeyFrame::fx*v2.x()/v2.z()+KeyFrame::cx;
        double predict_v=KeyFrame::fy*v2.y()/v2.z()+KeyFrame::cy;
        vector<size_t> vIndices2 = F2.GetFeaturesInArea(predict_u,predict_v, windowSize);
//         cv::Scalar temp_c= cv::Scalar(rand() % 255, rand() % 255, rand() % 255, 255);
//         cv::circle(cur_dis, cv::Point2f(predict_u, predict_v), 2, temp_c, 2);
//         cv::circle(ref_dis, kp1.pt,2, temp_c, 2);
//         for(int i=0; i<vIndices2.size(); i++){
//             cv::line(cur_dis, F2.mvKeys[vIndices2[i]].pt, cv::Point2f(predict_u, predict_v), temp_c);
//         }
        if(vIndices2.empty())
            continue;
        cv::Mat d1 = F1.mDescriptors.row(i1);
        int bestDist = INT_MAX;
        int bestDist2 = INT_MAX;
        int bestIdx2 = -1;
        //std::cout<<"========================="<<std::endl;
        for(vector<size_t>::iterator vit=vIndices2.begin(); vit!=vIndices2.end(); vit++)
        {
            size_t i2 = *vit;

            cv::Mat d2 = F2.mDescriptors.row(i2);

            int dist = DescriptorDistance(d1,d2);
            //std::cout<<dist<<",";

            if(vMatchedDistance[i2]<=dist)
                continue;

            if(dist<bestDist)
            {
                bestDist2=bestDist;
                bestDist=dist;
                bestIdx2=i2;
            }
            else if(dist<bestDist2)
            {
                bestDist2=dist;
            }
        }
        //std::cout<<std::endl<<"========================="<<std::endl;
        if(bestDist<=TH_LOW)
        {
            vnMatches12[i1]=bestIdx2;
            vMatchedDistance[bestIdx2]=bestDist;
            //std::cout<<bestDist<<std::endl;
            avg_diff=avg_diff+bestDist;
//             cv::line(cur_dis, F2.mvKeys[bestIdx2].pt, F1.mvKeys[i1].pt, temp_c);
            nmatches++;
        }
    }
//     cv::imshow("f1", ref_dis);
//     cv::imshow("f2", cur_dis);

    return nmatches;
}

void ORBmatcher::ComputeThreeMaxima(vector<int>* histo, const int L, int &ind1, int &ind2, int &ind3)
{
    int max1=0;
    int max2=0;
    int max3=0;

    for(int i=0; i<L; i++)
    {
        const int s = histo[i].size();
        if(s>max1)
        {
            max3=max2;
            max2=max1;
            max1=s;
            ind3=ind2;
            ind2=ind1;
            ind1=i;
        }
        else if(s>max2)
        {
            max3=max2;
            max2=s;
            ind3=ind2;
            ind2=i;
        }
        else if(s>max3)
        {
            max3=s;
            ind3=i;
        }
    }

    if(max2<0.1f*(float)max1)
    {
        ind2=-1;
        ind3=-1;
    }
    else if(max3<0.1f*(float)max1)
    {
        ind3=-1;
    }
}


// Bit set count operation from
// http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
int ORBmatcher::DescriptorDistance(const cv::Mat &a, const cv::Mat &b)
{
    const int *pa = a.ptr<int32_t>();
    const int *pb = b.ptr<int32_t>();

    int dist=0;
    int count=a.cols/4;
    for(int i=0; i<count; i++, pa++, pb++)
    {
        unsigned  int v = *pa ^ *pb;
        v = v - ((v >> 1) & 0x55555555);
        v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
        dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
    }

    return dist;
}

}
