#include "KeyFrame.h"
#include "ORBextractor.h"
#include <iostream>
using namespace std;
namespace PANO
{
float KeyFrame::fx;
float KeyFrame::fy;
float KeyFrame::cx;
float KeyFrame::cy;
float KeyFrame::mnMinX;
float KeyFrame::mnMaxX;
float KeyFrame::mnMinY;
float KeyFrame::mnMaxY;
float KeyFrame::mfGridElementWidthInv;
float KeyFrame::mfGridElementHeightInv;
ORBextractor* KeyFrame::mpORBextractor;
KeyFrame::KeyFrame(cv::Mat img){

    color_img=img.clone();
    cv::Mat img_gray;
    if(color_img.type()==CV_8UC4){
        cv::cvtColor(color_img, img_gray, cv::COLOR_BGRA2GRAY);
    }else{
        img_gray=color_img;
    }
    if(img_gray.type()!=CV_8UC1){
        std::cout<<"input img format wrong!!"<<std::endl;
        return;
    }
    mpORBextractor->ExtractDesc(img_gray,cv::Mat(),mvKeys,mDescriptors,true);
    N = mvKeys.size();
    if(mvKeys.empty()){
        std::cout<<"no keypoints get"<<std::endl;
        return;
    }
        
    mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
    mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);
    AssignFeaturesToGrid();
}
KeyFrame::~KeyFrame(){
    color_img.release();
    mDescriptors.release();
}

bool KeyFrame::PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY)
{
    posX = round((kp.pt.x-mnMinX)*mfGridElementWidthInv);
    posY = round((kp.pt.y-mnMinY)*mfGridElementHeightInv);

    //Keypoint's coordinates are undistorted, which could cause to go out of the image
    if(posX<0 || posX>=FRAME_GRID_COLS || posY<0 || posY>=FRAME_GRID_ROWS)
        return false;

    return true;
}

void KeyFrame::AssignFeaturesToGrid()
{
    int nReserve = 0.5f*N/(FRAME_GRID_COLS*FRAME_GRID_ROWS);
    for(unsigned int i=0; i<FRAME_GRID_COLS;i++)
        for (unsigned int j=0; j<FRAME_GRID_ROWS;j++)
            mGrid[i][j].reserve(nReserve);

    for(int i=0;i<N;i++)
    {
        const cv::KeyPoint &kp = mvKeys[i];

        int nGridPosX, nGridPosY;
        if(PosInGrid(kp,nGridPosX,nGridPosY))
            mGrid[nGridPosX][nGridPosY].push_back(i);
    }
}

vector<size_t> KeyFrame::GetFeaturesInArea(const float &x, const float  &y, const float  &r) const
{
    vector<size_t> vIndices;
    vIndices.reserve(N);

    const int nMinCellX = max(0,(int)floor((x-mnMinX-r)*mfGridElementWidthInv));
    if(nMinCellX>=FRAME_GRID_COLS)
        return vIndices;

    const int nMaxCellX = min((int)FRAME_GRID_COLS-1,(int)ceil((x-mnMinX+r)*mfGridElementWidthInv));
    if(nMaxCellX<0)
        return vIndices;

    const int nMinCellY = max(0,(int)floor((y-mnMinY-r)*mfGridElementHeightInv));
    if(nMinCellY>=FRAME_GRID_ROWS)
        return vIndices;

    const int nMaxCellY = min((int)FRAME_GRID_ROWS-1,(int)ceil((y-mnMinY+r)*mfGridElementHeightInv));
    if(nMaxCellY<0)
        return vIndices;

    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            const vector<size_t> vCell = mGrid[ix][iy];
            if(vCell.empty())
                continue;

            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = mvKeys[vCell[j]];

                const float distx = kpUn.pt.x-x;
                const float disty = kpUn.pt.y-y;

                if(fabs(distx)<r && fabs(disty)<r)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}


}
