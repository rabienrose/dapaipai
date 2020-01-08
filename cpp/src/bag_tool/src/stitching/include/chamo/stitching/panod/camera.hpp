 

#ifndef OPENCV_STITCHING_CAMERA_HPP
#define OPENCV_STITCHING_CAMERA_HPP

#include "opencv2/core.hpp"
using namespace cv;
namespace chamo {
namespace panod {

//! @addtogroup stitching
//! @{

/** @brief Describes camera parameters.

@note Translation is assumed to be zero during the whole stitching pipeline. :
 */
struct CV_EXPORTS_W_SIMPLE CameraParams
{
    CameraParams();
    CameraParams(const CameraParams& other);
    CameraParams& operator =(const CameraParams& other);
    CV_WRAP Mat K() const;

    CV_PROP_RW double focal; // Focal length
    CV_PROP_RW double aspect; // Aspect ratio
    CV_PROP_RW double ppx; // Principal point X
    CV_PROP_RW double ppy; // Principal point Y
    CV_PROP_RW Mat R; // Rotation
    CV_PROP_RW Mat t; // Translation
};

//! @}

} // namespace panod
} // namespace cv

#endif // #ifndef OPENCV_STITCHING_CAMERA_HPP
