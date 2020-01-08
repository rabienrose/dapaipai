 

#ifndef OPENCV_STITCHING_AUTOCALIB_HPP
#define OPENCV_STITCHING_AUTOCALIB_HPP

#include "opencv2/core.hpp"
#include "matchers.hpp"
using namespace cv;
namespace chamo {
namespace panod {

//! @addtogroup stitching_autocalib
//! @{

/** @brief Tries to estimate focal lengths from the given homography under the assumption that the camera
undergoes rotations around its centre only.

@param H Homography.
@param f0 Estimated focal length along X axis.
@param f1 Estimated focal length along Y axis.
@param f0_ok True, if f0 was estimated successfully, false otherwise.
@param f1_ok True, if f1 was estimated successfully, false otherwise.

See "Construction of Panoramic Image Mosaics with Global and Local Alignment"
by Heung-Yeung Shum and Richard Szeliski.
 */
void CV_EXPORTS_W focalsFromHomography(const Mat &H, double &f0, double &f1, bool &f0_ok, bool &f1_ok);

/** @brief Estimates focal lengths for each given camera.

@param features Features of images.
@param pairwise_matches Matches between all image pairs.
@param focals Estimated focal lengths for each camera.
 */
void CV_EXPORTS estimateFocal(const std::vector<ImageFeatures> &features,
                              const std::vector<MatchesInfo> &pairwise_matches,
                              std::vector<double> &focals);

bool CV_EXPORTS_W calibrateRotatingCamera(const std::vector<Mat> &Hs,CV_OUT Mat &K);

//! @} stitching_autocalib

} // namespace panod
} // namespace cv

#endif // OPENCV_STITCHING_AUTOCALIB_HPP
