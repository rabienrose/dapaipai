 

#ifndef OPENCV_STITCHING_WARPER_CREATORS_HPP
#define OPENCV_STITCHING_WARPER_CREATORS_HPP

#include "chamo/stitching/panod/warpers.hpp"
#include <string>
using namespace cv;
using namespace cv::detail;
namespace chamo {
class CV_EXPORTS_W WarperCreator
{
public:
    CV_WRAP virtual ~WarperCreator() {}
    virtual Ptr<panod::RotationWarper> create(float scale) const = 0;
};


/** @brief Plane warper factory class.
  @sa panod::PlaneWarper
 */
class CV_EXPORTS  PlaneWarper : public WarperCreator
{
public:
    Ptr<panod::RotationWarper> create(float scale) const CV_OVERRIDE { return makePtr<panod::PlaneWarper>(scale); }
};

/** @brief Affine warper factory class.
  @sa panod::AffineWarper
 */
class CV_EXPORTS  AffineWarper : public WarperCreator
{
public:
    Ptr<panod::RotationWarper> create(float scale) const CV_OVERRIDE { return makePtr<panod::AffineWarper>(scale); }
};

/** @brief Cylindrical warper factory class.
@sa panod::CylindricalWarper
*/
class CV_EXPORTS CylindricalWarper: public WarperCreator
{
public:
    Ptr<panod::RotationWarper> create(float scale) const CV_OVERRIDE { return makePtr<panod::CylindricalWarper>(scale); }
};

/** @brief Spherical warper factory class */
class CV_EXPORTS SphericalWarper: public WarperCreator
{
public:
    Ptr<panod::RotationWarper> create(float scale) const CV_OVERRIDE { return makePtr<panod::SphericalWarper>(scale); }
};

class CV_EXPORTS FisheyeWarper : public WarperCreator
{
public:
    Ptr<panod::RotationWarper> create(float scale) const CV_OVERRIDE { return makePtr<panod::FisheyeWarper>(scale); }
};

class CV_EXPORTS StereographicWarper: public WarperCreator
{
public:
    Ptr<panod::RotationWarper> create(float scale) const CV_OVERRIDE { return makePtr<panod::StereographicWarper>(scale); }
};

class CV_EXPORTS CompressedRectilinearWarper: public WarperCreator
{
    float a, b;
public:
    CompressedRectilinearWarper(float A = 1, float B = 1)
    {
        a = A; b = B;
    }
    Ptr<panod::RotationWarper> create(float scale) const CV_OVERRIDE { return makePtr<panod::CompressedRectilinearWarper>(scale, a, b); }
};

class CV_EXPORTS CompressedRectilinearPortraitWarper: public WarperCreator
{
    float a, b;
public:
    CompressedRectilinearPortraitWarper(float A = 1, float B = 1)
    {
        a = A; b = B;
    }
    Ptr<panod::RotationWarper> create(float scale) const CV_OVERRIDE { return makePtr<panod::CompressedRectilinearPortraitWarper>(scale, a, b); }
};

class CV_EXPORTS PaniniWarper: public WarperCreator
{
    float a, b;
public:
    PaniniWarper(float A = 1, float B = 1)
    {
        a = A; b = B;
    }
    Ptr<panod::RotationWarper> create(float scale) const CV_OVERRIDE { return makePtr<panod::PaniniWarper>(scale, a, b); }
};

class CV_EXPORTS PaniniPortraitWarper: public WarperCreator
{
    float a, b;
public:
    PaniniPortraitWarper(float A = 1, float B = 1)
    {
        a = A; b = B;
    }
    Ptr<panod::RotationWarper> create(float scale) const CV_OVERRIDE { return makePtr<panod::PaniniPortraitWarper>(scale, a, b); }
};

class CV_EXPORTS MercatorWarper: public WarperCreator
{
public:
    Ptr<panod::RotationWarper> create(float scale) const CV_OVERRIDE { return makePtr<panod::MercatorWarper>(scale); }
};

class CV_EXPORTS TransverseMercatorWarper: public WarperCreator
{
public:
    Ptr<panod::RotationWarper> create(float scale) const CV_OVERRIDE { return makePtr<panod::TransverseMercatorWarper>(scale); }
};



#ifdef HAVE_OPENCV_CUDAWARPING
class PlaneWarperGpu: public WarperCreator
{
public:
    Ptr<panod::RotationWarper> create(float scale) const CV_OVERRIDE { return makePtr<panod::PlaneWarperGpu>(scale); }
};


class CylindricalWarperGpu: public WarperCreator
{
public:
    Ptr<panod::RotationWarper> create(float scale) const CV_OVERRIDE { return makePtr<panod::CylindricalWarperGpu>(scale); }
};


class SphericalWarperGpu: public WarperCreator
{
public:
    Ptr<panod::RotationWarper> create(float scale) const CV_OVERRIDE { return makePtr<panod::SphericalWarperGpu>(scale); }
};
#endif

//! @} stitching_warp

} // namespace cv

#endif // OPENCV_STITCHING_WARPER_CREATORS_HPP
