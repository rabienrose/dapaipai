 


#ifndef OPENCV_STITCHING_TIMELAPSERS_HPP
#define OPENCV_STITCHING_TIMELAPSERS_HPP

#include "opencv2/core.hpp"
using namespace cv;
using namespace cv::detail;
namespace chamo {
namespace panod {

//! @addtogroup stitching
//! @{

//  Base Timelapser class, takes a sequence of images, applies appropriate shift, stores result in dst_.

class CV_EXPORTS_W Timelapser
{
public:

    enum {AS_IS, CROP};

    virtual ~Timelapser() {}

    CV_WRAP static Ptr<Timelapser> createDefault(int type);

    CV_WRAP virtual void initialize(const std::vector<Point> &corners, const std::vector<Size> &sizes);
    CV_WRAP virtual void process(InputArray img, InputArray mask, Point tl);
    CV_WRAP virtual const UMat& getDst() {return dst_;}

protected:

    virtual bool test_point(Point pt);

    UMat dst_;
    Rect dst_roi_;
};


class CV_EXPORTS_W TimelapserCrop : public Timelapser
{
public:
    virtual void initialize(const std::vector<Point> &corners, const std::vector<Size> &sizes) CV_OVERRIDE;
};

//! @}

} // namespace panod
} // namespace cv

#endif // OPENCV_STITCHING_TIMELAPSERS_HPP
