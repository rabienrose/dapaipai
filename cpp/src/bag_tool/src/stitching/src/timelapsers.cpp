 

#include "precomp.hpp"
using namespace cv;
namespace chamo {
namespace panod {

Ptr<Timelapser> Timelapser::createDefault(int type)
{
    if (type == AS_IS)
        return makePtr<Timelapser>();
    if (type == CROP)
        return makePtr<TimelapserCrop>();
    CV_Error(Error::StsBadArg, "unsupported timelapsing method");
}


void Timelapser::initialize(const std::vector<Point> &corners, const std::vector<Size> &sizes)
{
    dst_roi_ = resultRoi(corners, sizes);
    dst_.create(dst_roi_.size(), CV_16SC3);
}

void Timelapser::process(InputArray _img, InputArray /*_mask*/, Point tl)
{
    dst_.setTo(Scalar::all(0));

    Mat img = _img.getMat();
    Mat dst = dst_.getMat(ACCESS_RW);

    CV_Assert(img.type() == CV_16SC3);
    int dx = tl.x - dst_roi_.x;
    int dy = tl.y - dst_roi_.y;

    for (int y = 0; y < img.rows; ++y)
    {
        const Point3_<short> *src_row = img.ptr<Point3_<short> >(y);

        for (int x = 0; x < img.cols; ++x)
        {
            if (test_point(Point(tl.x + x, tl.y + y)))
            {
                Point3_<short> *dst_row = dst.ptr<Point3_<short> >(dy + y);
                dst_row[dx + x] = src_row[x];
            }
        }
    }
}


bool Timelapser::test_point(Point pt)
{
    return dst_roi_.contains(pt);
}


void TimelapserCrop::initialize(const std::vector<Point> &corners, const std::vector<Size> &sizes)
{
    dst_roi_ = resultRoiIntersection(corners, sizes);
    dst_.create(dst_roi_.size(), CV_16SC3);
}


} // namespace panod
} // namespace cv
