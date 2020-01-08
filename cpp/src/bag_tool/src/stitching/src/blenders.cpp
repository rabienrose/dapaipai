 

#include "precomp.hpp"
using namespace cv;
namespace chamo {
namespace panod {

static const float WEIGHT_EPS = 1e-5f;

Ptr<Blender> Blender::createDefault(int type, bool try_gpu)
{
    if (type == NO)
        return makePtr<Blender>();
    if (type == FEATHER)
        return makePtr<FeatherBlender>(try_gpu);
    if (type == MULTI_BAND)
        return makePtr<MultiBandBlender>(try_gpu);
    CV_Error(Error::StsBadArg, "unsupported blending method");
}


void Blender::prepare(const std::vector<Point> &corners, const std::vector<Size> &sizes)
{
    prepare(resultRoi(corners, sizes));
}


void Blender::prepare(Rect dst_roi)
{
    dst_.create(dst_roi.size(), CV_16SC3);
    dst_.setTo(Scalar::all(0));
    dst_mask_.create(dst_roi.size(), CV_8U);
    dst_mask_.setTo(Scalar::all(0));
    dst_roi_ = dst_roi;
}


void Blender::feed(InputArray _img, InputArray _mask, Point tl)
{
    Mat img = _img.getMat();
    Mat mask = _mask.getMat();
    Mat dst = dst_.getMat(ACCESS_RW);
    Mat dst_mask = dst_mask_.getMat(ACCESS_RW);
//    if(img.cols!=dst.cols || img.rows!=dst.rows){
//        std::cout<<img.cols<<" : "<<dst.cols<<std::endl;
//        std::cout<<mask.rows<<" : "<<mask.rows<<std::endl;
//        std::cout<<"img.cols!=dst.cols || img.rows!=dst.rows"<<std::endl;
//    }
//    for(int i=0; i<img.cols;i++){
//        for(int j=0; j<img.rows;j++){
//            if(mask.at<uchar>(j,i)){
//                dst.at<Point3_<short>>(j,i)=img.at<Point3_<short>>(j,i);
//                dst_mask.at<uchar>(j,i) |= mask.at<uchar>(j,i);
//            }
//        }
//    }

    CV_Assert(img.type() == CV_16SC3);
    CV_Assert(mask.type() == CV_8U);
    int dx = tl.x - dst_roi_.x;
    int dy = tl.y - dst_roi_.y;
    for (int y = 0; y < img.rows; ++y)
    {
        const Point3_<short> *src_row = img.ptr<Point3_<short> >(y);
        Point3_<short> *dst_row = dst.ptr<Point3_<short> >(dy + y);
        const uchar *mask_row = mask.ptr<uchar>(y);
        uchar *dst_mask_row = dst_mask.ptr<uchar>(dy + y);

        for (int x = 0; x < img.cols; ++x)
        {
            int final_u=dx + x;
            if(final_u>=dst_.cols){
                final_u=final_u-dst_.cols;
            }
            if (mask_row[x]){
                dst_row[final_u] = src_row[x];
            }
            dst_mask_row[final_u] |= mask_row[x];
        }
    }
}


void Blender::blend(InputOutputArray dst, InputOutputArray dst_mask)
{
    UMat mask;
    compare(dst_mask_, 0, mask, CMP_EQ);
    dst_.setTo(Scalar::all(0), mask);
    dst.assign(dst_);
    dst_mask.assign(dst_mask_);
    dst_.release();
    dst_mask_.release();
}

void FeatherBlender::prepare(Rect dst_roi)
{
    Blender::prepare(dst_roi);
    dst_weight_map_.create(dst_roi.size(), CV_32F);
    dst_weight_map_.setTo(0);
}


void FeatherBlender::feed(InputArray _img, InputArray mask, Point tl)
{
    Mat img = _img.getMat();
    Mat dst = dst_.getMat(ACCESS_RW);

    CV_Assert(img.type() == CV_16SC3);
    CV_Assert(mask.type() == CV_8U);

    createWeightMap(mask, sharpness_, weight_map_);
    Mat weight_map = weight_map_.getMat(ACCESS_READ);
    Mat dst_weight_map = dst_weight_map_.getMat(ACCESS_RW);

    int dx = tl.x - dst_roi_.x;
    int dy = tl.y - dst_roi_.y;

    for (int y = 0; y < img.rows; ++y)
    {
        const Point3_<short>* src_row = img.ptr<Point3_<short> >(y);
        Point3_<short>* dst_row = dst.ptr<Point3_<short> >(dy + y);
        const float* weight_row = weight_map.ptr<float>(y);
        float* dst_weight_row = dst_weight_map.ptr<float>(dy + y);
        for (int x = 0; x < img.cols; ++x)
        {
            dst_row[dx + x].x += static_cast<short>(src_row[x].x * weight_row[x]);
            dst_row[dx + x].y += static_cast<short>(src_row[x].y * weight_row[x]);
            dst_row[dx + x].z += static_cast<short>(src_row[x].z * weight_row[x]);
            dst_weight_row[dx + x] += weight_row[x];
        }
    }
}


void FeatherBlender::blend(InputOutputArray dst, InputOutputArray dst_mask)
{
    normalizeUsingWeightMap(dst_weight_map_, dst_);
    compare(dst_weight_map_, WEIGHT_EPS, dst_mask_, CMP_GT);
    Blender::blend(dst, dst_mask);
}


Rect FeatherBlender::createWeightMaps(const std::vector<UMat> &masks, const std::vector<Point> &corners,
                                      std::vector<UMat> &weight_maps)
{
    weight_maps.resize(masks.size());
    for (size_t i = 0; i < masks.size(); ++i)
        createWeightMap(masks[i], sharpness_, weight_maps[i]);

    Rect dst_roi = resultRoi(corners, masks);
    Mat weights_sum(dst_roi.size(), CV_32F);
    weights_sum.setTo(0);

    for (size_t i = 0; i < weight_maps.size(); ++i)
    {
        Rect roi(corners[i].x - dst_roi.x, corners[i].y - dst_roi.y,
                 weight_maps[i].cols, weight_maps[i].rows);
        add(weights_sum(roi), weight_maps[i], weights_sum(roi));
    }

    for (size_t i = 0; i < weight_maps.size(); ++i)
    {
        Rect roi(corners[i].x - dst_roi.x, corners[i].y - dst_roi.y,
                 weight_maps[i].cols, weight_maps[i].rows);
        Mat tmp = weights_sum(roi);
        tmp.setTo(1, tmp < std::numeric_limits<float>::epsilon());
        divide(weight_maps[i], tmp, weight_maps[i]);
    }

    return dst_roi;
}


MultiBandBlender::MultiBandBlender(int try_gpu, int num_bands, int weight_type)
{
    num_bands_ = 0;
    setNumBands(num_bands);

#if defined(HAVE_OPENCV_CUDAARITHM) && defined(HAVE_OPENCV_CUDAWARPING)
    can_use_gpu_ = try_gpu && cuda::getCudaEnabledDeviceCount();
    gpu_feed_idx_ = 0;
#else
    CV_UNUSED(try_gpu);
    can_use_gpu_ = false;
#endif

    CV_Assert(weight_type == CV_32F || weight_type == CV_16S);
    weight_type_ = weight_type;
}

void MultiBandBlender::prepare(Rect dst_roi)
{
    dst_roi_final_ = dst_roi;

    // Crop unnecessary bands
    double max_len = static_cast<double>(std::max(dst_roi.width, dst_roi.height));
    num_bands_ = std::min(actual_num_bands_, static_cast<int>(ceil(std::log(max_len) / std::log(2.0))));

    // Add border to the final image, to ensure sizes are divided by (1 << num_bands_)
    dst_roi.width += ((1 << num_bands_) - dst_roi.width % (1 << num_bands_)) % (1 << num_bands_);
    dst_roi.height += ((1 << num_bands_) - dst_roi.height % (1 << num_bands_)) % (1 << num_bands_);

    Blender::prepare(dst_roi);
    dst_pyr_laplace_.resize(num_bands_ + 1);
    dst_pyr_laplace_[0] = dst_;

    dst_band_weights_.resize(num_bands_ + 1);
    dst_band_weights_[0].create(dst_roi.size(), weight_type_);
    dst_band_weights_[0].setTo(0);

    for (int i = 1; i <= num_bands_; ++i)
    {
        dst_pyr_laplace_[i].create((dst_pyr_laplace_[i - 1].rows + 1) / 2,
            (dst_pyr_laplace_[i - 1].cols + 1) / 2, CV_16SC3);
        dst_band_weights_[i].create((dst_band_weights_[i - 1].rows + 1) / 2,
            (dst_band_weights_[i - 1].cols + 1) / 2, weight_type_);
        dst_pyr_laplace_[i].setTo(Scalar::all(0));
        dst_band_weights_[i].setTo(0);
    }
}

#ifdef HAVE_OPENCL
static bool ocl_MultiBandBlender_feed(InputArray _src, InputArray _weight,
        InputOutputArray _dst, InputOutputArray _dst_weight)
{
    String buildOptions = "-D DEFINE_feed";
    ocl::buildOptionsAddMatrixDescription(buildOptions, "src", _src);
    ocl::buildOptionsAddMatrixDescription(buildOptions, "weight", _weight);
    ocl::buildOptionsAddMatrixDescription(buildOptions, "dst", _dst);
    ocl::buildOptionsAddMatrixDescription(buildOptions, "dstWeight", _dst_weight);
    ocl::Kernel k("feed", ocl::stitching::multibandblend_oclsrc, buildOptions);
    if (k.empty())
        return false;

    UMat src = _src.getUMat();

    k.args(ocl::KernelArg::ReadOnly(src),
           ocl::KernelArg::ReadOnly(_weight.getUMat()),
           ocl::KernelArg::ReadWrite(_dst.getUMat()),
           ocl::KernelArg::ReadWrite(_dst_weight.getUMat())
           );

    size_t globalsize[2] = {(size_t)src.cols, (size_t)src.rows };
    return k.run(2, globalsize, NULL, false);
}
#endif

void MultiBandBlender::feed(InputArray _img, InputArray mask, Point tl)
{
    UMat img;
    img = _img.getUMat();
    CV_Assert(img.type() == CV_16SC3 || img.type() == CV_8UC3);
    CV_Assert(mask.type() == CV_8U);

    // Keep source image in memory with small border
//    int gap = 3 * (1 << num_bands_);
//    std::cout<<"gap: "<<gap<<std::endl;
//    std::cout<<"tl.x: "<<tl.x<<std::endl;
//    std::cout<<"_img.col: "<<_img.cols()<<std::endl;
    Point tl_new(std::max(dst_roi_.x, tl.x),
                 std::max(dst_roi_.y, tl.y));
    Point br_new(std::min(dst_roi_.br().x, tl.x + img.cols),
                 std::min(dst_roi_.br().y, tl.y + img.rows));
//
//    tl_new.x = dst_roi_.x + (((tl_new.x - dst_roi_.x) >> num_bands_) << num_bands_);
//    std::cout<<"(((tl_new.x - dst_roi_.x) >> num_bands_) << num_bands_): "<<(((tl_new.x - dst_roi_.x) >> num_bands_) << num_bands_)<<std::endl;
//    tl_new.y = dst_roi_.y + (((tl_new.y - dst_roi_.y) >> num_bands_) << num_bands_);
//    int width = br_new.x - tl_new.x;
//    std::cout<<"width: "<<width<<std::endl;
//    int height = br_new.y - tl_new.y;
//    width += ((1 << num_bands_) - width % (1 << num_bands_)) % (1 << num_bands_);
//    std::cout<<"width: "<<width<<std::endl;
//    height += ((1 << num_bands_) - height % (1 << num_bands_)) % (1 << num_bands_);
//    br_new.x = tl_new.x + width;
//    std::cout<<"br_new.x: "<<br_new.x<<std::endl;
//    br_new.y = tl_new.y + height;
//    int dy = std::max(br_new.y - dst_roi_.br().y, 0);
//    int dx = std::max(br_new.x - dst_roi_.br().x, 0);
//    std::cout<<"dx: "<<dx<<std::endl;
//    tl_new.x -= dx; br_new.x -= dx;
//    tl_new.y -= dy; br_new.y -= dy;
//
    int top = 0;
    int left = 0;
    int bottom = img.rows;
    int right = img.cols;

    // Create the source image Laplacian pyramid
    UMat img_with_border;
//    std::cout<<"top: "<<top<<std::endl;
//    std::cout<<"bottom: "<<bottom<<std::endl;
//    std::cout<<"left: "<<left<<std::endl;
//    std::cout<<"right: "<<right<<std::endl;
    copyMakeBorder(_img, img_with_border, top, bottom, left, right,
                   BORDER_REFLECT);
    std::vector<UMat> src_pyr_laplace;
    createLaplacePyr(img_with_border, num_bands_, src_pyr_laplace);
    // Create the weight map Gaussian pyramid
    UMat weight_map;
    std::vector<UMat> weight_pyr_gauss(num_bands_ + 1);

    if (weight_type_ == CV_32F)
    {
        mask.getUMat().convertTo(weight_map, CV_32F, 1./255.);
    }
    else // weight_type_ == CV_16S
    {
        mask.getUMat().convertTo(weight_map, CV_16S);
        UMat add_mask;
        compare(mask, 0, add_mask, CMP_NE);
        add(weight_map, Scalar::all(1), weight_map, add_mask);
    }

    copyMakeBorder(weight_map, weight_pyr_gauss[0], top, bottom, left, right, BORDER_CONSTANT);

    for (int i = 0; i < num_bands_; ++i)
        pyrDown(weight_pyr_gauss[i], weight_pyr_gauss[i + 1]);
    int y_tl = tl_new.y - dst_roi_.y;
    int y_br = br_new.y - dst_roi_.y;
    int x_tl = tl_new.x - dst_roi_.x;
    int x_br = br_new.x - dst_roi_.x;

    // Add weighted layer of the source image to the final Laplacian pyramid layer
    for (int i = 0; i <= num_bands_; ++i)
    {
        Rect rc(x_tl, y_tl, x_br - x_tl, y_br - y_tl);
        {
            Mat _src_pyr_laplace = src_pyr_laplace[i].getMat(ACCESS_READ);
            Mat _dst_pyr_laplace = dst_pyr_laplace_[i](rc).getMat(ACCESS_RW);
            Mat _weight_pyr_gauss = weight_pyr_gauss[i].getMat(ACCESS_READ);
            Mat _dst_band_weights = dst_band_weights_[i](rc).getMat(ACCESS_RW);
            if (weight_type_ == CV_32F)
            {
                for (int y = 0; y < rc.height; ++y)
                {
                    const Point3_<short>* src_row = _src_pyr_laplace.ptr<Point3_<short> >(y);
                    Point3_<short>* dst_row = _dst_pyr_laplace.ptr<Point3_<short> >(y);
                    const float* weight_row = _weight_pyr_gauss.ptr<float>(y);
                    float* dst_weight_row = _dst_band_weights.ptr<float>(y);

                    for (int x = 0; x < rc.width; ++x)
                    {
                        dst_row[x].x += static_cast<short>(src_row[x].x * weight_row[x]);
                        dst_row[x].y += static_cast<short>(src_row[x].y * weight_row[x]);
                        dst_row[x].z += static_cast<short>(src_row[x].z * weight_row[x]);
                        dst_weight_row[x] += weight_row[x];
                    }
                }
            }
            else // weight_type_ == CV_16S
            {
                for (int y = 0; y < y_br - y_tl; ++y)
                {
                    const Point3_<short>* src_row = _src_pyr_laplace.ptr<Point3_<short> >(y);
                    Point3_<short>* dst_row = _dst_pyr_laplace.ptr<Point3_<short> >(y);
                    const short* weight_row = _weight_pyr_gauss.ptr<short>(y);
                    short* dst_weight_row = _dst_band_weights.ptr<short>(y);

                    for (int x = 0; x < x_br - x_tl; ++x)
                    {
                        dst_row[x].x += short((src_row[x].x * weight_row[x]) >> 8);
                        dst_row[x].y += short((src_row[x].y * weight_row[x]) >> 8);
                        dst_row[x].z += short((src_row[x].z * weight_row[x]) >> 8);
                        dst_weight_row[x] += weight_row[x];
                    }
                }
            }
        }
        x_tl /= 2; y_tl /= 2;
        x_br /= 2; y_br /= 2;
    }
}


void MultiBandBlender::blend(InputOutputArray dst, InputOutputArray dst_mask)
{
    Rect dst_rc(0, 0, dst_roi_final_.width, dst_roi_final_.height);
    cv::UMat dst_band_weights_0;

    for (int i = 0; i <= num_bands_; ++i)
        normalizeUsingWeightMap(dst_band_weights_[i], dst_pyr_laplace_[i]);

    restoreImageFromLaplacePyr(dst_pyr_laplace_);

    dst_ = dst_pyr_laplace_[0](dst_rc);
    dst_band_weights_0 = dst_band_weights_[0];

    dst_pyr_laplace_.clear();
    dst_band_weights_.clear();

    compare(dst_band_weights_0(dst_rc), WEIGHT_EPS, dst_mask_, CMP_GT);

    Blender::blend(dst, dst_mask);
}


//////////////////////////////////////////////////////////////////////////////
// Auxiliary functions

#ifdef HAVE_OPENCL
static bool ocl_normalizeUsingWeightMap(InputArray _weight, InputOutputArray _mat)
{
    String buildOptions = "-D DEFINE_normalizeUsingWeightMap";
    ocl::buildOptionsAddMatrixDescription(buildOptions, "mat", _mat);
    ocl::buildOptionsAddMatrixDescription(buildOptions, "weight", _weight);
    ocl::Kernel k("normalizeUsingWeightMap", ocl::stitching::multibandblend_oclsrc, buildOptions);
    if (k.empty())
        return false;

    UMat mat = _mat.getUMat();

    k.args(ocl::KernelArg::ReadWrite(mat),
           ocl::KernelArg::ReadOnly(_weight.getUMat())
           );

    size_t globalsize[2] = {(size_t)mat.cols, (size_t)mat.rows };
    return k.run(2, globalsize, NULL, false);
}
#endif

void normalizeUsingWeightMap(InputArray _weight, InputOutputArray _src)
{
    Mat src;
    Mat weight;

#ifdef HAVE_OPENCL
    if ( !cv::ocl::isOpenCLActivated() ||
            !ocl_normalizeUsingWeightMap(_weight, _src) )
#endif
    {
        src = _src.getMat();
        weight = _weight.getMat();

        CV_Assert(src.type() == CV_16SC3);

        if (weight.type() == CV_32FC1)
        {
            for (int y = 0; y < src.rows; ++y)
            {
                Point3_<short> *row = src.ptr<Point3_<short> >(y);
                const float *weight_row = weight.ptr<float>(y);

                for (int x = 0; x < src.cols; ++x)
                {
                    row[x].x = static_cast<short>(row[x].x / (weight_row[x] + WEIGHT_EPS));
                    row[x].y = static_cast<short>(row[x].y / (weight_row[x] + WEIGHT_EPS));
                    row[x].z = static_cast<short>(row[x].z / (weight_row[x] + WEIGHT_EPS));
                }
            }
        }
        else
        {
            CV_Assert(weight.type() == CV_16SC1);

            for (int y = 0; y < src.rows; ++y)
            {
                const short *weight_row = weight.ptr<short>(y);
                Point3_<short> *row = src.ptr<Point3_<short> >(y);

                for (int x = 0; x < src.cols; ++x)
                {
                    int w = weight_row[x] + 1;
                    row[x].x = static_cast<short>((row[x].x << 8) / w);
                    row[x].y = static_cast<short>((row[x].y << 8) / w);
                    row[x].z = static_cast<short>((row[x].z << 8) / w);
                }
            }
        }
    }
#ifdef HAVE_OPENCL
    else
    {
        CV_IMPL_ADD(CV_IMPL_OCL);
    }
#endif
}


void createWeightMap(InputArray mask, float sharpness, InputOutputArray weight)
{
    CV_Assert(mask.type() == CV_8U);
    distanceTransform(mask, weight, DIST_L1, 3);
    UMat tmp;
    multiply(weight, sharpness, tmp);
    threshold(tmp, weight, 1.f, 1.f, THRESH_TRUNC);
}


void createLaplacePyr(InputArray img, int num_levels, std::vector<UMat> &pyr)
{
    pyr.resize(num_levels + 1);

    if(img.depth() == CV_8U)
    {
        if(num_levels == 0)
        {
            img.getUMat().convertTo(pyr[0], CV_16S);
            return;
        }

        UMat downNext;
        UMat current = img.getUMat();
        pyrDown(img, downNext);

        for(int i = 1; i < num_levels; ++i)
        {
            UMat lvl_up;
            UMat lvl_down;

            pyrDown(downNext, lvl_down);
            pyrUp(downNext, lvl_up, current.size());
            subtract(current, lvl_up, pyr[i-1], noArray(), CV_16S);

            current = downNext;
            downNext = lvl_down;
        }

        {
            UMat lvl_up;
            pyrUp(downNext, lvl_up, current.size());
            subtract(current, lvl_up, pyr[num_levels-1], noArray(), CV_16S);

            downNext.convertTo(pyr[num_levels], CV_16S);
        }
    }
    else
    {
        pyr[0] = img.getUMat();
        for (int i = 0; i < num_levels; ++i)
            pyrDown(pyr[i], pyr[i + 1]);
        UMat tmp;
        for (int i = 0; i < num_levels; ++i)
        {
            pyrUp(pyr[i + 1], tmp, pyr[i].size());
            subtract(pyr[i], tmp, pyr[i]);
        }
    }
}


void createLaplacePyrGpu(InputArray img, int num_levels, std::vector<UMat> &pyr)
{
#if defined(HAVE_OPENCV_CUDAARITHM) && defined(HAVE_OPENCV_CUDAWARPING)
    pyr.resize(num_levels + 1);

    std::vector<cuda::GpuMat> gpu_pyr(num_levels + 1);
    gpu_pyr[0].upload(img);
    for (int i = 0; i < num_levels; ++i)
        cuda::pyrDown(gpu_pyr[i], gpu_pyr[i + 1]);

    cuda::GpuMat tmp;
    for (int i = 0; i < num_levels; ++i)
    {
        cuda::pyrUp(gpu_pyr[i + 1], tmp);
        cuda::subtract(gpu_pyr[i], tmp, gpu_pyr[i]);
        gpu_pyr[i].download(pyr[i]);
    }

    gpu_pyr[num_levels].download(pyr[num_levels]);
#else
    CV_UNUSED(img);
    CV_UNUSED(num_levels);
    CV_UNUSED(pyr);
    CV_Error(Error::StsNotImplemented, "CUDA optimization is unavailable");
#endif
}


void restoreImageFromLaplacePyr(std::vector<UMat> &pyr)
{
    if (pyr.empty())
        return;
    UMat tmp;
    for (size_t i = pyr.size() - 1; i > 0; --i)
    {
        pyrUp(pyr[i], tmp, pyr[i - 1].size());
        add(tmp, pyr[i - 1], pyr[i - 1]);
    }
}


void restoreImageFromLaplacePyrGpu(std::vector<UMat> &pyr)
{
#if defined(HAVE_OPENCV_CUDAARITHM) && defined(HAVE_OPENCV_CUDAWARPING)
    if (pyr.empty())
        return;

    std::vector<cuda::GpuMat> gpu_pyr(pyr.size());
    for (size_t i = 0; i < pyr.size(); ++i)
        gpu_pyr[i].upload(pyr[i]);

    cuda::GpuMat tmp;
    for (size_t i = pyr.size() - 1; i > 0; --i)
    {
        cuda::pyrUp(gpu_pyr[i], tmp);
        cuda::add(tmp, gpu_pyr[i - 1], gpu_pyr[i - 1]);
    }

    gpu_pyr[0].download(pyr[0]);
#else
    CV_UNUSED(pyr);
    CV_Error(Error::StsNotImplemented, "CUDA optimization is unavailable");
#endif
}

} // namespace panod
} // namespace cv
