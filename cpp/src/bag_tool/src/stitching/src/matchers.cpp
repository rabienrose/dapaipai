#include "precomp.hpp"
using namespace chamo;
using namespace chamo::panod;
using namespace cv::detail;

namespace {
struct DistIdxPair
{
    bool operator<(const DistIdxPair &other) const { return dist < other.dist; }
    double dist;
    int idx;
};


struct MatchPairsBody : ParallelLoopBody
{
    MatchPairsBody(FeaturesMatcher &_matcher, const std::vector<ImageFeatures> &_features,
                   std::vector<MatchesInfo> &_pairwise_matches, std::vector<std::pair<int,int> > &_near_pairs)
            : matcher(_matcher), features(_features),
              pairwise_matches(_pairwise_matches), near_pairs(_near_pairs) {}

    void operator ()(const Range &r) const CV_OVERRIDE
    {
        cv::RNG rng = cv::theRNG(); // save entry rng state
        const int num_images = static_cast<int>(features.size());
        for (int i = r.start; i < r.end; ++i)
        {
            cv::theRNG() = cv::RNG(rng.state + i); // force "stable" RNG seed for each processed pair

            int from = near_pairs[i].first;
            int to = near_pairs[i].second;
            int pair_idx = from*num_images + to;

            matcher(features[from], features[to], pairwise_matches[pair_idx]);
            pairwise_matches[pair_idx].src_img_idx = from;
            pairwise_matches[pair_idx].dst_img_idx = to;

            size_t dual_pair_idx = to*num_images + from;

            pairwise_matches[dual_pair_idx] = pairwise_matches[pair_idx];
            pairwise_matches[dual_pair_idx].src_img_idx = to;
            pairwise_matches[dual_pair_idx].dst_img_idx = from;

            if (!pairwise_matches[pair_idx].H.empty())
                pairwise_matches[dual_pair_idx].H = pairwise_matches[pair_idx].H.inv();

            for (size_t j = 0; j < pairwise_matches[dual_pair_idx].matches.size(); ++j)
                std::swap(pairwise_matches[dual_pair_idx].matches[j].queryIdx,
                          pairwise_matches[dual_pair_idx].matches[j].trainIdx);
        }
    }

    FeaturesMatcher &matcher;
    const std::vector<ImageFeatures> &features;
    std::vector<MatchesInfo> &pairwise_matches;
    std::vector<std::pair<int,int> > &near_pairs;

private:
    void operator =(const MatchPairsBody&);
};


//////////////////////////////////////////////////////////////////////////////

typedef std::set<std::pair<int,int> > MatchesSet;

// These two classes are aimed to find features matches only, not to
// estimate homography

class CpuMatcher CV_FINAL : public FeaturesMatcher
{
public:
    CpuMatcher(float match_conf) : FeaturesMatcher(true), match_conf_(match_conf) {}
    void match(const ImageFeatures &features1, const ImageFeatures &features2, MatchesInfo& matches_info) CV_OVERRIDE;

private:
    float match_conf_;
};

void CpuMatcher::match(const ImageFeatures &features1, const ImageFeatures &features2, MatchesInfo& matches_info)
{
    CV_Assert(features1.descriptors.type() == features2.descriptors.type());
    CV_Assert(features2.descriptors.depth() == CV_8U || features2.descriptors.depth() == CV_32F);

    matches_info.matches.clear();

    Ptr<cv::DescriptorMatcher> matcher;
    {
        Ptr<flann::IndexParams> indexParams = makePtr<flann::KDTreeIndexParams>();
        Ptr<flann::SearchParams> searchParams = makePtr<flann::SearchParams>();

        if (features2.descriptors.depth() == CV_8U)
        {
            indexParams->setAlgorithm(cvflann::FLANN_INDEX_LSH);
            searchParams->setAlgorithm(cvflann::FLANN_INDEX_LSH);
        }

        matcher = makePtr<FlannBasedMatcher>(indexParams, searchParams);
    }
    std::vector< std::vector<DMatch> > pair_matches;
    MatchesSet matches;

    // Find 1->2 matches
    matcher->knnMatch(features1.descriptors, features2.descriptors, pair_matches, 2);
    for (size_t i = 0; i < pair_matches.size(); ++i)
    {
        if (pair_matches[i].size() < 2)
            continue;
        const DMatch& m0 = pair_matches[i][0];
        const DMatch& m1 = pair_matches[i][1];
        if (m0.distance < (1.f - match_conf_) * m1.distance)
        {
            matches_info.matches.push_back(m0);
            matches.insert(std::make_pair(m0.queryIdx, m0.trainIdx));
        }
    }
    // Find 2->1 matches
    pair_matches.clear();
    matcher->knnMatch(features2.descriptors, features1.descriptors, pair_matches, 2);
    for (size_t i = 0; i < pair_matches.size(); ++i)
    {
        if (pair_matches[i].size() < 2)
            continue;
        const DMatch& m0 = pair_matches[i][0];
        const DMatch& m1 = pair_matches[i][1];
        if (m0.distance < (1.f - match_conf_) * m1.distance)
            if (matches.find(std::make_pair(m0.trainIdx, m0.queryIdx)) == matches.end())
                matches_info.matches.push_back(DMatch(m0.trainIdx, m0.queryIdx, m0.distance));
    }
}

} // namespace


namespace chamo {
namespace panod {

void computeImageFeatures(
    const Ptr<Feature2D> &featuresFinder,
    InputArrayOfArrays  images,
    std::vector<ImageFeatures> &features,
    InputArrayOfArrays masks)
{
    // compute all features
    std::vector<std::vector<KeyPoint>> keypoints;
    std::vector<UMat> descriptors;
    // TODO replace with 1 call to new over load of detectAndCompute
    featuresFinder->detect(images, keypoints, masks);
    featuresFinder->compute(images, keypoints, descriptors);

    // store to ImageFeatures
    size_t count = images.total();
    features.resize(count);
    CV_Assert(count == keypoints.size() && count == descriptors.size());
    for (size_t i = 0; i < count; ++i)
    {
        features[i].img_size = images.size(int(i));
        features[i].keypoints = std::move(keypoints[i]);
        features[i].descriptors = std::move(descriptors[i]);
    }
}

void computeImageFeatures(
    const Ptr<Feature2D> &featuresFinder,
    InputArray image,
    ImageFeatures &features,
    InputArray mask)
{
    features.img_size = image.size();
    featuresFinder->detectAndCompute(image, mask, features.keypoints, features.descriptors);
}

//////////////////////////////////////////////////////////////////////////////

MatchesInfo::MatchesInfo() : src_img_idx(-1), dst_img_idx(-1), num_inliers(0), confidence(0) {}

MatchesInfo::MatchesInfo(const MatchesInfo &other) { *this = other; }

MatchesInfo& MatchesInfo::operator =(const MatchesInfo &other)
{
    src_img_idx = other.src_img_idx;
    dst_img_idx = other.dst_img_idx;
    matches = other.matches;
    inliers_mask = other.inliers_mask;
    num_inliers = other.num_inliers;
    H = other.H.clone();
    confidence = other.confidence;
    return *this;
}


//////////////////////////////////////////////////////////////////////////////

void FeaturesMatcher::operator ()(const std::vector<ImageFeatures> &features, std::vector<MatchesInfo> &pairwise_matches,
                                  const UMat &mask)
{
    const int num_images = static_cast<int>(features.size());

    CV_Assert(mask.empty() || (mask.type() == CV_8U && mask.cols == num_images && mask.rows));
    Mat_<uchar> mask_(mask.getMat(ACCESS_READ));
    if (mask_.empty())
        mask_ = Mat::ones(num_images, num_images, CV_8U);

    std::vector<std::pair<int,int> > near_pairs;
    for (int i = 0; i < num_images - 1; ++i)
        for (int j = i + 1; j < num_images; ++j)
            if (features[i].keypoints.size() > 0 && features[j].keypoints.size() > 0 && mask_(i, j))
                near_pairs.push_back(std::make_pair(i, j));

    pairwise_matches.clear(); // clear history values
    pairwise_matches.resize(num_images * num_images);
    MatchPairsBody body(*this, features, pairwise_matches, near_pairs);

    if (is_thread_safe_)
        parallel_for_(Range(0, static_cast<int>(near_pairs.size())), body);
    else
        body(Range(0, static_cast<int>(near_pairs.size())));
}


//////////////////////////////////////////////////////////////////////////////

BestOf2NearestMatcher::BestOf2NearestMatcher(bool try_use_gpu, float match_conf, int num_matches_thresh1, int num_matches_thresh2)
{
    CV_UNUSED(try_use_gpu);
    {
        impl_ = makePtr<CpuMatcher>(match_conf);
    }

    is_thread_safe_ = impl_->isThreadSafe();
    num_matches_thresh1_ = num_matches_thresh1;
    num_matches_thresh2_ = num_matches_thresh2;
}

Ptr<BestOf2NearestMatcher> BestOf2NearestMatcher::create(bool try_use_gpu, float match_conf, int num_matches_thresh1, int num_matches_thresh2)
{
    return makePtr<BestOf2NearestMatcher>(try_use_gpu, match_conf, num_matches_thresh1, num_matches_thresh2);
}



void BestOf2NearestMatcher::match(const ImageFeatures &features1, const ImageFeatures &features2,
                                  MatchesInfo &matches_info)
{
    (*impl_)(features1, features2, matches_info);

    // Check if it makes sense to find homography
    if (matches_info.matches.size() < static_cast<size_t>(num_matches_thresh1_))
        return;

    // Construct point-point correspondences for homography estimation
    Mat src_points(1, static_cast<int>(matches_info.matches.size()), CV_32FC2);
    Mat dst_points(1, static_cast<int>(matches_info.matches.size()), CV_32FC2);
    for (size_t i = 0; i < matches_info.matches.size(); ++i)
    {
        const DMatch& m = matches_info.matches[i];

        Point2f p = features1.keypoints[m.queryIdx].pt;
        p.x -= features1.img_size.width * 0.5f;
        p.y -= features1.img_size.height * 0.5f;
        src_points.at<Point2f>(0, static_cast<int>(i)) = p;

        p = features2.keypoints[m.trainIdx].pt;
        p.x -= features2.img_size.width * 0.5f;
        p.y -= features2.img_size.height * 0.5f;
        dst_points.at<Point2f>(0, static_cast<int>(i)) = p;
    }

    // Find pair-wise motion
    matches_info.H = findHomography(src_points, dst_points, matches_info.inliers_mask, RANSAC);
    if (matches_info.H.empty() || std::abs(determinant(matches_info.H)) < std::numeric_limits<double>::epsilon())
        return;

    // Find number of inliers
    matches_info.num_inliers = 0;
    for (size_t i = 0; i < matches_info.inliers_mask.size(); ++i)
        if (matches_info.inliers_mask[i])
            matches_info.num_inliers++;

    // These coeffs are from paper M. Brown and D. Lowe. "Automatic Panoramic Image Stitching
    // using Invariant Features"
    matches_info.confidence = matches_info.num_inliers / (8 + 0.3 * matches_info.matches.size());

    // Set zero confidence to remove matches between too close images, as they don't provide
    // additional information anyway. The threshold was set experimentally.
    matches_info.confidence = matches_info.confidence > 3. ? 0. : matches_info.confidence;

    // Check if we should try to refine motion
    if (matches_info.num_inliers < num_matches_thresh2_)
        return;

    // Construct point-point correspondences for inliers only
    src_points.create(1, matches_info.num_inliers, CV_32FC2);
    dst_points.create(1, matches_info.num_inliers, CV_32FC2);
    int inlier_idx = 0;
    for (size_t i = 0; i < matches_info.matches.size(); ++i)
    {
        if (!matches_info.inliers_mask[i])
            continue;

        const DMatch& m = matches_info.matches[i];

        Point2f p = features1.keypoints[m.queryIdx].pt;
        p.x -= features1.img_size.width * 0.5f;
        p.y -= features1.img_size.height * 0.5f;
        src_points.at<Point2f>(0, inlier_idx) = p;

        p = features2.keypoints[m.trainIdx].pt;
        p.x -= features2.img_size.width * 0.5f;
        p.y -= features2.img_size.height * 0.5f;
        dst_points.at<Point2f>(0, inlier_idx) = p;

        inlier_idx++;
    }

    // Rerun motion estimation on inliers only
    matches_info.H = findHomography(src_points, dst_points, RANSAC);
}

void BestOf2NearestMatcher::collectGarbage()
{
    impl_->collectGarbage();
}


BestOf2NearestRangeMatcher::BestOf2NearestRangeMatcher(int range_width, bool try_use_gpu, float match_conf, int num_matches_thresh1, int num_matches_thresh2): BestOf2NearestMatcher(try_use_gpu, match_conf, num_matches_thresh1, num_matches_thresh2)
{
    range_width_ = range_width;
}


void BestOf2NearestRangeMatcher::operator ()(const std::vector<ImageFeatures> &features, std::vector<MatchesInfo> &pairwise_matches,
                                  const UMat &mask)
{
    const int num_images = static_cast<int>(features.size());

    CV_Assert(mask.empty() || (mask.type() == CV_8U && mask.cols == num_images && mask.rows));
    Mat_<uchar> mask_(mask.getMat(ACCESS_READ));
    if (mask_.empty())
        mask_ = Mat::ones(num_images, num_images, CV_8U);

    std::vector<std::pair<int,int> > near_pairs;
    for (int i = 0; i < num_images - 1; ++i)
        for (int j = i + 1; j < std::min(num_images, i + range_width_); ++j)
            if (features[i].keypoints.size() > 0 && features[j].keypoints.size() > 0 && mask_(i, j))
                near_pairs.push_back(std::make_pair(i, j));

    pairwise_matches.resize(num_images * num_images);
    MatchPairsBody body(*this, features, pairwise_matches, near_pairs);

    if (is_thread_safe_)
        parallel_for_(Range(0, static_cast<int>(near_pairs.size())), body);
    else
        body(Range(0, static_cast<int>(near_pairs.size())));
}


void AffineBestOf2NearestMatcher::match(const ImageFeatures &features1, const ImageFeatures &features2,
                                        MatchesInfo &matches_info)
{
    (*impl_)(features1, features2, matches_info);

    // Check if it makes sense to find transform
    if (matches_info.matches.size() < static_cast<size_t>(num_matches_thresh1_))
        return;

    // Construct point-point correspondences for transform estimation
    Mat src_points(1, static_cast<int>(matches_info.matches.size()), CV_32FC2);
    Mat dst_points(1, static_cast<int>(matches_info.matches.size()), CV_32FC2);
    for (size_t i = 0; i < matches_info.matches.size(); ++i)
    {
        const cv::DMatch &m = matches_info.matches[i];
        src_points.at<Point2f>(0, static_cast<int>(i)) = features1.keypoints[m.queryIdx].pt;
        dst_points.at<Point2f>(0, static_cast<int>(i)) = features2.keypoints[m.trainIdx].pt;
    }

    // Find pair-wise motion
    if (full_affine_)
        matches_info.H = estimateAffine2D(src_points, dst_points, matches_info.inliers_mask);
    else
        matches_info.H = estimateAffinePartial2D(src_points, dst_points, matches_info.inliers_mask);

    if (matches_info.H.empty()) {
        // could not find transformation
        matches_info.confidence = 0;
        matches_info.num_inliers = 0;
        return;
    }

    // Find number of inliers
    matches_info.num_inliers = 0;
    for (size_t i = 0; i < matches_info.inliers_mask.size(); ++i)
        if (matches_info.inliers_mask[i])
            matches_info.num_inliers++;

    // These coeffs are from paper M. Brown and D. Lowe. "Automatic Panoramic
    // Image Stitching using Invariant Features"
    matches_info.confidence =
        matches_info.num_inliers / (8 + 0.3 * matches_info.matches.size());

    /* should we remove matches between too close images? */
    // matches_info.confidence = matches_info.confidence > 3. ? 0. : matches_info.confidence;

    // extend H to represent linear transformation in homogeneous coordinates
    matches_info.H.push_back(Mat::zeros(1, 3, CV_64F));
    matches_info.H.at<double>(2, 2) = 1;
}


} // namespace panod
} // namespace cv
