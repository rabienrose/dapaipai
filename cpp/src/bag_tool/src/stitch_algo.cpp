#include "stitch_algo.h"
#include "KeyFrame.h"
#include "PanoMap.h"
#include "Optimizer.h"
#include "Initializer.h"
#include "ORBmatcher.h"
#include "ORBextractor.h"
#include <cmath>
#include <sensor_msgs/image_encodings.h>
#include <rosbag/view.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <time.h>
#include "chamo/stitching/panod/autocalib.hpp"
#include "chamo/stitching/panod/blenders.hpp"
#include "chamo/stitching/panod/timelapsers.hpp"
#include "chamo/stitching/panod/camera.hpp"
#include "chamo/stitching/panod/exposure_compensate.hpp"
#include "chamo/stitching/panod/matchers.hpp"
#include "chamo/stitching/panod/motion_estimators.hpp"
#include "chamo/stitching/panod/seam_finders.hpp"
#include "chamo/stitching/panod/warpers.hpp"
#include "chamo/stitching/warpers.hpp"
#include "opencv2/xfeatures2d/nonfree.hpp"
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <ctime>
void interDouble(double v1, double v2, double t1, double t2, double& v3_out, double t3){
    v3_out=v1+(v2-v1)*(t3-t1)/(t2-t1);
}

double degree2radian(double degree){
    return degree/180.0*3.1415926;
}

double radian2degree(double radian){
    return radian*180.0/3.1415926;
}

Eigen::Matrix3d eular2matrix(Eigen::Vector3d rot){
    //std::cout<<"a: "<<rot.x()<<"b: "<<rot.y()<<"c: "<<rot.z()<<std::endl;
    Eigen::AngleAxisd rollAngle(rot.z(), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(rot.x(), Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd pitchAngle(rot.y(), Eigen::Vector3d::UnitX());
    Eigen::Quaternion<double> q = yawAngle * pitchAngle * rollAngle;
    //std::cout<<"a: "<<q.x()<<"b: "<<q.y()<<"c: "<<q.z()<<std::endl;
    Eigen::Matrix3d rotationMatrix = q.matrix();
    return rotationMatrix;
}

void StitchAlgo::switchPaint(){
    if(is_paint==false){
        std::cout<<"start!!"<<std::endl;
        is_paint=true;
    }else{
        std::cout<<"end!!"<<std::endl;
        is_paint=false;
        if(true){
            bag_ptr.reset(new rosbag::Bag());
            time_t now = time(0);
            tm *ltm = localtime(&now);
            std::stringstream ss;
            ss<<1900 + ltm->tm_year<<"_"<<1 + ltm->tm_mon<<"_"<<ltm->tm_mday<<"_"<<ltm->tm_hour<<"_"<<ltm->tm_min<<"_"<<ltm->tm_sec<<".bag";
            std::string full_file_name=bag_root+"/"+ss.str();
            std::cout<<full_file_name<<std::endl;
            bag_ptr->open(full_file_name.c_str(), rosbag::bagmode::Write);
            int count=0;
            for (auto it=frame_pool.begin(); it!=frame_pool.end(); ++it){
                geometry_msgs::QuaternionStamped dir_msg;
                Eigen::Quaterniond qua(it->second->direction);
                dir_msg.quaternion.x=qua.x();
                dir_msg.quaternion.y=qua.y();
                dir_msg.quaternion.z=qua.z();
                dir_msg.quaternion.w=qua.w();
                dir_msg.header.seq=count;
                bag_ptr->write("dir", ros::Time(now+count), dir_msg);
                sensor_msgs::CompressedImage img_ros_img;
                std::vector<unsigned char> binaryBuffer_;
                cv::imencode(".jpg", it->second->color_img, binaryBuffer_);
                img_ros_img.data=binaryBuffer_;
                img_ros_img.header.seq=count;
                img_ros_img.format="jpeg";
                bag_ptr->write("img", ros::Time(now+count), img_ros_img);
                count++;
                std::cout<<count<<std::endl;
            }
            bag_ptr->close();
        }
        FinalImg();
    }
}

//int color2int(cv::Scalar color){
//    int c;
//    //c=color.r<<24+color.g<<24
//}
StitchAlgo::StitchAlgo(){
    R_b_c<<0.00007493, -0.99962055,  0.02754549, -0.99994504, -0.00036369, -0.01047809, 0.01048413, -0.02754319, -0.99956563;
    R_pano_w=Eigen::Matrix3d::Zero();
    R_pano_w(0,1)=1;
    R_pano_w(1,2)=1;
    R_pano_w(2,0)=1;
    R_unity_w<<1, 0,  0, 0, 0, 1, 0, -1, 0;
    is_paint=false;
    std::vector<Eigen::Vector2i> slot_uvs_temp;
//    for (int v =900-300; v<900+301;v=v+300){
//        for (int u =200; u<3600; u=u+400){
//            slot_uvs_temp.push_back(Eigen::Vector2i(u,v));
//        }
//    }
//    for (int v =900-600; v<900+601;v=v+1200){
//        for (int u =300; u<3600; u=u+600){
//            slot_uvs_temp.push_back(Eigen::Vector2i(u,v));
//        }
//    }
    for (int v =900-200; v<900+201;v=v+200){
        for (int u =150; u<3600; u=u+300){
            slot_uvs_temp.push_back(Eigen::Vector2i(u,v));
        }
    }
    for (int v =900-400; v<900+401;v=v+800){
        for (int u =200; u<3600; u=u+400){
            slot_uvs_temp.push_back(Eigen::Vector2i(u,v));
        }
    }
    for (int v =900-600; v<900+601;v=v+1200){
        for (int u =300; u<3600; u=u+600){
            slot_uvs_temp.push_back(Eigen::Vector2i(u,v));
        }
    }

    for (float v = 0; v < 180; v = v + 0.1)
    {
        for (float u = 0; u < 360; u = u + 0.1) {
            Eigen::Vector3d posi;
            posi.z() = sin(degree2radian(180-v)) * cos(degree2radian(u));
            posi.x() = sin(degree2radian(180-v)) * sin(degree2radian(u));
            posi.y() = cos(degree2radian(180-v));
            sphere_pts.push_back(posi);
            Eigen::Vector2i uv;
            uv.x()=u*10;
            uv.y()=v*10;
            sphere_pts_uv.push_back(uv);
            for(int i=0; i<slot_uvs_temp.size(); i++){
                if(slot_uvs_temp[i].x()==uv.x() && slot_uvs_temp[i].y()==uv.y()){
                    slot_posis.push_back(posi);
                    slot_uvs.push_back(slot_uvs_temp[i]);
                }
            }
        }
    }
    std::cout<<"slot_posis.size: "<<slot_posis.size()<<std::endl;
    width=640;
    height=480;
    fx=945/1280.0*640.0;
    fy=945/1280.0*640.0;
    cx=640/1280.0*640.0;
    cy=340/1280.0*640.0;
    K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    cv::Mat DistCoef =cv::Mat(4,1,CV_32F);
    DistCoef.at<float>(0) = 0.097;
    DistCoef.at<float>(1) = -0.1506;
    DistCoef.at<float>(2) = 4.9496e-04;
    DistCoef.at<float>(3) = -4.4399e-04;
    pano_map=new PANO::PanoMap();
    PANO::KeyFrame::fx=K.at<float>(0,0);
    PANO::KeyFrame::fy=K.at<float>(1,1);
    PANO::KeyFrame::cx=K.at<float>(0,2);
    PANO::KeyFrame::cy=K.at<float>(1,2);
    PANO::KeyFrame::mnMinX = 0.0f;
    PANO::KeyFrame::mnMaxX = width;
    PANO::KeyFrame::mnMinY = 0.0f;
    PANO::KeyFrame::mnMaxY = height;
    PANO::KeyFrame::mpORBextractor = new PANO::ORBextractor(2000,1.2,8,20,7);
    ClearImgSphere();
    last_update_time = std::chrono::system_clock::now();
    image_count=0;
}

PANO::KeyFrame* StitchAlgo::CreateNewFrame(cv::Mat img, Eigen::Matrix3d dir){
    PANO::KeyFrame* frame = new PANO::KeyFrame(img);
    //std::cout<<frame->mvKeys.size()<<std::endl;
    frame->direction=dir;
    return frame;
}

Eigen::Vector3d getPointOnSphere(Eigen::Matrix3d dir1){
    Eigen::Vector3d one_v(0,0,1);
    return dir1*one_v;
}

float CalRayAngle(Eigen::Vector3d ray1, Eigen::Vector3d ray2){
    double x1=ray1(0);
    double y1=ray1(1);
    double z1=ray1(2);
    double x2=ray2(0);
    double y2=ray2(1);
    double z2=ray2(2);
    double dot = x1*x2 + y1*y2 + z1*z2;
    double lenSq1 = x1*x1 + y1*y1 + z1*z1;
    double lenSq2 = x2*x2 + y2*y2 + z2*z2;
    double angle = acos(dot/sqrt(lenSq1 * lenSq2));
    return angle;
}

float CalAngleDiff(Eigen::Matrix3d dir1, Eigen::Matrix3d dir2){
    Eigen::Vector3d ray1 = getPointOnSphere(dir1);
    Eigen::Vector3d ray2 = getPointOnSphere(dir2);
    return CalRayAngle(ray1, ray2);
}

void StitchAlgo::FindNearRay(Eigen::Matrix3d cur_dir, std::vector<PANO::KeyFrame*>& candi_list, std::vector<PANO::KeyFrame*>& overlay_list){
    bool is_overlay=false;
    for(int i=0; i<pano_map->frames.size(); i++){
        float angle_diff = CalAngleDiff(cur_dir, pano_map->frames[i]->direction);
//         std::cout<<"radian2degree(angle_diff): "<<radian2degree(angle_diff)<<std::endl;
        if(radian2degree(angle_diff) <25 && radian2degree(angle_diff) >10){
            candi_list.push_back(pano_map->frames[i]);
        }
        if(radian2degree(angle_diff) <=10){
            candi_list.clear();
            break;
        }
//         if(radian2degree(angle_diff) <10){
//             overlay_list.push_back(pano_map->frames[i]);
//         }
    }
}

Eigen::Matrix<double,3,3> toMatrix3d(const cv::Mat &cvMat3)
{
    Eigen::Matrix<double,3,3> M;

    M << cvMat3.at<float>(0,0), cvMat3.at<float>(0,1), cvMat3.at<float>(0,2),
         cvMat3.at<float>(1,0), cvMat3.at<float>(1,1), cvMat3.at<float>(1,2),
         cvMat3.at<float>(2,0), cvMat3.at<float>(2,1), cvMat3.at<float>(2,2);

    return M;
}

void StitchAlgo::DoOptimize(){
    if(pano_map->rot_graph.size()>0){
        BundleAdjustment(pano_map);
    }
}

Mat rot2euler(const Mat & rotationMatrix)
{
    Mat euler(3, 1, CV_64F);

    double m00 = rotationMatrix.at<double>(0, 0);
    double m02 = rotationMatrix.at<double>(0, 2);
    double m10 = rotationMatrix.at<double>(1, 0);
    double m11 = rotationMatrix.at<double>(1, 1);
    double m12 = rotationMatrix.at<double>(1, 2);
    double m20 = rotationMatrix.at<double>(2, 0);
    double m22 = rotationMatrix.at<double>(2, 2);

    double x, y, z;

    // Assuming the angles are in radians.
    if (m10 > 0.998) { // singularity at north pole
        x = 0;
        y = CV_PI / 2;
        z = atan2(m02, m22);
    }
    else if (m10 < -0.998) { // singularity at south pole
        x = 0;
        y = -CV_PI / 2;
        z = atan2(m02, m22);
    }
    else
    {
        x = atan2(-m12, m11);
        y = asin(m10);
        z = atan2(-m20, m00);
    }

    euler.at<double>(0) = x;
    euler.at<double>(1) = y;
    euler.at<double>(2) = z;

    return euler;
}

cv::Mat StitchAlgo::FinalImg(){
    using namespace std;
    using namespace cv;
    using namespace chamo::panod;
    using namespace chamo;
    std::vector<PANO::KeyFrame*> temp_framelist;
    for (auto it=frame_pool.begin(); it!=frame_pool.end(); ++it){
        temp_framelist.push_back(it->second);
    }
    auto time1 = std::chrono::steady_clock::now();
    int num_images=temp_framelist.size();
    cv::Ptr<cv::Feature2D> finder=cv::xfeatures2d::SURF::create();
    vector<chamo::panod::ImageFeatures> features(num_images);
    vector<Mat> images(num_images);
    vector<string> img_names;
    vector<Size> full_img_sizes(num_images);
    Mat full_img, img;
    double work_scale = 1, seam_scale = 1, compose_scale = 1;
    bool is_work_scale_set = false, is_seam_scale_set = false, is_compose_scale_set = false;
    double work_megapix=-1;
    double seam_megapix=-1;
    float match_conf = 0.65f;
    double seam_work_aspect = 1;
    float conf_thresh = 1.f;
    int expos_comp_nr_feeds = 1;
    int expos_comp_nr_filtering = 2;
    int expos_comp_block_size = 256;
    double compose_megapix = -1;
    float blend_strength = 5;
    for (int i = 0; i < num_images; ++i){
        std::stringstream ss;
        ss<<i;
        img_names.push_back(ss.str());
        full_img = temp_framelist[i]->color_img;
        full_img_sizes[i] = full_img.size();
        if (work_megapix < 0){
            img = full_img;
            work_scale = 1;
            is_work_scale_set = true;
        }else{
            if (!is_work_scale_set){
                work_scale = min(1.0, sqrt(work_megapix * 1e6 / full_img.size().area()));
                is_work_scale_set = true;
            }
            resize(full_img, img, Size(), work_scale, work_scale, INTER_LINEAR_EXACT);
        }
        if (!is_seam_scale_set){
            seam_scale = min(1.0, sqrt(seam_megapix * 1e6 / full_img.size().area()));
            seam_work_aspect = seam_scale / work_scale;
            is_seam_scale_set = true;
        }
        computeImageFeatures(finder, img, features[i]);
        features[i].img_idx = i;
        std::cout<<"Features in image #" << i+1 << ": " << features[i].keypoints.size()<<std::endl;
        resize(full_img, img, Size(), seam_scale, seam_scale, INTER_LINEAR_EXACT);
        cv::cvtColor(img, images[i], cv::COLOR_BGRA2BGR);
        //images[i] = img.clone();
    }
    full_img.release();
    img.release();
    std::cout<<"Pairwise matching"<<std::endl;
    vector<chamo::panod::MatchesInfo> pairwise_matches;
    Ptr<chamo::panod::FeaturesMatcher> matcher;
    matcher = makePtr<BestOf2NearestMatcher>(false, match_conf);
    (*matcher)(features, pairwise_matches);
    matcher->collectGarbage();
    // Leave only images we are sure are from the same panorama
    vector<int> indices = leaveBiggestComponent(features, pairwise_matches, conf_thresh);
    vector<Mat> img_subset;
    vector<String> img_names_subset;
    vector<Size> full_img_sizes_subset;
    std::vector<PANO::KeyFrame*> frame_subset;
    for (size_t i = 0; i < indices.size(); ++i){
        img_names_subset.push_back(img_names[indices[i]]);
        img_subset.push_back(images[indices[i]]);
        full_img_sizes_subset.push_back(full_img_sizes[indices[i]]);
        frame_subset.push_back(temp_framelist[indices[i]]);
    }

    images = img_subset;
    img_names = img_names_subset;
    full_img_sizes = full_img_sizes_subset;
    // Check if we still have enough images
    num_images = static_cast<int>(img_names.size());
    if (num_images < 2){
        std::cout<<"Need more images"<<std::endl;;
        return cv::Mat();
    }
    Ptr<Estimator> estimator;
    estimator = makePtr<HomographyBasedEstimator>();
    vector<CameraParams> cameras;
    for(int i=0; i<features.size(); i++){
        CameraParams cam_conf;
        cam_conf.focal=PANO::KeyFrame::fx;
        cam_conf.ppx=PANO::KeyFrame::cx;
        cam_conf.ppy=PANO::KeyFrame::cy;
        Eigen::Matrix3d& m=temp_framelist[i]->direction;
        cv::Mat mc(3,3,CV_32F);
        mc.at<float>(0,0)=m(0,0);
        mc.at<float>(0,1)=m(0,1);
        mc.at<float>(0,2)=m(0,2);
        mc.at<float>(1,0)=m(1,0);
        mc.at<float>(1,1)=m(1,1);
        mc.at<float>(1,2)=m(1,2);
        mc.at<float>(2,0)=m(2,0);
        mc.at<float>(2,1)=m(2,1);
        mc.at<float>(2,2)=m(2,2);
        cam_conf.R=mc;
        std::cout<<rot2euler(cam_conf.R).t()<<std::endl;
        cameras.push_back(cam_conf);
    }
//    if (!(*estimator)(features, pairwise_matches, cameras)){
//        cout << "Homography estimation failed.\n";
//        return;
//    }
    for (size_t i = 0; i < cameras.size(); ++i){
        Mat R;
        cameras[i].R.convertTo(R, CV_32F);
        cameras[i].R = R;
    }
    Ptr<chamo::panod::BundleAdjusterBase> adjuster;
    adjuster = makePtr<chamo::panod::BundleAdjusterRay>();
    adjuster->setConfThresh(conf_thresh);
    string ba_refine_mask = "_____";
    Mat_<uchar> refine_mask = Mat::zeros(3, 3, CV_8U);
    if (ba_refine_mask[0] == 'x') refine_mask(0,0) = 1;
    if (ba_refine_mask[1] == 'x') refine_mask(0,1) = 1;
    if (ba_refine_mask[2] == 'x') refine_mask(0,2) = 1;
    if (ba_refine_mask[3] == 'x') refine_mask(1,1) = 1;
    if (ba_refine_mask[4] == 'x') refine_mask(1,2) = 1;
    adjuster->setRefinementMask(refine_mask);
    if (!(*adjuster)(features, pairwise_matches, cameras)){
        cout << "Camera parameters adjusting failed.\n";
        return cv::Mat();
    }
    vector<double> focals;
    for (size_t i = 0; i < cameras.size(); ++i){
        focals.push_back(cameras[i].focal);
        std::cout<<rot2euler(cameras[i].R).t()<<std::endl;
    }
    sort(focals.begin(), focals.end());
    float warped_image_scale;
    if (focals.size() % 2 == 1)
        warped_image_scale = static_cast<float>(focals[focals.size() / 2]);
    else
        warped_image_scale = static_cast<float>(focals[focals.size() / 2 - 1] + focals[focals.size() / 2]) * 0.5f;
//    if (do_wave_correct)
//    {
//        vector<Mat> rmats;
//        for (size_t i = 0; i < cameras.size(); ++i)
//            rmats.push_back(cameras[i].R.clone());
//        waveCorrect(rmats, wave_correct);
//        for (size_t i = 0; i < cameras.size(); ++i)
//            cameras[i].R = rmats[i];
//    }
    std::cout<<"Warping images (auxiliary)... "<<std::endl;
    vector<Point> corners(num_images);
    vector<UMat> masks_warped(num_images);
    vector<UMat> images_warped(num_images);
    vector<Size> sizes(num_images);
    vector<UMat> masks(num_images);
    // Prepare images masks
    for (int i = 0; i < num_images; ++i){
        masks[i].create(images[i].size(), CV_8U);
        masks[i].setTo(Scalar::all(255));
    }
    Ptr<WarperCreator> warper_creator;
    warper_creator = makePtr<chamo::SphericalWarper>();
    if (!warper_creator){
        cout << "Can't create the following warper \n";
        return cv::Mat();
    }

    Ptr<RotationWarper> warper = warper_creator->create(static_cast<float>(warped_image_scale * seam_work_aspect));
    for (int i = 0; i < num_images; ++i){
        Mat_<float> K;
        cameras[i].K().convertTo(K, CV_32F);
        float swa = (float)seam_work_aspect;
        K(0,0) *= swa; K(0,2) *= swa;
        K(1,1) *= swa; K(1,2) *= swa;
        corners[i] = warper->warp(images[i], K, cameras[i].R, INTER_LINEAR, BORDER_REFLECT, images_warped[i]);
        sizes[i] = images_warped[i].size();
        warper->warp(masks[i], K, cameras[i].R, INTER_NEAREST, BORDER_CONSTANT, masks_warped[i]);
    }
    vector<UMat> images_warped_f(num_images);
    for (int i = 0; i < num_images; ++i)
        images_warped[i].convertTo(images_warped_f[i], CV_32F);
    std::cout<<"Compensating exposure..."<<std::endl;
    Ptr<ExposureCompensator> compensator = ExposureCompensator::createDefault(ExposureCompensator::CHANNELS_BLOCKS);
    if (dynamic_cast<GainCompensator*>(compensator.get()))
    {
        GainCompensator* gcompensator = dynamic_cast<GainCompensator*>(compensator.get());
        gcompensator->setNrFeeds(expos_comp_nr_feeds);
    }

    if (dynamic_cast<ChannelsCompensator*>(compensator.get()))
    {
        ChannelsCompensator* ccompensator = dynamic_cast<ChannelsCompensator*>(compensator.get());
        ccompensator->setNrFeeds(expos_comp_nr_feeds);
    }

    if (dynamic_cast<BlocksCompensator*>(compensator.get()))
    {
        BlocksCompensator* bcompensator = dynamic_cast<BlocksCompensator*>(compensator.get());
        bcompensator->setNrFeeds(expos_comp_nr_feeds);
        bcompensator->setNrGainsFilteringIterations(expos_comp_nr_filtering);
        bcompensator->setBlockSize(expos_comp_block_size, expos_comp_block_size);
    }

    compensator->feed(corners, images_warped, masks_warped);
    std::cout<<"Finding seams..."<<std::endl;
    Ptr<SeamFinder> seam_finder;
    seam_finder = makePtr<chamo::panod::DpSeamFinder>(DpSeamFinder::COLOR_GRAD);
    seam_finder->find(images_warped_f, corners, masks_warped);
    // Release unused memory
    //images.clear();
    images_warped.clear();
    images_warped_f.clear();
    masks.clear();
    std::cout<<"Compositing..."<<std::endl;
    Mat img_warped, img_warped_s;
    Mat dilated_mask, seam_mask, mask, mask_warped;
    Ptr<Blender> blender;
    double compose_work_aspect = 1;
    Rect final_rect;
    std::vector<Rect> rois;
    for (int img_idx = 0; img_idx < num_images; ++img_idx){
        std::cout<<"Compositing image #" << indices[img_idx]+1<<std::endl;
        // Read image and resize it if necessary
        full_img = images[img_idx]; // to-do
        cv::cvtColor(images[img_idx], full_img, cv::COLOR_BGRA2BGR);
        if (!is_compose_scale_set)
        {
            if (compose_megapix > 0)
                compose_scale = min(1.0, sqrt(compose_megapix * 1e6 / full_img.size().area()));
            is_compose_scale_set = true;

            // Compute relative scales
            //compose_seam_aspect = compose_scale / seam_scale;
            compose_work_aspect = compose_scale / work_scale;

            // Update warped image scale
            warped_image_scale *= static_cast<float>(compose_work_aspect);
            warper = warper_creator->create(warped_image_scale);
            std::cout<<"warped_image_scale: "<<warped_image_scale<<std::endl;

            // Update corners and sizes
            for (int i = 0; i < num_images; ++i)
            {
                // Update intrinsics
                cameras[i].focal *= compose_work_aspect;
                cameras[i].ppx *= compose_work_aspect;
                cameras[i].ppy *= compose_work_aspect;

                // Update corner and size
                Size sz = full_img_sizes[i];
                if (std::abs(compose_scale - 1) > 1e-1)
                {
                    sz.width = cvRound(full_img_sizes[i].width * compose_scale);
                    sz.height = cvRound(full_img_sizes[i].height * compose_scale);
                }

                Mat K;
                cameras[i].K().convertTo(K, CV_32F);
                Rect roi = warper->warpRoi(sz, K, cameras[i].R);
                corners[i] = roi.tl();
                sizes[i] = roi.size();
                rois.push_back(roi);
            }
        }
        if (abs(compose_scale - 1) > 1e-1)
            resize(full_img, img, Size(), compose_scale, compose_scale, INTER_LINEAR_EXACT);
        else
            img = full_img;
        full_img.release();
        Size img_size = img.size();

        Mat K;
        cameras[img_idx].K().convertTo(K, CV_32F);

        // Warp the current image
        warper->warp(img, K, cameras[img_idx].R, INTER_LINEAR, BORDER_REFLECT, img_warped);

        // Warp the current image mask
        mask.create(img_size, CV_8U);
        mask.setTo(Scalar::all(255));
        warper->warp(mask, K, cameras[img_idx].R, INTER_NEAREST, BORDER_CONSTANT, mask_warped);

        // Compensate exposure
        compensator->apply(img_idx, corners[img_idx], img_warped, mask_warped);

        img_warped.convertTo(img_warped_s, CV_16S);
        img_warped.release();
        img.release();
        mask.release();

        dilate(masks_warped[img_idx], dilated_mask, Mat());
        resize(dilated_mask, seam_mask, mask_warped.size(), 0, 0, INTER_LINEAR_EXACT);
        mask_warped = seam_mask & mask_warped;

        if (!blender)
        {
            int blend_type = Blender::FEATHER;
            blender = Blender::createDefault(blend_type, false);
            final_rect = resultRoi(corners, sizes);
            Size dst_sz = final_rect.size();
            vector<Point> corners_t;
            vector<Size> sizes_t;
            corners_t.push_back(Point(0,0));
            sizes_t.push_back(Size(2*CV_PI*warped_image_scale, CV_PI*warped_image_scale));
            float blend_width = sqrt(static_cast<float>(dst_sz.area())) * blend_strength / 100.f;
            if (blend_width < 1.f){
                blender = Blender::createDefault(Blender::NO, false);
                std::cout<<"Use default blender."<<std::endl;
            }
            else if (blend_type == Blender::MULTI_BAND)
            {
                MultiBandBlender* mb = dynamic_cast<MultiBandBlender*>(blender.get());
                mb->setNumBands(static_cast<int>(ceil(log(blend_width)/log(2.)) - 1.));
                std::cout<<"Multi-band blender, number of bands: " << mb->numBands()<<std::endl;
            }
            else if (blend_type == Blender::FEATHER)
            {
                FeatherBlender* fb = dynamic_cast<FeatherBlender*>(blender.get());
                fb->setSharpness(0.1f);
                std::cout<<"Feather blender, sharpness: " << fb->sharpness()<<std::endl;
                
            }
            
            blender->prepare(corners_t, sizes_t);
        }
        blender->feed(img_warped_s, mask_warped, corners[img_idx]);
    }
    Mat result, result_mask;
    blender->blend(result, result_mask);
    std::cout<<result.cols<<" : "<<result.rows<<std::endl;
    if(result.type()==CV_16SC3){
        std::cout<<"CV_16SC3"<<std::endl;
    }
    result.convertTo(result, CV_8UC3);
    resize(result, result, Size(3600, 1800));
    cv::cvtColor(result, sphere_img, cv::COLOR_BGR2BGRA);
    std::vector<cv::Point2f> point_src;
    std::vector<cv::Point2f> point_dst;
    for(int i=0; i<rois.size(); i++){
        float x =(rois[i].tl().x+rois[i].br().x)/2.0/warped_image_scale*10*180/CV_PI;
        float y =(rois[i].tl().y+rois[i].br().y)/2.0/warped_image_scale*10*180/CV_PI;
        point_src.push_back(cv::Point2f(x,y));
        Eigen::Vector3d pt_sphere = getPointOnSphere(frame_subset[i]->direction);
        float min_dist=9999;
        float min_id=-1;
        for(int j=0; j<sphere_pts.size();j++){
            float dist = (sphere_pts[j]-pt_sphere).norm();
            if(min_dist>dist){
                min_dist=dist;
                min_id=j;
            }
        }
        point_dst.push_back(cv::Point2f(sphere_pts_uv[min_id].x(), sphere_pts_uv[min_id].y()));
        std::cout<<point_src.back().x<<" : "<<point_src.back().y<<std::endl;
        std::cout<<point_dst.back().x<<" : "<<point_dst.back().y<<std::endl;
    }
//    cv::Mat warp_mat = cv::estimateRigidTransform( point_src, point_dst, false);
//    std::cout<<warp_mat<<std::endl;
//    for(int i=0;i<point_src.size(); i++){
//        cv::Mat pt(3,1,CV_64FC1);
//        pt.at<double>(0,0)=point_src[i].x;
//        pt.at<double>(1,0)=point_src[i].y;
//        pt.at<double>(2,0)=1;
//        cv::Mat warped_pt = warp_mat*pt;
//        std::cout<<warped_pt.t()<<std::endl;
//
//    }
//    warpAffine( result, sphere_img, warp_mat, sphere_img.size());
//    cv::flip(sphere_img, sphere_img, -1);
    auto time2 = std::chrono::steady_clock::now();
    auto diff = time2 - time1;
    std::cout << std::chrono::duration <double, std::milli> (diff).count() << " ms" << std::endl;
    return sphere_img;
}

void StitchAlgo::CalSphereSurfaceByFrame(PANO::KeyFrame* frame){
    for(int i=0; i<sphere_pts.size(); i++){
        int sphere_yaw=sphere_pts_uv[i].x();
        int sphere_pitch=sphere_pts_uv[i].y();
        Eigen::Vector3d posi_l = frame->direction.transpose()*sphere_pts[i];
        //std::cout<<sphere_pts[i].transpose()<<std::endl;
        if(posi_l.z()<0){
            continue;
        }
        double u=fx*posi_l.x()/posi_l.z()+cx;
        double v=fy*posi_l.y()/posi_l.z()+cy;
        //std::cout<<v<<" : "<<u<<std::endl;
        if(u>=0 && u<width && v>=0 && v<height){

            if(sphere_yaw<0 || sphere_yaw>=3600){
                continue;
            }
            if(sphere_pitch<0 || sphere_pitch>=1800){
                continue;
            }
            
            sphere_img.at<cv::Vec4b>(sphere_pitch, sphere_yaw)=frame->color_img.at<cv::Vec4b>(v,u);
        }
    }
}

bool detect_blur(DataItem data){
    //std::cout<<data.rot.norm()<<" : "<<data.acc.norm()<<std::endl;
    if(data.rot.norm()<0.1 && data.acc.norm()<0.1){
        return false;
    }else{
        return true;
    }
}

int StitchAlgo::check_in_slot(Eigen::Matrix3d dir){
    Eigen::Vector3d pt_sphere = getPointOnSphere(dir);
    for(int i=0; i<slot_posis.size(); i++){
        if((pt_sphere-slot_posis[i]).norm()<0.03){
            return i;
        }
    }
    return -1;
    
}

void StitchAlgo::AddData(cv::Mat img, Eigen::Matrix3d dir){
    PANO::KeyFrame* frame = CreateNewFrame(img, dir);
    frame_pool[image_count]=frame;
    image_count++;
}

void StitchAlgo::AddImageSimple(cv::Mat img, double timestamp){
    latest_img=img.clone();
    ImageItem imgitem;
    imgitem.img=latest_img;
    imgitem.time= timestamp;
    raw_imgs.push_back(imgitem);
    align_img();
    while(!raw_data.empty()){
        if(is_paint==true){
            auto time_now = std::chrono::system_clock::now();
            std::chrono::duration<double> elapsed_seconds = time_now-last_update_time;
            if(elapsed_seconds.count()>2){
                DataItem data = raw_data.back();
                if(!detect_blur(data)){
                    Eigen::Matrix3d R_w_b(data.dir);
                    Eigen::Matrix3d R_p_c=R_pano_w*R_w_b*R_b_c;
                    int slot_id= check_in_slot(R_p_c);
                    if(slot_id!=-1){
                        std::map<int, PANO::KeyFrame*>::iterator it=frame_pool.find(slot_id);
                        if(it!=frame_pool.end()){
                            delete it->second;
                            it->second=NULL;
                        }
                        cv::Mat imgUndistort;
                        cv::undistort(data.img, imgUndistort, K, DistCoef);
                        
                        PANO::KeyFrame* frame = CreateNewFrame(imgUndistort, R_p_c);
                        frame_pool[slot_id]=frame;
                        CalSphereSurfaceByFrame(frame);
                        DrawMarder();
                        last_update_time=std::chrono::system_clock::now();
                    }
                }
            }
        }
        raw_data.pop();
    }
}

void StitchAlgo::AddRot(Eigen::Quaterniond rot_angle,Eigen::Vector3d rot_speed, Eigen::Vector3d acc, double timestamp){
    Eigen::Matrix3d R_w_b(rot_angle);
    Eigen::Matrix3d R_p_c=R_pano_w*R_w_b*R_b_c;
    cam_dir=R_p_c;
    RotItem rotitem;
    rotitem.dir=rot_angle;
    rotitem.rot=rot_speed;
    rotitem.acc=acc;
    rotitem.time= timestamp;
    raw_rots.push_back(rotitem);
}

void StitchAlgo::align_img(){
    int last_rot_id=-1;
    int last_img_id=-1;
    int img_size=raw_imgs.size();
    int rot_size=raw_rots.size();
    //std::cout<<raw_imgs.size()<<std::endl;
    for(int i=0;i<img_size;i++){
        for(int j=0;j<rot_size-1;j++){
            if(raw_imgs[i].time>raw_rots[j].time && raw_imgs[i].time<=raw_rots[j+1].time){
                DataItem data;
                data.img=raw_imgs[i].img;
                data.dir = raw_rots[j].dir.slerp((raw_imgs[i].time-raw_rots[j].time)/(raw_rots[j+1].time-raw_rots[j].time), raw_rots[j+1].dir);
                data.timestamp=raw_imgs[i].time;
                if(fabs(raw_imgs[i].time-raw_rots[j].time)>fabs(raw_imgs[i].time-raw_rots[j+1].time)){
                    data.rot=raw_rots[j+1].rot;
                    data.acc=raw_rots[j+1].acc;
                }else{
                    data.rot=raw_rots[j].rot;
                    data.acc=raw_rots[j].acc;
                }
                raw_data.push(data);
                last_rot_id=j;
                last_img_id=i;
                break;
            }
        }
    }
    if(last_rot_id>0){
        if(last_rot_id-1<raw_rots.size()){
            raw_rots.erase(raw_rots.begin(), raw_rots.begin()+last_rot_id+1);
        }else{
            std::cout<<"test overflow"<<std::endl;
        }
    }
    if(last_img_id>=0){
        if(last_img_id<raw_imgs.size()){
            raw_imgs.erase(raw_imgs.begin(), raw_imgs.begin()+last_img_id+1);
        }else{
            std::cout<<"test overflow"<<std::endl;
        }
    }
    
}
void StitchAlgo::update(){
    while(!raw_data.empty()){
        DataItem data=raw_data.front();
        raw_data.pop();
    }
}

Eigen::Vector3f getRGB(int c){
    int b=c>>24;
    int g=c>>16;
    int r=c>>8;
    Eigen::Vector3f re_color(r/255.0, g/255.0, b/255.0);
    return re_color;
}
//int getColorInt(Eigen::Vector3f c){
//    int b=c>>24;
//    int g=c>>16;
//    int r=c>>8;
//    Eigen::Vector3f re_color(r/255.0, g/255.0, b/255.0);
//    return re_color;
//}

cv::Mat StitchAlgo::GetParoImage(){
    return sphere_img;
}

cv::Mat StitchAlgo::GetRawImage(){
    return latest_img;
}

void StitchAlgo::ClearImgSphere(){
    sphere_img=cv::Mat(1800, 3600, CV_8UC4);
    sphere_img.setTo(cv::Vec4b(255,255,255,255));
    DrawMarder();
}

void StitchAlgo::DrawMarder(){
    for(int k=0; k<slot_uvs.size(); k++){
        bool used_slot=false;
        for (auto it=frame_pool.begin(); it!=frame_pool.end(); ++it){
            if((slot_posis[k]-getPointOnSphere(it->second->direction)).norm()<0.1 ){
                used_slot=true;
            }
        }
        if(!used_slot){
            cv::circle(sphere_img, cv::Point(slot_uvs[k].x(), slot_uvs[k].y()), 30, cv::Scalar(0,0,255,255), 3);
        }
    }
}

void StitchAlgo::Reset(){
    frame_pool.clear();
    ClearImgSphere();
}
    


