#include "stitch_algo.h"
#include "KeyFrame.h"
#include "PanoMap.h"
#include "Optimizer.h"
#include "Initializer.h"
#include "ORBmatcher.h"
#include "ORBextractor.h"
#include <cmath>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
#include <rosbag/view.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <time.h> 
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

//int color2int(cv::Scalar color){
//    int c;
//    //c=color.r<<24+color.g<<24
//}
StitchAlgo::StitchAlgo(){
    R_b_c<<0.00007493, -0.99962055,  0.02754549, -0.99994504, -0.00036369, -0.01047809, 0.01048413, -0.02754319, -0.99956563;
    R_unity_w<<1, 0,  0, 0, 0, 1, 0, -1, 0;
    is_paint=false;
    for (float j = -90; j < 90; j = j + 0.1)
    {
        for (float i = 0; i < 360; i = i + 0.1) {
            Eigen::Vector3d posi;
            posi.x() = cos(degree2radian(j)) * cos(degree2radian(i));
            posi.y() = cos(degree2radian(j)) * sin(degree2radian(i));
            posi.z() = sin(degree2radian(j));
            sphere_pts.push_back(posi);
            Eigen::Vector2i uv;
            uv.x()=i*10;
            uv.y()=(j+90)*10;
            sphere_pts_uv.push_back(uv);
        }
    }
    width=1280;
    height=720;
    fx=945;
    fy=945;
    cx=640;
    cy=340;
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

void StitchAlgo::AddNewFrame(PANO::KeyFrame *new_frame, bool& is_discard, std::vector<PANO::KeyFrame*>& candi_list, std::vector<PANO::KeyFrame*>& overlay_list){
    is_discard=true;
    if(pano_map->frames.size()==0){
        pano_map->frames.push_back(new_frame);
        is_discard=false;
        return;
    }
    
    PANO::Initializer init(*new_frame);
    std::cout<<"candi_list: "<<candi_list.size()<<std::endl;
    if(candi_list.size()==overlay_list.size()){
        candi_list.clear();
    }
    for(int i=0; i<candi_list.size(); i++){
        Eigen::Matrix3d rot_1_2;
        PANO::ORBmatcher matcher;
        matcher.debug_img_ref=new_frame->color_img;
        matcher.debug_img_cur=candi_list[i]->color_img;
        std::vector<cv::Point2f> vbPrevMatched;
        std::vector<int> vnMatches12;
        int match_count = matcher.SearchForInitialization(*new_frame, *candi_list[i], vnMatches12, 10);
        std::cout<<"match count: "<<match_count<<std::endl;
        int good_count =-1;
        if(match_count>20){
            cv::Mat R21;
            cv::Mat t21;
            std::vector<cv::Point3f> vP3D;
            std::vector<bool> vbTriangulated;
            good_count = init.Initialize(*candi_list[i], vnMatches12, R21, t21, vP3D, vbTriangulated);
            std::cout<<"match final: "<<good_count<<std::endl;
            if(good_count>0){
                rot_1_2=toMatrix3d(R21.t());
                PANO::GraphNode node;
                node.frame1=new_frame;
                node.frame2=candi_list[i];
                node.rot_1_2=rot_1_2;
                node.weight=good_count;
                pano_map->rot_graph.push_back(node);
                is_discard=false;
            }
        }
    }
    if(is_discard==false){
        pano_map->frames.push_back(new_frame);
        if(overlay_list.size()>0){
            std::map<PANO::KeyFrame*, int> frame_score;
            for(int i=0; i<overlay_list.size(); i++){
                frame_score[overlay_list[i]]=0;
            }
            frame_score[new_frame]=0;
            for(int i=0; i<pano_map->rot_graph.size(); i++){
                std::map<PANO::KeyFrame*, int>::iterator it1=frame_score.find(pano_map->rot_graph[i].frame1);
                std::map<PANO::KeyFrame*, int>::iterator it2=frame_score.find(pano_map->rot_graph[i].frame2);
                std::map<PANO::KeyFrame*, int>::iterator it;
                for(it=frame_score.begin(); it!=frame_score.end(); it++){
                    it->second=it->second+pano_map->rot_graph[i].weight;
                }
                if(it1!=frame_score.end()){
                    it1->second=it1->second-pano_map->rot_graph[i].weight;
                }
                if(it2!=frame_score.end()){
                    it2->second=it2->second-pano_map->rot_graph[i].weight;
                }
            }
            std::map<PANO::KeyFrame*, int>::iterator it;
            int min_score=999999999;
            PANO::KeyFrame* min_frame=NULL;
            for(it=frame_score.begin(); it!=frame_score.end(); it++){
                //std::cout<<it->second<<",";
                if(it->second<min_score){
                    min_score=it->second;
                    min_frame=it->first;
                }
            }
            if(min_frame==NULL){
                return;
            }
            //std::cout<<"min_score: "<<min_score<<std::endl;
            for(int i=pano_map->rot_graph.size()-1; i>=0; i--){
                if(min_frame==pano_map->rot_graph[i].frame1 || min_frame==pano_map->rot_graph[i].frame2){
                    pano_map->rot_graph.erase(pano_map->rot_graph.begin()+i);
                }
            }
            for(int i=0; i<pano_map->frames.size(); i++){
                if(pano_map->frames[i]==min_frame){
                    pano_map->frames.erase(pano_map->frames.begin()+i);
                    break;
                }
            }
            if(min_frame==new_frame){
                is_discard=true;
            }
            delete min_frame;
            min_frame=NULL;
        }
    }else{
        delete new_frame;
    }
}

void StitchAlgo::DoOptimize(){
    if(pano_map->rot_graph.size()>0){
        BundleAdjustment(pano_map);
    }
}

void StitchAlgo::CalSphereSurface(Eigen::Matrix3d cur_dir){
    auto time1 = std::chrono::steady_clock::now();
    std::vector<PANO::KeyFrame*> candi_frames;
    Eigen::Vector3d cur_on_sphere= getPointOnSphere(cur_dir);
    for(int kk=0; kk<pano_map->frames.size(); kk++){
        Eigen::Vector3d cam_on_sphere= getPointOnSphere(pano_map->frames[kk]->direction);
        float angle_diff = CalRayAngle(cam_on_sphere, cur_on_sphere);
        if(abs(radian2degree(angle_diff))<30){
            candi_frames.push_back(pano_map->frames[kk]);
        }
    }
    for(int i=0; i<sphere_pts.size(); i++){
        int sphere_yaw=sphere_pts_uv[i].x();
        int sphere_pitch=sphere_pts_uv[i].y();
        if((sphere_pts[i]-cur_on_sphere).norm()>0.5){
            continue;
        }
        bool get_color=false;
        for(int kk=0; kk<candi_frames.size(); kk++)
        {
            Eigen::Vector3d posi_l = candi_frames[kk]->direction.transpose()*sphere_pts[i];
            double u=fx*posi_l.x()/posi_l.z()+cx;
            double v=fy*posi_l.y()/posi_l.z()+cy;
            //std::cout<<u<<" : "<<v<<std::endl;
            if(u>=width*0.2 && u<=width*0.8 && v>=height*0.2 && v<=height*0.8){
                if(sphere_yaw<0 || sphere_yaw>=3600){
                    continue;
                }
                if(sphere_pitch<0 || sphere_pitch>=1800){
                    continue;
                }
                get_color=true;
                sphere_img.at<cv::Vec4b>(sphere_pitch, sphere_yaw)=candi_frames[kk]->color_img.at<cv::Vec4b>(v,u);
            }
        }
        if(get_color==false){
            sphere_img.at<cv::Vec4b>(sphere_pitch, sphere_yaw)=cv::Vec4b(0,0,0,0);
        }
    }
    auto time2 = std::chrono::steady_clock::now();
    auto diff = time2 - time1;
    std::cout << std::chrono::duration <double, std::milli> (diff).count() << " ms" << std::endl;
}

bool detect_blur(DataItem data){
    //std::cout<<data.rot.norm()<<" : "<<data.acc.norm()<<std::endl;
    if(data.rot.norm()<0.1 && data.acc.norm()<0.1){
        return false;
    }else{
        return false;
    }
}

void StitchAlgo::AddImage(cv::Mat img, double timestamp){
    latest_img=img.clone();
    if(is_paint==false){
        ImageItem imgitem;
        imgitem.img=latest_img;
        imgitem.time= timestamp;
        raw_imgs.push_back(imgitem);
        align_img();
        while(!raw_data.empty()){
            raw_data.pop();
        }
        return;
    }
    std::cout<<"frame count: "<<pano_map->frames.size()<<std::endl;
    ImageItem imgitem;
    imgitem.img=latest_img;
    imgitem.time= timestamp;
    raw_imgs.push_back(imgitem);
    align_img();
    std::cout<<"raw_data: "<<raw_data.size()<<std::endl;
    while(!raw_data.empty()){
        DataItem data = raw_data.front();
        if(!detect_blur(data)){
            Eigen::Matrix3d R_w_b(data.dir);
            Eigen::Matrix3d R_w_c=R_w_b*R_b_c;
            std::vector<PANO::KeyFrame*> candi_list;
            std::vector<PANO::KeyFrame*> overlay_list;
            FindNearRay(R_w_c, candi_list, overlay_list);
            if(candi_list.size()>0 || pano_map->frames.size()==0){
                cv::Mat imgUndistort;
                cv::undistort(data.img, imgUndistort, K, DistCoef);
                PANO::KeyFrame* frame = CreateNewFrame(imgUndistort, R_w_c);
                bool is_discard=true;
                std::string discard_reason;
                AddNewFrame(frame, is_discard, candi_list,overlay_list);
                if(!is_discard){
                    DoOptimize();
                    //std::cout<<"frame count: "<<pano_map->frames.size()<<std::endl;
                    //ClearImgSphere();
                    CalSphereSurface(frame->direction);
                    cv::imshow("pano", sphere_img); 
                }
            }
        }
        
        
//            auto time2 = std::chrono::system_clock::now();
//            std::chrono::duration<double> elapsed_seconds = time2-time1;
//            std::cout<< "elapsed time: " << elapsed_seconds.count() <<std::endl;
        raw_data.pop();
    }
}
void StitchAlgo::AddRot(Eigen::Quaterniond rot_angle,Eigen::Vector3d rot_speed, Eigen::Vector3d acc, double timestamp){
    Eigen::Matrix3d R_w_b(rot_angle);
    Eigen::Matrix3d R_w_c=R_w_b*R_b_c;
    cam_dir=R_unity_w*R_w_c;
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
    sphere_img=cv::Mat::zeros(1800, 3600, CV_8UC4);
}
    


