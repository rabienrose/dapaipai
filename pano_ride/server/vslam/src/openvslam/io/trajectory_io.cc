#include "openvslam/data/keyframe.h"
#include "openvslam/data/landmark.h"
#include "openvslam/data/map_database.h"
#include "openvslam/io/trajectory_io.h"

#include <iostream>
#include <iomanip>

#include <spdlog/spdlog.h>
#include <nlohmann/json.hpp>

namespace openvslam {
namespace io {


bool WritePly(std::string filename, std::vector<Eigen::Vector3d> &v){
    std::ofstream plyFile;
    plyFile.open (filename.c_str(), std::ios::out |  std::ios::trunc | std::ios::binary);
    if (! plyFile.is_open()){
        std::cerr << "Cannot open file to write!" << std::endl;
        return false;
    }
//    bool useColors = false;
//    for (unsigned int i = 0; i!=v.size(); i++){
//        if (v[i].hasColor()){
//            useColors = true;
//            break;
//        }
//    }
    
    plyFile.imbue(std::locale::classic());
    
    // Write Header
    plyFile << "ply" << std::endl;
    plyFile << "format binary_little_endian 1.0" << std::endl;
    plyFile << "comment Super4PCS output file" << std::endl;
    plyFile << "element vertex " << v.size() << std::endl;
    plyFile << "property float x" << std::endl;
    plyFile << "property float y" << std::endl;
    plyFile << "property float z" << std::endl;
//    if(useColors) {
//        plyFile << "property uchar red" << std::endl;
//        plyFile << "property uchar green" << std::endl;
//        plyFile << "property uchar blue" << std::endl;
//    }
    
    plyFile << "end_header" << std::endl;
    
    // Read all elements in data, correct their depth and print them in the file
    char tmpChar;
    float tmpFloat;
    for (unsigned int i = 0; i!=v.size(); i++){
        tmpFloat = v[i].x();
        plyFile.write(reinterpret_cast<const char*>(&tmpFloat),sizeof(float));
        tmpFloat = v[i].y();
        plyFile.write(reinterpret_cast<const char*>(&tmpFloat),sizeof(float));
        tmpFloat = v[i].z();
        plyFile.write(reinterpret_cast<const char*>(&tmpFloat),sizeof(float));
        
//        if (useColors){
//            tmpChar = v[i].rgb()[0];
//            plyFile.write(reinterpret_cast<const char*>(&tmpChar),sizeof(char));
//            tmpChar = v[i].rgb()[1];
//            plyFile.write(reinterpret_cast<const char*>(&tmpChar),sizeof(char));
//            tmpChar = v[i].rgb()[2];
//            plyFile.write(reinterpret_cast<const char*>(&tmpChar),sizeof(char));
//        }
    }
    
    plyFile.close();
    
    return true;
}

trajectory_io::trajectory_io(data::map_database* map_db)
    : map_db_(map_db) {}

void trajectory_io::save_frame_trajectory(const std::string& path, const std::string& format) const {
    std::lock_guard<std::mutex> lock(data::map_database::mtx_database_);

    // 1. acquire the frame stats

    assert(map_db_);
    auto frm_stats = map_db_->get_frame_statistics();

    // 2. save the frames

    const auto num_valid_frms = frm_stats.get_num_valid_frames();
    const auto reference_keyframes = frm_stats.get_reference_keyframes();
    const auto rel_cam_poses_from_ref_keyfrms = frm_stats.get_relative_cam_poses();
    std::map<unsigned int, double> timestamps = frm_stats.get_timestamps();
    std::map<unsigned int, Mat44_t> frame_poses = frm_stats.get_frameposes();
    const auto is_lost_frms = frm_stats.get_lost_frames();
    std::ofstream ofs(path+"/frames.txt", std::ios::out);
    std::map<unsigned int, double>::iterator iter;
    for(iter=timestamps.begin(); iter!=timestamps.end(); iter++){
        unsigned int frame_id=iter->first;
        double timestamp=iter->second;
        Mat44_t cam_pose_wc=frame_poses[frame_id];
        ofs <<timestamp<<" "<< std::setprecision(9)
        << cam_pose_wc(0, 0) << " " << cam_pose_wc(0, 1) << " " << cam_pose_wc(0, 2) << " " << cam_pose_wc(0, 3) << " "
        << cam_pose_wc(1, 0) << " " << cam_pose_wc(1, 1) << " " << cam_pose_wc(1, 2) << " " << cam_pose_wc(1, 3) << " "
        << cam_pose_wc(2, 0) << " " << cam_pose_wc(2, 1) << " " << cam_pose_wc(2, 2) << " " << cam_pose_wc(2, 3) << " "<<frm_stats.frame_range[frame_id]<<std::endl;
    }
    ofs.close();
    std::ofstream ofs_mps(path+"/mps.txt", std::ios::out);
    std::vector<Eigen::Vector3d> pcs;
    std::vector<data::landmark*> ldmks = map_db_->get_all_landmarks();
    for (int i=0; i<ldmks.size(); i++){
        Vec3_t temp_posi = ldmks[i]->get_pos_in_world();
        pcs.push_back(ldmks[i]->get_pos_in_world());
        ofs_mps <<temp_posi(0)<<" "<<temp_posi(1)<<" "<<temp_posi(2)<<std::endl;
    }
    ofs_mps.close();
    WritePly(path+"/chamo.ply",pcs);
}

void trajectory_io::save_keyframe_trajectory(const std::string& path, const std::string& format) const {
    std::lock_guard<std::mutex> lock(data::map_database::mtx_database_);

    // 1. acquire keyframes and sort them

    assert(map_db_);
    auto keyfrms = map_db_->get_all_keyframes();
    std::sort(keyfrms.begin(), keyfrms.end(), [&](data::keyframe* keyfrm_1, data::keyframe* keyfrm_2) {
        return *keyfrm_1 < *keyfrm_2;
    });

    // 2. save the keyframes

    if (keyfrms.empty()) {
        spdlog::warn("there are no valid keyframes, cannot dump keyframe trajectory");
        return;
    }

    std::ofstream ofs(path+"/keyframes.txt", std::ios::out);

    spdlog::info("dump keyframe trajectory in \"{}\" format from keyframe {} to keyframe {} ({} keyframes)",
                 format, (*keyfrms.begin())->id_, (*keyfrms.rbegin())->id_, keyfrms.size());

    for (const auto keyfrm : keyfrms) {
        const Mat44_t cam_pose_cw = keyfrm->get_cam_pose();
        const Mat44_t cam_pose_wc = cam_pose_cw.inverse();
        const auto timestamp = keyfrm->timestamp_;

        if (format == "KITTI") {
            ofs <<keyfrm->timestamp_<<" "<< std::setprecision(9)
                << cam_pose_wc(0, 0) << " " << cam_pose_wc(0, 1) << " " << cam_pose_wc(0, 2) << " " << cam_pose_wc(0, 3) << " "
                << cam_pose_wc(1, 0) << " " << cam_pose_wc(1, 1) << " " << cam_pose_wc(1, 2) << " " << cam_pose_wc(1, 3) << " "
                << cam_pose_wc(2, 0) << " " << cam_pose_wc(2, 1) << " " << cam_pose_wc(2, 2) << " " << cam_pose_wc(2, 3) << std::endl;
        }
        else if (format == "TUM") {
            const Mat33_t& rot_wc = cam_pose_wc.block<3, 3>(0, 0);
            const Vec3_t& trans_wc = cam_pose_wc.block<3, 1>(0, 3);
            const Quat_t quat_wc = Quat_t(rot_wc);
            ofs << std::setprecision(15)
                << timestamp << " "
                << std::setprecision(9)
                << trans_wc(0) << " " << trans_wc(1) << " " << trans_wc(2) << " "
                << quat_wc.x() << " " << quat_wc.y() << " " << quat_wc.z() << " " << quat_wc.w() << std::endl;
        }
        else {
            throw std::runtime_error("Not implemented: trajectory format \"" + format + "\"");
        }
    }

    ofs.close();
}

} // namespace io
} // namespace openvslam
