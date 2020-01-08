#include "stitch_algo.h"
#import <UIKit/UIKit.h>
#import <QuartzCore/QuartzCore.h>
#import <CoreMotion/CoreMotion.h>
#import <CoreMedia/CoreMedia.h>
#import <AVFoundation/AVFoundation.h>
#include <deque>
#include <memory>
#include <vector>
#include <string>
#include <queue>
#import <CoreLocation/CoreLocation.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
extern "C" {
void start_paro();
void set_algo_status(bool go);
}
void get_raw(cv::Mat& raw_img);
void get_paro(cv::Mat& paro_img);


@interface ParoWorld : NSObject<AVCaptureVideoDataOutputSampleBufferDelegate, CLLocationManagerDelegate>{
    dispatch_queue_t sessionQueue;
    dispatch_queue_t processQueue;
    dispatch_queue_t saveQueue;
    NSOperationQueue *quene;
    AVCaptureSession *session;
    AVCaptureDeviceInput *videoDeviceInput;
    AVCaptureDeviceDiscoverySession *videoDeviceDiscoverySession;
    AVCaptureDevice *videoDevice;
    AVCaptureVideoDataOutput *video_output;
    CMMotionManager *motionManager;
    double cur_lat;
    double cur_lon;
    double cur_dir;
    NSDate *last_update_time;
    std::shared_ptr<rosbag::Bag> bag_ptr;
    bool is_recording_bag;
    int motion_data_id;
    bool algo_status;
}
- (cv::Mat) get_paroimg;
- (void) finalImg;
- (void) Reset;
- (void) switchAlgo;
@end

