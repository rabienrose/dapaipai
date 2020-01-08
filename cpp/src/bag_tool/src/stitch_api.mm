#import "stitch_api.h"
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include "cv_ios_bridge.h"
#import <CoreMotion/CoreMotion.h>

@interface ParoWorld ()

@end

@implementation ParoWorld
StitchAlgo algo;
- (UIImage *) imageFromSampleBuffer:(CMSampleBufferRef) sampleBuffer
{
    CVImageBufferRef imageBuffer = CMSampleBufferGetImageBuffer(sampleBuffer);
    CVPixelBufferLockBaseAddress(imageBuffer, 0);
    void *baseAddress = CVPixelBufferGetBaseAddress(imageBuffer);
    size_t bytesPerRow = CVPixelBufferGetBytesPerRow(imageBuffer);
    size_t width = CVPixelBufferGetWidth(imageBuffer);
    size_t height = CVPixelBufferGetHeight(imageBuffer);
    CGColorSpaceRef colorSpace = CGColorSpaceCreateDeviceRGB();
    CGContextRef context = CGBitmapContextCreate(baseAddress, width, height, 8,
                                                 bytesPerRow, colorSpace, kCGBitmapByteOrder32Little | kCGImageAlphaPremultipliedFirst);
    CGImageRef quartzImage = CGBitmapContextCreateImage(context);
    CVPixelBufferUnlockBaseAddress(imageBuffer,0);
    CGContextRelease(context);
    CGColorSpaceRelease(colorSpace);
    UIImage *image = [UIImage imageWithCGImage:quartzImage];
    CGImageRelease(quartzImage);
    return image;
}

- (void) toogleRecording{
    if(!is_recording_bag){
        dispatch_async( saveQueue, ^{
            NSArray *dirPaths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
            NSDate *date = [NSDate date];
            NSDateFormatter *formatter = [[NSDateFormatter alloc] init];
            [formatter setDateFormat:@"MM-dd-HH-mm-ss"];
            NSString *timeString = [formatter stringFromDate:date];
            NSString *string1 = [NSString stringWithFormat:@"%@.bag",timeString];
            NSString *full_addr = [[dirPaths objectAtIndex:0] stringByAppendingPathComponent:string1];
            char *docsPath;
            docsPath = (char*)[full_addr cStringUsingEncoding:[NSString defaultCStringEncoding]];
            std::string full_file_name(docsPath);
            std::cout<<full_file_name<<std::endl;
            bag_ptr.reset(new rosbag::Bag());
            bag_ptr->open(full_file_name.c_str(), rosbag::bagmode::Write);
            is_recording_bag=true;
        });
    }else{
        is_recording_bag=false;
        dispatch_async( saveQueue, ^{
            bag_ptr->close();
            NSLog(@"close the bag");
        });
    }
}

- (void)init_device{
    is_recording_bag=false;
    last_update_time=[NSDate date];
    sessionQueue = dispatch_queue_create( "session queue", DISPATCH_QUEUE_SERIAL );
    processQueue = dispatch_queue_create( "session queue", DISPATCH_QUEUE_SERIAL );
    saveQueue = dispatch_queue_create( "session queue", DISPATCH_QUEUE_SERIAL );
    
    quene =[[NSOperationQueue alloc] init];
    quene.maxConcurrentOperationCount=1;
    session = [[AVCaptureSession alloc] init];
    NSError *error = nil;
    [session beginConfiguration];
    session.sessionPreset =AVCaptureSessionPreset640x480;
    videoDevice = [AVCaptureDevice defaultDeviceWithDeviceType:AVCaptureDeviceTypeBuiltInWideAngleCamera mediaType:AVMediaTypeVideo position:AVCaptureDevicePositionUnspecified];
    AVCaptureDeviceInput *videoDeviceInput = [AVCaptureDeviceInput deviceInputWithDevice:videoDevice error:&error];
    if ( ! videoDeviceInput ) {
        NSLog( @"Could not create video device input: %@", error );
        [session commitConfiguration];
        return;
    }
    if ( [session canAddInput:videoDeviceInput] ) {
        [session addInput:videoDeviceInput];
        videoDeviceInput = videoDeviceInput;
    }
    else {
        NSLog( @"Could not add video device input to the session" );
        [session commitConfiguration];
        return;
    }
    [videoDevice setActiveVideoMinFrameDuration:CMTimeMake(1, 10)];
    video_output = [[AVCaptureVideoDataOutput alloc] init];
    NSDictionary *newSettings = @{ (NSString *)kCVPixelBufferPixelFormatTypeKey: @(kCVPixelFormatType_32BGRA)};
    video_output.videoSettings = newSettings;
    [video_output setAlwaysDiscardsLateVideoFrames:YES];
    if ([session canAddOutput:video_output]) {
        [session addOutput:video_output];
    }else {
        NSLog(@"add output wrong!!!");
    }

    [video_output setSampleBufferDelegate:self queue:sessionQueue];
    [session commitConfiguration];
//    if ( [videoDevice lockForConfiguration:&error] ) {
//        if ( [videoDevice isFocusModeSupported:AVCaptureFocusModeLocked] ) {
//            videoDevice.focusMode = AVCaptureFocusModeLocked;
//        }
//        [videoDevice setFocusModeLockedWithLensPosition:0.8 completionHandler:nil];
//        if([videoDevice isExposureModeSupported:AVCaptureExposureModeCustom]){
//            videoDevice.exposureMode=AVCaptureExposureModeCustom;
//        }
//        [videoDevice setExposureModeCustomWithDuration:CMTimeMakeWithSeconds( 0.01, 1000*1000*1000 ) ISO:100 completionHandler:nil];
//        [videoDevice unlockForConfiguration];
//    }else {
//        NSLog( @"Could not lock device for configuration: %@", error );
//    }
    [session startRunning];
    motion_data_id=0;
    motionManager = [[CMMotionManager alloc] init];
    if (motionManager.deviceMotionAvailable){
        motionManager.deviceMotionUpdateInterval =0.01;
        [motionManager startDeviceMotionUpdatesUsingReferenceFrame:CMAttitudeReferenceFrameXMagneticNorthZVertical
         toQueue: quene
         withHandler:
         ^(CMDeviceMotion *data, NSError *error){
            //std::cout<<data.attitude.yaw/3.1415926*360<<std::endl;
            if(is_recording_bag){
                motion_data_id++;
                dispatch_async( saveQueue, ^{
                    geometry_msgs::QuaternionStamped dir_msg;
                    dir_msg.header.stamp=ros::Time(data.timestamp);
                    dir_msg.header.seq=motion_data_id;
                    dir_msg.quaternion.w=data.attitude.quaternion.w;
                    dir_msg.quaternion.x=data.attitude.quaternion.x;
                    dir_msg.quaternion.y=data.attitude.quaternion.y;
                    dir_msg.quaternion.z=data.attitude.quaternion.z;
                    geometry_msgs::Vector3Stamped rot_msg;
                    rot_msg.header.stamp=ros::Time(data.timestamp);
                    rot_msg.header.seq=motion_data_id;
                    rot_msg.vector.x=data.rotationRate.x;
                    rot_msg.vector.y=data.rotationRate.y;
                    rot_msg.vector.z=data.rotationRate.z;
                    geometry_msgs::Vector3Stamped acc_msg;
                    acc_msg.header.stamp=ros::Time(data.timestamp);
                    acc_msg.header.seq=motion_data_id;
                    acc_msg.vector.x=data.userAcceleration.x;
                    acc_msg.vector.y=data.userAcceleration.y;
                    acc_msg.vector.z=data.userAcceleration.z;
                    
                    if(bag_ptr->isOpen()){
                        NSDate * t1 = [NSDate date];
                        NSTimeInterval now = [t1 timeIntervalSince1970];
                        bag_ptr->write("dir", ros::Time(now), dir_msg);
                        bag_ptr->write("rot", ros::Time(now), rot_msg);
                        bag_ptr->write("acc", ros::Time(now), acc_msg);
                    }
                });
            }else{
                dispatch_async( processQueue, ^{
                    Eigen::Quaterniond qua;
                    qua.x()=data.attitude.quaternion.x;
                    qua.y()=data.attitude.quaternion.y;
                    qua.z()=data.attitude.quaternion.z;
                    qua.w()=data.attitude.quaternion.w;
                    Eigen::Vector3d rot_eig;
                    rot_eig.x()=data.rotationRate.x;
                    rot_eig.y()=data.rotationRate.y;
                    rot_eig.z()=data.rotationRate.z;
                    Eigen::Vector3d acc_eig;
                    acc_eig.x()=data.userAcceleration.x;
                    acc_eig.y()=data.userAcceleration.y;
                    acc_eig.z()=data.userAcceleration.z;
                    algo.AddRot(qua, rot_eig, acc_eig, data.timestamp);
                });
            }
            
         }];
    }
    algo.ClearImgSphere();
    NSArray *dirPaths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    NSString *full_addr = [dirPaths objectAtIndex:0];
    char *docsPath;
    docsPath = (char*)[full_addr cStringUsingEncoding:[NSString defaultCStringEncoding]];
    std::string full_file_name(docsPath);
    algo.bag_root=full_file_name;
}

- (cv::Mat) get_paroimg{
    return algo.GetParoImage();
}

- (void) switchAlgo{
    dispatch_async( saveQueue, ^{
        algo.switchPaint();
    });
}

- (void) Reset{
    algo.Reset();
}

- (void)captureOutput:(AVCaptureOutput *)captureOutput didOutputSampleBuffer:(CMSampleBufferRef)sampleBuffer fromConnection:(AVCaptureConnection *)connection {
    CMTime timestamp = CMSampleBufferGetPresentationTimeStamp(sampleBuffer);
    double sync_sensor_time = (double)timestamp.value/(double)timestamp.timescale;
    UIImage *image = [self imageFromSampleBuffer:sampleBuffer];
    cv::Mat img_cv = [cv_ios_bridge cvMatFromUIImage:image];
    if(is_recording_bag){
        sensor_msgs::CompressedImage img_ros_img;
        std::vector<unsigned char> binaryBuffer_;
        cv::imencode(".jpg", img_cv, binaryBuffer_);
        img_ros_img.data=binaryBuffer_;
        img_ros_img.header.seq=1;
        img_ros_img.header.stamp= ros::Time(sync_sensor_time);
        img_ros_img.format="jpeg";
        dispatch_async( saveQueue, ^{
            if(bag_ptr->isOpen()){
                NSDate * t1 = [NSDate date];
                NSTimeInterval now = [t1 timeIntervalSince1970];
                bag_ptr->write("img", ros::Time(now), img_ros_img);
            }
        });
    }else{
        dispatch_async( processQueue, ^{
            algo.AddImageSimple(img_cv, sync_sensor_time);
        });
    }
        
//
//    img_count++;
}
ParoWorld *world;

extern "C" {
    void start_paro(){
        algo.ClearImgSphere();
        world = [ParoWorld alloc];
        [world init_device];
    }

    void get_paro(char *arr)
    {
        //std::cout<<(long long)arr<<std::endl;
        cv::Mat img= algo.GetParoImage();
        memcpy(arr, img.data, img.rows*img.cols*4);
        //cv::Mat img_out(img.rows, img.cols, img.type(), arr);
        //img.copyTo(img_out);
    }
    
    void get_dir(float *qua){
        qua[0]=algo.cam_dir.x();
        qua[1]=algo.cam_dir.y();
        qua[2]=algo.cam_dir.z();
        qua[3]=algo.cam_dir.w();
    }
    void set_algo_status(bool go){
        [world switchAlgo];
    }
    void do_final(){
        [world Reset];
    }
}
void get_paro(cv::Mat& paro_img){
    paro_img= algo.GetParoImage();
}

void get_raw(cv::Mat& raw_img){
    raw_img=algo.GetRawImage();
}


@end


