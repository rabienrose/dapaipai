#import <UIKit/UIKit.h>
#import "BarrageRenderer.h"
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
@interface ViewController : UIViewController<BarrageRendererDelegate, AVCaptureVideoDataOutputSampleBufferDelegate, CLLocationManagerDelegate>{
    BarrageRenderer * _renderer;
    dispatch_queue_t sessionQueue;
    NSOperationQueue *quene;
    AVCaptureSession *session;
    AVCaptureDeviceInput *videoDeviceInput;
    AVCaptureDeviceDiscoverySession *videoDeviceDiscoverySession;
    AVCaptureDevice *videoDevice;
    AVCaptureVideoDataOutput *video_output;
    CMMotionManager *motionManager;
    struct DanmuTag{
    public:
        double lat;
        double lon;
        double alt;
        float rx;
        float ry;
        float rz;
        double time;
        int vis_id;
    };
    struct Danmu{
    public:
        DanmuTag tag;
        std::string text;
        int usr_id;
    };
    std::vector<Danmu> danmu_list;
    std::queue<int> danmu_waitlist;
    std::string danmu_file_addr;
    int send_danmu_count;
    double cur_lat;
    double cur_lon;
    double cur_dir;
    NSDate *last_update_time;
}
@property (nonatomic, strong) CLLocationManager *locationManager;
@property (weak, nonatomic) IBOutlet UIImageView *image_view;
@property (weak, nonatomic) IBOutlet UITextField *text_box;

@end


