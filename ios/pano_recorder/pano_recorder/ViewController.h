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

#include <ros/ros.h>
#include <rosbag/bag.h>

#import <UIKit/UIKit.h>

@interface ViewController : UIViewController
NSOperationQueue *quene;
AVCaptureSession *session;

@end

