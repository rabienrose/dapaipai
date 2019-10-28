#import "ViewController.h"
#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#import <Foundation/Foundation.h>
@interface ViewController ()
    
@end

std::vector<std::string> split(const std::string& str, const std::string& delim)
{
    std::vector<std::string> tokens;
    size_t prev = 0, pos = 0;
    do
    {
        pos = str.find(delim, prev);
        if (pos == std::string::npos) pos = str.length();
        std::string token = str.substr(prev, pos-prev);
        if (!token.empty()) tokens.push_back(token);
        prev = pos + delim.length();
    }
    while (pos < str.length() && prev < str.length());
    return tokens;
}

void readDanmu(std::string filename, std::vector<Danmu>& danmu_list){
    std::string line;
    std::ifstream infile_danmu(filename.c_str());
    if(!infile_danmu.is_open()){
        return;
    }
    int danmu_count=0;
    while (true)
    {
        std::getline(infile_danmu, line);
        if (line==""){
            break;
        }
        std::vector<std::string> splited = split(line, ",");
        Danmu danmu;
        danmu.tag.lat=atof(splited[0].c_str());
        danmu.tag.lon=atof(splited[1].c_str());
        danmu.tag.alt=atof(splited[2].c_str());
        danmu.tag.rx=atof(splited[3].c_str());
        danmu.tag.ry=atof(splited[4].c_str());
        danmu.tag.rz=atof(splited[5].c_str());
        danmu.tag.time=atof(splited[6].c_str());
        danmu.tag.vis_id=atoi(splited[7].c_str());
        danmu.usr_id=atoi(splited[8].c_str());
        danmu.text=splited[9];
        danmu_list.push_back(danmu);
    }
    infile_danmu.close();
}

void appendDanmu(std::string filename, Danmu& new_danmu){
    std::string line;
    //outfile.open("test.txt", std::ios_base::app);
    std::ofstream outfile_danmu(filename.c_str(),std::ios_base::app);
    if(!outfile_danmu.is_open()){
        return;
    } outfile_danmu<<std::setprecision(15)<<new_danmu.tag.lat<<","<<new_danmu.tag.lon<<","<<new_danmu.tag.alt<<","<<new_danmu.tag.rx<<","<<new_danmu.tag.ry<<","<<new_danmu.tag.rz<<","<<new_danmu.tag.time<<","<<new_danmu.tag.vis_id<<","<<new_danmu.usr_id<<","<<new_danmu.text<<","<<std::endl;
    outfile_danmu.close();
}



@implementation ViewController
- (void)viewDidLoad {
    [super viewDidLoad];
    last_update_time=[NSDate date];
    _renderer = [[BarrageRenderer alloc]init];
    _renderer.smoothness = .2f;
    _renderer.delegate = self;
    [self.image_view addSubview:_renderer.view];
    _renderer.canvasMargin = UIEdgeInsetsMake(50, 10, 10, 10);
    // 若想为弹幕增加点击功能, 请添加此句话, 并在Descriptor中注入行为
    _renderer.view.userInteractionEnabled = NO;
    [self.image_view sendSubviewToBack:_renderer.view];
    
    sessionQueue = dispatch_queue_create( "session queue", DISPATCH_QUEUE_SERIAL );
    quene =[[NSOperationQueue alloc] init];
    quene.maxConcurrentOperationCount=1;
    _locationManager = [[CLLocationManager alloc] init];
    self.locationManager.delegate = self;
    self.locationManager.desiredAccuracy = kCLLocationAccuracyBest;
    self.locationManager.distanceFilter = kCLDistanceFilterNone;
    if ([_locationManager respondsToSelector:@selector(requestWhenInUseAuthorization)]) {
        [_locationManager requestWhenInUseAuthorization];
    }

    session = [[AVCaptureSession alloc] init];
    NSError *error = nil;
    [session beginConfiguration];
    session.sessionPreset =AVCaptureSessionPreset1280x720;
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
    [videoDevice setActiveVideoMinFrameDuration:CMTimeMake(1, 30)];
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
//if ( [videoDevice lockForConfiguration:&error] ) {
//    if ( [videoDevice isFocusModeSupported:AVCaptureFocusModeLocked] ) {
//        videoDevice.focusMode = AVCaptureFocusModeContinuousAutoFocus;
//    }
//    if([videoDevice isExposureModeSupported:AVCaptureExposureModeCustom]){
//        videoDevice.exposureMode=AVCaptureExposureModeCustom;
//    }
//    //[videoDevice setFocusModeLockedWithLensPosition:0.8 completionHandler:nil];
//    [videoDevice setExposureModeCustomWithDuration:CMTimeMakeWithSeconds( 0.01, 1000*1000*1000 ) ISO:AVCaptureISOCurrent completionHandler:nil];
//    [videoDevice unlockForConfiguration];
//}else {
//    NSLog( @"Could not lock device for configuration: %@", error );
//}

NSArray *dirPaths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
NSString *danmu_addr = [[dirPaths objectAtIndex:0] stringByAppendingPathComponent:@"damu.txt"];
danmu_file_addr = std::string([danmu_addr UTF8String]);
BOOL isDir;
NSFileManager *fileManager= [NSFileManager defaultManager];
//if(![fileManager fileExistsAtPath:danmu_addr isDirectory:&isDir]){
//
//}
readDanmu(danmu_file_addr, danmu_list);
[session startRunning];
[_locationManager startUpdatingLocation];
[_locationManager startUpdatingHeading];
[_renderer start];
send_danmu_count=0;
}

double DegToRad(const double deg) {
  return deg * 0.0174532925199432954743716805978692718781530857086181640625;
}

std::vector<double> EllToXYZ(std::vector<double> ell) {
    double a_ = 6378137;
    double b_ = 6.356752314245179e+06;
    double f_ = 0.003352810664747;
    double e2_ = (a_ * a_ - b_ * b_) / (a_ * a_);
    const double lat = DegToRad(ell[0]);
    const double lon = DegToRad(ell[1]);
    const double alt = ell[2];

    const double sin_lat = sin(lat);
    const double sin_lon = sin(lon);
    const double cos_lat = cos(lat);
    const double cos_lon = cos(lon);

    // Normalized radius
    const double N = a_ / sqrt(1 - e2_ * sin_lat * sin_lat);
    std::vector<double> xyz;
    xyz.resize(3);
    xyz[0] = (N + alt) * cos_lat * cos_lon;
    xyz[1] = (N + alt) * cos_lat * sin_lon;
    xyz[2] = (N * (1 - e2_) + alt) * sin_lat;

  return xyz;
}

-(void) checkDanmu{
    std::vector<double> cur_ell;
    cur_ell.resize(3);
    cur_ell[0]=cur_lat;
    cur_ell[1]=cur_lon;
    cur_ell[2]=0;
    std::vector<double> cur_xyz = EllToXYZ(cur_ell);
    std::vector<int> re;
    for(int i=0; i<danmu_list.size(); i++){
        std::vector<double> ell;
        ell.resize(3);
        ell[0]=danmu_list[i].tag.lat;
        ell[1]=danmu_list[i].tag.lon;
        ell[2]=0;
        std::vector<double> xyz = EllToXYZ(ell);
        double d_dist = sqrt((xyz[0]-cur_xyz[0])*(xyz[0]-cur_xyz[0])+(xyz[1]-cur_xyz[1])*(xyz[1]-cur_xyz[1])+(xyz[2]-cur_xyz[2])*(xyz[2]-cur_xyz[2]));
        double r_dist=fabs(cur_dir-danmu_list[i].tag.rx);
        if(d_dist<100 && r_dist<20){
            danmu_waitlist.push(i);
        }
    }
    while(!danmu_waitlist.empty()){
        std::cout<<danmu_list[danmu_waitlist.front()].text<<std::endl;
        NSString* text_ns = [NSString stringWithCString:danmu_list[danmu_waitlist.front()].text.c_str() encoding:NSUTF8StringEncoding];
        [self sent_danmu:text_ns];
        danmu_waitlist.pop();
    }
}

- (IBAction)send_btn:(id)sender {
    std::string danmu_text = std::string([self.text_box.text UTF8String]);
    Danmu danmu = [self create_danmu: danmu_text];
    //std::cout<<danmu.tag.lat<<","<<danmu.tag.lon<<","<<danmu.tag.rx<<","<<danmu.text<<std::endl;
    appendDanmu(danmu_file_addr, danmu);
    danmu_list.push_back(danmu);
    [self sent_danmu:self.text_box.text];
    self.text_box.text=@"";
}

-(void)touchesBegan:(NSSet *)touches withEvent:(UIEvent *)event {
    [self.text_box resignFirstResponder];
}

- (Danmu)create_danmu:(std::string) text{
    Danmu danmu;
    danmu.tag.lat=cur_lat;
    danmu.tag.lon=cur_lon;
    danmu.tag.alt=0;
    danmu.tag.rx=cur_dir;
    danmu.tag.ry=0;
    danmu.tag.rz=0;
    danmu.tag.time=0;
    danmu.tag.vis_id=0;
    danmu.text=text;
    danmu.usr_id=0;
    return danmu;
}
- (IBAction)text_end:(id)sender {
}
- (IBAction)text_exit:(id)sender {
}

- (void)sent_danmu:(NSString*) text{
    send_danmu_count++;
    BarrageDescriptor * descriptor = [[BarrageDescriptor alloc]init];
    descriptor.spriteName = NSStringFromClass([BarrageWalkTextSprite class]);
    descriptor.params[@"bizMsgId"] = [NSString stringWithFormat:@"%ld",(long)send_danmu_count];
    descriptor.params[@"text"] = text;
    descriptor.params[@"textColor"] = [UIColor whiteColor];
    descriptor.params[@"speed"] = @(100 * (double)random()/RAND_MAX+50);
    descriptor.params[@"direction"] = @(BarrageWalkDirectionR2L);
    descriptor.params[@"side"] = @(BarrageWalkSideDefault);
    [_renderer receive:descriptor];
}

-(BOOL) textFieldShouldReturn:(UITextField *)textField{
    [self.text_box resignFirstResponder];
    return true;
}


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


- (void)captureOutput:(AVCaptureOutput *)captureOutput didOutputSampleBuffer:(CMSampleBufferRef)sampleBuffer fromConnection:(AVCaptureConnection *)connection {
    CMTime timestamp = CMSampleBufferGetPresentationTimeStamp(sampleBuffer);
//    sync_sensor_time = (double)timestamp.value/(double)timestamp.timescale;
//    sync_sys_time = [NSDate date];
//    double time_sec = (double)timestamp.value/(double)timestamp.timescale;
    UIImage *image = [self imageFromSampleBuffer:sampleBuffer];
    dispatch_async( dispatch_get_main_queue(), ^{
        self.image_view.image=image;
    });
//    cv::Mat img_cv = [mm_Try cvMatFromUIImage:image];
//    img_count++;
}

bool outOfChina(double lat, double lon) {
    if (lon < 72.004 || lon > 137.8347) return true;
    if (lat < 0.8293 || lat > 55.8271) return true;
    return false;
}
double transformLat(double x, double y) {
    double pi = 3.1415926535897932384626;
    double ret = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y + 0.1 * x * y
    + 0.2 * sqrt(fabs(x));
    ret += (20.0 * sin(6.0 * x * pi) + 20.0 * sin(2.0 * x * pi)) * 2.0 / 3.0;
    ret += (20.0 * sin(y * pi) + 40.0 * sin(y / 3.0 * pi)) * 2.0 / 3.0;
    ret += (160.0 * sin(y / 12.0 * pi) + 320 * sin(y * pi / 30.0)) * 2.0 / 3.0;
    return ret;
}
double transformLon(double x, double y) {
    double pi = 3.1415926535897932384626;
    double ret = 300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y + 0.1* sqrt(fabs(x));
    ret += (20.0 * sin(6.0 * x * pi) + 20.0 * sin(2.0 * x * pi)) * 2.0 / 3.0;
    ret += (20.0 * sin(x * pi) + 40.0 * sin(x / 3.0 * pi)) * 2.0 / 3.0;
    ret += (150.0 * sin(x / 12.0 * pi) + 300.0 * sin(x / 30.0 * pi)) * 2.0 / 3.0;
    return ret;
}

void gps84_To_Gcj02(double& lat, double& lon) {
    double pi = 3.1415926535897932384626;
    double ee = 0.00669342162296594323;
    double a = 6378245.0;
    if (outOfChina(lat, lon)) {
        return;
    }
    double dLat = transformLat(lon - 105.0, lat - 35.0);
    double dLon = transformLon(lon - 105.0, lat - 35.0);
    double radLat = lat / 180.0 * pi;
    double magic = sin(radLat);
    magic = 1 - ee * magic * magic;
    double sqrtMagic = sqrt(magic);
    dLat = (dLat * 180.0) / ((a * (1 - ee)) / (magic * sqrtMagic) * pi);
    dLon = (dLon * 180.0) / (a / sqrtMagic * cos(radLat) * pi);
    lat = lat + dLat;
    lon = lon + dLon;
}


-(void) locationManager: (CLLocationManager *)manager didUpdateHeading:(CLHeading *)newHeading{
    NSTimeInterval timeInterval = [last_update_time timeIntervalSinceNow];
    //std::cout<<(double)timeInterval<<std::endl;
    if(fabs(timeInterval)<1){
        return;
    }
    last_update_time=[NSDate date];
    CGFloat currentHeading = newHeading.magneticHeading;
    //std::cout<<(double)currentHeading<<std::endl;
    cur_dir =currentHeading;
    [self checkDanmu];
    
}

- (void)locationManager:(CLLocationManager *)manager didUpdateToLocation:(CLLocation *)newLocation fromLocation:(CLLocation *)oldLocation {
    if (newLocation.horizontalAccuracy < 0) {
        return;
    }
    NSTimeInterval locationAge = -[newLocation.timestamp timeIntervalSinceNow];
    if (locationAge > 5.0) {
        return;
    }
    cur_lat=newLocation.coordinate.latitude;
    cur_lon=newLocation.coordinate.longitude;
    [self checkDanmu];
    //std::cout<<cur_lat<<","<<cur_lon<<std::endl;
    //gps84_To_Gcj02(latitude, longitude);
    //float position_covariance=newLocation.horizontalAccuracy;
}


@end
