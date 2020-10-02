# MetaDataSDK-Cpp
MetaData-Cpp is a C++ library that is used to extract some data, such as GPSData, gyroData and exposure data, from file of specified format that insv.

### Supported platforms

| Platform | Version                                       |
| :------- | :-------------------------------------------- |
| Windows  | Windows 7 or later, only x64 supported        |
| Linux    | Ubuntu, other distributions need to be tested |

### Supported file format

| filetype | import format | export format |
| :------- | :------------ | :------------ |
| Video    | insv          | mp4           |

## Table of contents

- [Running test demo](#demo)
- [Get Started](#started)
- [Parameters Guidance](#Guidance)
	- [GyroDataItem](#GyroDataItem)
	- [GpsDataItem](#GpsDataItem)
	- [how to use gyrodata](#how to use gyrodata)


## <span id="demo">Running test demo</span>

**On Ubuntu**, follow the steps below to build and run **metaSDKDemo**.

```bash
$ gcc main.cc -o metaSDKDemo -I/path/to/include -L /path/to/lib -lInsMetaDataSDK -lpthread
$ sudo chmod +x metaSDKDemo
$ sudo cp /path/to/libInsMetaDataSDK.so /usr/lib
$ ./metaSDKDemo /path/to/insv
```

## <span id="started">Get Started</span>

```c++
#include <metaData.h>
#include <fstream>
#include <iostream>

std::ostream& operator<<(std::ostream &os, const GpsDataItem_t &item) {
	os << "timestamp:" << item.timestampMs
		<< " latitude:" << item.latitude
		<< " longitude:" << item.longitude
		<< " altitude:" << item.altitude << std::endl;
	return os;
}

std::ostream& operator<<(std::ostream &os, const GyroDataItem_t &item) {

	os << "timestamp:" << item.timestamp
		<< " gyro:" << item.rotation[0] << " " << item.rotation[1] << " " << item.rotation[2]
		<< " accel:" << item.acceleration[0] << " " << item.acceleration[1] << " " << item.acceleration[2] << std::endl;
	return os;
}

std::ostream& operator<<(std::ostream &os, const ExposureDataItem_t &item) {

	os << "timestamp:" << item.timestamp
		<< " shutter_speed_s:" << item.shutter_speed_s << std::endl;
	return os;
}

int main(int argc, char* argv[]) {

	std::string strFilePath = argv[1];

	auto metaDataParser = std::make_shared<ins_metadata::MetaDataParser>();

	if (metaDataParser->Parse(strFilePath)) {
	
		std::vector<GpsDataItem_t> GpsData;
		std::vector<GyroDataItem_t> GyroData; 
		std::vector<ExposureDataItem_t> ExposureData;

		metaDataParser->GetExposureData(ExposureData);

		metaDataParser->GetGyroData(GyroData);

		metaDataParser->GetGPSData(GpsData);	

		int64_t firstTimeStamp = metaDataParser->GetFirstTimeStamp();
		
		std::cout << "FirstTimeStamp: " << firstTimeStamp << std::endl;
		std::cout << "Offset: " << metaDataParser->GetOffset() << std::endl;
		std::cout << "CameraType: " << metaDataParser->GetCameraType() << std::endl;
		std::cout << "FirewareVersion: " << metaDataParser->GetFireWareVersion() << std::endl;

		if (!GpsData.empty()) {
			std::ofstream out("GpsData.txt");

			for (auto& GpsDataItem: GpsData) {
				out << GpsDataItem;
			}
		}

		if (!GyroData.empty()) {
			std::ofstream out("GyroData.txt");

			for (auto& GyroDataItem : GyroData) {
				out << GyroDataItem;
			}
		}

		if (!ExposureData.empty()) {
			std::ofstream out("ExposureData.txt");

			for (auto& ExposureDataItem : ExposureData) {
				out << ExposureDataItem;
			}
		}
	}

	return 0;
}
```



## <span id="Guidance">Parameters Guidance</span>

#### <span id = "GyroDataItem">GyroDataItem_t</span>
```c++
typedef struct GyroDataItem{
  int64_t timestamp; 		// microsecond
  double acceleration[3];   // G 
  double rotation[3];		// Rad/s
}GyroDataItem_t;
```
####  <span id="GpsDataItem">GpsDataItem_t</span>

```c++
typedef struct GpsDataItem {
  int64_t timestampMs; // timestampMs
  double latitude;	   // 纬度    
  double longitude;    // 经度
  double altitude;     // 海拔 m
}GpsDataItem_t;
```

#### <span id="how to use gyrodata">how to use gyrodata</span>

you can get some information and files after you run demo

```
FirstTimeStamp: 68991
Offset:2_1481.87_1515.86_1515.49_-0.216822_0.00549514_-179.347_1487.04_4559.34_1503.08_0.541963_0.0495245_0.171921_6080_3040_3105
CameraType: Insta360 OneR
FirewareVersion: v1.1.20_build1
```

and when you open GyroData.txt , you will see the message

```
timestamp:68969 gyro:-0.190682 -0.134223 -0.092678 accel:-0.883301 -0.110352 -0.09375
timestamp:68971 gyro:-0.186421 -0.136354 -0.0937433 accel:-0.881592 -0.111816 -0.0935059
timestamp:68973 gyro:-0.18216 -0.137419 -0.0948085 accel:-0.87793 -0.115723 -0.0898438
timestamp:68975 gyro:-0.173638 -0.13955 -0.0958738 accel:-0.876953 -0.115723 -0.0883789
...
timestamp:68989 gyro:-0.143811 -0.147006 -0.0958738 accel:-0.861816 -0.108154 -0.0751953
timestamp:68991 gyro:-0.137419 -0.149137 -0.0969391 accel:-0.856201 -0.111572 -0.0756836
timestamp:68993 gyro:-0.133158 -0.148072 -0.0958738 accel:-0.849854 -0.111816 -0.0732422
timestamp:68995 gyro:-0.131028 -0.149137 -0.0958738 accel:-0.850098 -0.108398 -0.0749512
timestamp:68997 gyro:-0.126766 -0.150202 -0.0958738 accel:-0.845947 -0.10498 -0.0756836
...
timestamp:69014 gyro:-0.118244 -0.149137 -0.0948085 accel:-0.82959 -0.0959473 -0.0561523
timestamp:69016 gyro:-0.118244 -0.148072 -0.092678 accel:-0.829834 -0.0932617 -0.0566406
timestamp:69018 gyro:-0.118244 -0.148072 -0.0916127 accel:-0.831299 -0.0878906 -0.0549316
```

The '**FirstTimeStamp**' is when the camera starts recording and generates the first frame. 

***'FirstTimeStamp: 68991**' actually goes inbetween two Gyro output. You could use either of the timestamp to pin the first Gyro data for the first stamp and work from there on.

Note that camera begins to record Gyro data as soon as it is triggered, and it cease to record data after video finishes recording. So you may find a longer Gyro data on both ends of the duration.







