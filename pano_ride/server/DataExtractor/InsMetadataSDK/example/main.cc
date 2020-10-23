
#include <metaData.h>
#include <fstream>
#include <iostream>
#include <iomanip>

std::ostream& operator<<(std::ostream &os, const GpsDataItem_t &item) {
	os << item.timestampMs<< std::setprecision(20)
		<< "," << item.latitude
		<< "," << item.longitude
		<< "," << item.altitude << std::endl;
	return os;
}

std::ostream& operator<<(std::ostream &os, const GyroDataItem_t &item) {

	os << item.timestamp
		<< "," << item.rotation[0] << "," << item.rotation[1] << "," << item.rotation[2]
		<< "," << item.acceleration[0] << "," << item.acceleration[1] << "," << item.acceleration[2] << std::endl;
	return os;
}

std::ostream& operator<<(std::ostream &os, const ExposureDataItem_t &item) {

	os << item.timestamp
		<< "," << item.shutter_speed_s << std::endl;
	return os;
}

int main(int argc, char* argv[]) {

	std::string strFilePath = argv[1];
    std::string strOutPath = argv[2];
    std::string strOutPre = argv[3];

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
			std::ofstream out(strOutPath+"/"+strOutPre+"_GpsData.txt");

			for (auto& GpsDataItem: GpsData) {
				out <<GpsDataItem;
			}
		}
		if (!GyroData.empty()) {
			std::ofstream out(strOutPath+"/"+strOutPre+"_GyroData.txt");

			for (auto& GyroDataItem : GyroData) {
				out << GyroDataItem;
			}
		}
		if (!ExposureData.empty()) {
			std::ofstream out(strOutPath+"/"+strOutPre+"_ExposureData.txt");

			for (auto& ExposureDataItem : ExposureData) {
				out << ExposureDataItem;
			}
		}
        std::ofstream out(strOutPath+"/"+strOutPre+"_FirstImage.txt");
        out << firstTimeStamp;
	}

	return 0;
}
