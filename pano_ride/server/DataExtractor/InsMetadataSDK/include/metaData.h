#ifndef INS_METADATA_H__
#define INS_METADATA_H__
#include <memory>
#include <string>
#include <vector>

#include "common.h"

#if WIN32
#ifdef USE_EXPORTDLL
#define METADATASDK_API _declspec(dllexport)
#else
#define METADATASDK_API _declspec(dllimport)
#endif
#else
#define METADATASDK_API
#endif

namespace ins_metadata {

	class  MetaDataParserImp;

	/**
	* \class MetaDataParser
	* \brief This class is used to extract some data, such as GPSData, gyroData and exposure data, from file of specified format that insv. 
	*/

	class METADATASDK_API MetaDataParser {

	public:
		MetaDataParser();
		~MetaDataParser();

	public:

		/**
		* \brief parse file 
		* \param the file path that you want parsed
		*/
		bool Parse(const std::string& filePath);

		/**
		* \brief Get GPS information
		* \param the vector that GpsData stored
		*/
		bool GetGPSData(std::vector<GpsDataItem_t>& GpsData);

		/**
		* \brief Get Gyro information
		* \param the vector that GyroData stored
		*/
		bool GetGyroData(std::vector<GyroDataItem_t>& GyroData);

		/**
		* \brief Get Exposure information
		* \param the vector that ExposureData stored
		*/
		bool GetExposureData(std::vector<ExposureDataItem_t>& ExposureData);

		/**
		* \brief Get FirstTimeStamp that used for GyroData
		* \param None
		* \return ms
		*/
		int64_t GetFirstTimeStamp() const;

		/**
		* \brief Get offset
		* \param None
		* \return ms
		*/
		std::string GetOffset() const;

		/**
		* \brief Get the version of FireWare
		* \param None
		* \return ms
		*/
		std::string GetFireWareVersion() const;

		/**
		* \brief Get camera type
		* \param None
		* \return ms
		*/
		std::string GetCameraType() const;

	private:
		std::shared_ptr<MetaDataParserImp> imp_;
	};
}
#endif