#ifndef INSMETADATA_COMMOM_H__
#define INSMETADATA_COMMOM_H__

#include <cstdlib>
#include <stdint.h>

typedef struct ExposureDataItem {
  // timestamp, in milliseconds
  int64_t timestamp;
  // shutter speed, in seconds
  double shutter_speed_s;
}ExposureDataItem_t;

typedef struct GpsDataItem {
  // timestamp, in seconds
  int64_t timestampMs;
  double latitude;
  double longitude;
  double altitude;
}GpsDataItem_t;

typedef struct GyroDataItem{
  int64_t timestamp;
  double acceleration[3];
  double rotation[3];
}GyroDataItem_t;

#endif