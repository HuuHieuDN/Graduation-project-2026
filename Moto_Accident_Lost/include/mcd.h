#ifndef MCD_H
#define MCD_H

#include <Arduino.h>
#include <TinyGPSPlus.h>
#include <MPU6050_tockn.h>

typedef struct {
  double lat;
  double lng;


} Location;

enum DeviceStatus {
  DEV_NONE  = 0,
  DEV_FALL  = 1,
  DEV_CRASH = 2,
  DEV_LOST1 = 3,
  DEV_LOST2 = 4
};

void mcdInit(uint8_t ledPin, uint8_t buzzerPin, uint8_t armSwitchPin);
DeviceStatus mcdUpdate(MPU6050 &mpu, TinyGPSPlus &gps, bool *antiTheftEnabledOut = nullptr);
void mcdRemoteSetAntiTheft(bool enable, TinyGPSPlus &gps);
float mcdGetLastCrashDeltaG();
float mcdGetCrashDeltaGThreshold();

#endif
