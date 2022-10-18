#pragma once
#include "api.h"

#define V5_MAX_DEVICE_PORTS 32

typedef enum {
    kDeviceTypeNoSensor        = 0,
    kDeviceTypeMotorSensor     = 2,
    kDeviceTypeLedSensor       = 3,
    kDeviceTypeAbsEncSensor    = 4,
    kDeviceTypeCrMotorSensor   = 5,
    kDeviceTypeImuSensor       = 6,
    kDeviceTypeRangeSensor     = 7,
    kDeviceTypeDistanceSensor  = 7,
    kDeviceTypeRadioSensor     = 8,
    kDeviceTypeTetherSensor    = 9,
    kDeviceTypeBrainSensor     = 10,
    kDeviceTypeVisionSensor    = 11,
    kDeviceTypeAdiSensor       = 12,
    kDeviceTypeRes1Sensor      = 13,
    kDeviceTypeRes2Sensor      = 14,
    kDeviceTypeRes3Sensor      = 15,
    kDeviceTypeOpticalSensor   = 16,
    kDeviceTypeMagnetSensor    = 17,
    kDeviceTypeGpsSensor       = 20,
    kDeviceTypeBumperSensor    = 0x40,
    kDeviceTypeGyroSensor      = 0x46,
    kDeviceTypeSonarSensor     = 0x47,
    kDeviceTypeGenericSensor   = 128,
    kDeviceTypeGenericSerial   = 129,
    kDeviceTypeUndefinedSensor = 255
} V5_DeviceType;

typedef struct _V5_Device* V5_DeviceT;

typedef enum _V5_MotorBrakeMode {
    kV5MotorBrakeModeCoast = 0,  /// Motor will coast when stopped
    kV5MotorBrakeModeBrake = 1,  /// Motor will brake when stopped
    kV5MotorBrakeModeHold  = 2   /// Motor will hold position when stopped
} V5MotorBrakeMode;
