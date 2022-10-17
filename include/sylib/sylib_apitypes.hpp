/**
 * \file sylib/sylib_apitypes.hpp
 *
 * \brief Contains enumerated types used by Sylib
 */

#pragma once

namespace sylib {
    typedef enum  {
        SylibMotorControlModeOFF          = 0,     /// Motor is off and in coast mode
        SylibMotorControlModeBRAKE        = 1,     /// Motor is off and in brake mode
        SylibMotorControlModeHOLD         = 2,     /// Motor is holding at current position
        SylibMotorControlModeCUSTOM       = 3,     /// Motor is in velocity control mode
        SylibMotorControlModePOSITION     = 4,     /// Motor is in velocity control mode
        SylibMotorControlModeVelocityAUTO = 5,     /// Motor is controlled by internal firmware
        SylibMotorControlModePositionAUTO = 6,
        SylibMotorControlModeVOLTAGE      = 7      /// Motor is set to a specific voltage
    } SylibMotorControlMode;

    typedef enum {
        SylibAddrledControlModeOFF = 0,
        SylibAddrledControlModeMANUAL = 1,
        SylibAddrledControlModePULSE = 2,
        SylibAddrledControlModeCYCLE = 3
    }SylibAddrledControlMode;
}