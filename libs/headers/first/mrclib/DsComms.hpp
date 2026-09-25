// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "stdint.h"
#include <span>
#include "DsComms.h"

namespace mrclib {

constexpr uint32_t EnabledShift = 0;
constexpr uint32_t RobotModeShift = 1;
constexpr uint32_t EStopShift = 3;
constexpr uint32_t FmsConnectedShift = 4;
constexpr uint32_t DsConnectedShift = 5;
constexpr uint32_t WatchdogActiveShift = 6;
constexpr uint32_t SupportsOpModesShift = 7;
constexpr uint32_t AllianceShift = 8;
constexpr uint32_t RobotWifiModeShift = 12;
constexpr uint32_t TimedMatchShift = 13;
constexpr uint32_t BrownoutShift = 14;

constexpr uint32_t EnabledMask = 0x1 << EnabledShift;
constexpr uint32_t RobotModeMask = 0x3 << RobotModeShift;
constexpr uint32_t EStopMask = 0x1 << EStopShift;
constexpr uint32_t FmsConnectedMask = 0x1 << FmsConnectedShift;
constexpr uint32_t DsConnectedMask = 0x1 << DsConnectedShift;
constexpr uint32_t WatchdogActiveMask = 0x1 << WatchdogActiveShift;
constexpr uint32_t SupportsOpModesMask = 0x1 << SupportsOpModesShift;
constexpr uint32_t AllianceMask = 0xF << AllianceShift;
constexpr uint32_t RobotWifiModeMask = 0x1 << RobotWifiModeShift;
constexpr uint32_t TimedMatchMask = 0x1 << TimedMatchShift;
constexpr uint32_t BrownoutMask = 0x1 << BrownoutShift;

#define GENERATE_GET_FUNCTION(Name)                        \
    constexpr uint32_t Get##Name(uint32_t ControlFlags) {  \
        return (ControlFlags & Name##Mask) >> Name##Shift; \
    }

GENERATE_GET_FUNCTION(Enabled)
GENERATE_GET_FUNCTION(RobotMode)
GENERATE_GET_FUNCTION(EStop)
GENERATE_GET_FUNCTION(FmsConnected)
GENERATE_GET_FUNCTION(DsConnected)
GENERATE_GET_FUNCTION(WatchdogActive)
GENERATE_GET_FUNCTION(SupportsOpModes)
GENERATE_GET_FUNCTION(Alliance)
GENERATE_GET_FUNCTION(RobotWifiMode)
GENERATE_GET_FUNCTION(TimedMatch)
GENERATE_GET_FUNCTION(Brownout)

#undef GENERATE_GET_FUNCTION
}  // namespace mrclib
