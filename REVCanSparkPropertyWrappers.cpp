

#ifdef _WIN32
#define EXPORT_FUNC __declspec(dllexport) __cdecl
#ifndef _WIN64
#pragma pack(push, 1)
#endif
#elif defined(__linux)
#define EXPORT_FUNC
#endif


#if 0
TODO
need to add DutyCycle properties
typedef struct {
    float dutyCycleEncoderVelocity;
    float dutyCycleEncoderPosition;
    uint64_t timestamp;
} c_Spark_PeriodicStatus5;

typedef struct {
    float iAccumulation;
    uint64_t timestamp;
} c_Spark_PeriodicStatus7;

// we currently have softLimit value, but not "wasReached" I think that is what we meant to publish


library: niREVSpark.dll
#endif

#define __STDC_LIMIT_MACROS
#include <stdint.h>
#include <cstdint>
#include "extcode.h"
//#include "oldtd.h"

#include "rev/CANCommonParameters.h"
#include "rev/CANSparkParameters.h"
#include "rev/CANSparkDriver.h"
#include "rev/REVLibErrors.h"
#include "rev/REVCommon.h"

#include "rev/REVLibVersion.h"
#include "rev/REVUtils.h"


struct private_parameter_table_entry_t {
    c_Spark_ConfigParameter id;
    c_REVLib_ParameterType type;
    uint32_t defaultValue;
};

struct vCompType {uInt32 mode; float64 nominal;};
struct followType {uInt32 leader; LVBoolean invert;};
struct posPIDWrapType {LVBoolean enable; float64 min, max;};
struct PIDFType {float64 kP, kI, kD, kF;};
struct PIDRngType {float64 min, max;};
struct PIDAdvType {float64 iZone, maxI, dFilter, errLimit;};
struct PIDMdlType {float64 kS, kA, kG, kCos, kCosRatio;};
struct MAXCfgType {float64 vel, accel, jerk, goal; int32 mode;};
struct MotorFaultsType {LVBoolean other, type, sensor, can, temp, gateDrv, escEEProm, firmware;};
struct MotorWarnType {LVBoolean brown, over, escEEProm, ext, sensor, stall, hasReset, other;};
struct MotorDetailsType {uInt32 can, major, minor, type, mode, prod, kV, pairs; float64 adv;};

// COPIED FROM CANSpartParameters.cpp because the removed the export of the info.
private_parameter_table_entry_t paramInfo[] = {
        {c_Spark_kCANID, c_REVLib_kUint32, 0},
        {c_Spark_kInputMode, c_REVLib_kUint32, c_Spark_kInputMode_PWM},
        {c_Spark_kMotorType, c_REVLib_kUint32, c_Spark_kMotorType_BRUSHLESS},
        {c_Spark_kCommutationAdvance, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kUnused, c_REVLib_kUnused, 0},
        {c_Spark_kControlType, c_REVLib_kUint32, c_Spark_kControlType_DUTY_CYCLE},
        {c_Spark_kIdleMode, c_REVLib_kUint32, c_Spark_kIdleMode_COAST},
        {c_Spark_kInputDeadband, c_REVLib_kFloat32, 0x3d4ccccd},
        {c_Spark_kUnused, c_REVLib_kUnused, 0},
        {c_Spark_kClosedLoopControlSensor, c_REVLib_kUint32, c_Spark_kSensor_NONE},
        {c_Spark_kPolePairs, c_REVLib_kUint32, 7},
        {c_Spark_kCurrentChop, c_REVLib_kFloat32, 0x42e60000},
        {c_Spark_kCurrentChopCycles, c_REVLib_kUint32, 0},
        {c_Spark_kP_0, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kI_0, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kD_0, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kV_0, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kIZone_0, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kDFilter_0, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kOutputMin_0, c_REVLib_kFloat32, 0xbf800000},
        {c_Spark_kOutputMax_0, c_REVLib_kFloat32, 0x3f800000},
        {c_Spark_kP_1, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kI_1, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kD_1, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kV_1, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kIZone_1, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kDFilter_1, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kOutputMin_1, c_REVLib_kFloat32, 0xbf800000},
        {c_Spark_kOutputMax_1, c_REVLib_kFloat32, 0x3f800000},
        {c_Spark_kP_2, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kI_2, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kD_2, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kV_2, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kIZone_2, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kDFilter_2, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kOutputMin_2, c_REVLib_kFloat32, 0xbf800000},
        {c_Spark_kOutputMax_2, c_REVLib_kFloat32, 0x3f800000},
        {c_Spark_kP_3, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kI_3, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kD_3, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kV_3, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kIZone_3, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kDFilter_3, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kOutputMin_3, c_REVLib_kFloat32, 0xbf800000},
        {c_Spark_kOutputMax_3, c_REVLib_kFloat32, 0x3f800000},
        {c_Spark_kInverted, c_REVLib_kBool, false},
        {c_Spark_kUnused, c_REVLib_kUnused, 0},
        {c_Spark_kUnused, c_REVLib_kUnused, 0},
        {c_Spark_kUnused, c_REVLib_kUnused, 0},
        {c_Spark_kUnused, c_REVLib_kUnused, 0},
        {c_Spark_kLimitSwitchFwdPolarity, c_REVLib_kBool, false},
        {c_Spark_kLimitSwitchRevPolarity, c_REVLib_kBool, false},
        {c_Spark_kHardLimitFwdEn, c_REVLib_kUint32, c_Spark_kLimitSwBehavior_STOP_MOVING_MOTOR},
        {c_Spark_kHardLimitRevEn, c_REVLib_kUint32, c_Spark_kLimitSwBehavior_STOP_MOVING_MOTOR},
        {c_Spark_kSoftLimitFwdEn, c_REVLib_kBool, false},
        {c_Spark_kSoftLimitRevEn, c_REVLib_kBool, false},
        {c_Spark_kOpenLoopRampRate, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kLegacyFollowerID, c_REVLib_kUint32, 0},
        {c_Spark_kLegacyFollowerConfig, c_REVLib_kUint32, 0},
        {c_Spark_kSmartCurrentStallLimit, c_REVLib_kUint32, 80},
        {c_Spark_kSmartCurrentFreeLimit, c_REVLib_kUint32, 20},
        {c_Spark_kSmartCurrentConfig, c_REVLib_kUint32, 10000},
        {c_Spark_kSmartCurrentReserved, c_REVLib_kUint32, 0},
        {c_Spark_kMotorKv, c_REVLib_kUint32, 480},
        {c_Spark_kUnused, c_REVLib_kUnused, 0},
        {c_Spark_kUnused, c_REVLib_kUnused, 0},
        {c_Spark_kUnused, c_REVLib_kUnused, 0},
        {c_Spark_kUnused, c_REVLib_kUnused, 0},
        {c_Spark_kUnused, c_REVLib_kUnused, 0},
        {c_Spark_kEncoderCountsPerRev, c_REVLib_kUint32, 8192},
        {c_Spark_kEncoderAverageDepth, c_REVLib_kUint32, 8},
        {c_Spark_kEncoderSampleDelta, c_REVLib_kUint32, 20},
        {c_Spark_kEncoderInverted, c_REVLib_kBool, false},
        {c_Spark_kUnused, c_REVLib_kUnused, 0},
        {c_Spark_kVoltageCompensationMode, c_REVLib_kUint32, c_Spark_kVoltageCompMode_NO_VOLTAGE_COMP},
        {c_Spark_kCompensatedNominalVoltage, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kUnused, c_REVLib_kUnused, 0},
        {c_Spark_kUnused, c_REVLib_kUnused, 0},
        {c_Spark_kUnused, c_REVLib_kUnused, 0},
        {c_Spark_kUnused, c_REVLib_kUnused, 0},
        {c_Spark_kUnused, c_REVLib_kUnused, 0},
        {c_Spark_kUnused, c_REVLib_kUnused, 0},
        {c_Spark_kUnused, c_REVLib_kUnused, 0},
        {c_Spark_kUnused, c_REVLib_kUnused, 0},
        {c_Spark_kUnused, c_REVLib_kUnused, 0},
        {c_Spark_kUnused, c_REVLib_kUnused, 0},
        {c_Spark_kUnused, c_REVLib_kUnused, 0},
        {c_Spark_kUnused, c_REVLib_kUnused, 0},
        {c_Spark_kUnused, c_REVLib_kUnused, 0},
        {c_Spark_kUnused, c_REVLib_kUnused, 0},
        {c_Spark_kUnused, c_REVLib_kUnused, 0},
        {c_Spark_kUnused, c_REVLib_kUnused, 0},
        {c_Spark_kUnused, c_REVLib_kUnused, 0},
        {c_Spark_kUnused, c_REVLib_kUnused, 0},
        {c_Spark_kUnused, c_REVLib_kUnused, 0},
        {c_Spark_kUnused, c_REVLib_kUnused, 0},
        {c_Spark_kIMaxAccum_0, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kAllowedClosedLoopError_0, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kUnused, c_REVLib_kUnused, 0},
        {c_Spark_kUnused, c_REVLib_kUnused, 0},
        {c_Spark_kIMaxAccum_1, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kAllowedClosedLoopError_1, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kUnused, c_REVLib_kUnused, 0},
        {c_Spark_kUnused, c_REVLib_kUnused, 0},
        {c_Spark_kIMaxAccum_2, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kAllowedClosedLoopError_2, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kUnused, c_REVLib_kUnused, 0},
        {c_Spark_kUnused, c_REVLib_kUnused, 0},
        {c_Spark_kIMaxAccum_3, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kAllowedClosedLoopError_3, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kUnused, c_REVLib_kUnused, 0},
        {c_Spark_kUnused, c_REVLib_kUnused, 0},
        {c_Spark_kPositionConversionFactor, c_REVLib_kFloat32, 0x3f800000},
        {c_Spark_kVelocityConversionFactor, c_REVLib_kFloat32, 0x3f800000},
        {c_Spark_kClosedLoopRampRate, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kSoftLimitForward, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kSoftLimitReverse, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kUnused, c_REVLib_kUnused, 0},
        {c_Spark_kUnused, c_REVLib_kUnused, 0},
        {c_Spark_kAnalogPositionConversion, c_REVLib_kFloat32, 0x3f800000},
        {c_Spark_kAnalogVelocityConversion, c_REVLib_kFloat32, 0x3f800000},
        {c_Spark_kAnalogAverageDepth, c_REVLib_kUint32, 64},
        {c_Spark_kAnalogSensorMode, c_REVLib_kUint32, c_Spark_kAnalogMode_ABSOLUTE},
        {c_Spark_kAnalogInverted, c_REVLib_kBool, 0},
        {c_Spark_kAnalogSampleDelta, c_REVLib_kUint32, 200},
        {c_Spark_kUnused, c_REVLib_kUnused, 0},
        {c_Spark_kUnused, c_REVLib_kUnused, 0},
        {c_Spark_kCompatibilityPortConfig, c_REVLib_kUint32, c_Spark_kCompatibilityPort_DEFAULT},
        {c_Spark_kAltEncoderCountsPerRev, c_REVLib_kUint32, 8192},
        {c_Spark_kAltEncoderAverageDepth, c_REVLib_kUint32, 8},
        {c_Spark_kAltEncoderSampleDelta, c_REVLib_kUint32, 20},
        {c_Spark_kAltEncoderInverted, c_REVLib_kBool, false},
        {c_Spark_kAltEncoderPositionConversion, c_REVLib_kFloat32, 0x3f800000},
        {c_Spark_kAltEncoderVelocityConversion, c_REVLib_kFloat32, 0x3f800000},
        {c_Spark_kUnused, c_REVLib_kUnused, 0},
        {c_Spark_kUnused, c_REVLib_kUnused, 0},
        {c_Spark_kUnused, c_REVLib_kUnused, 0},
        {c_Spark_kUnused, c_REVLib_kUnused, 0},
        {c_Spark_kNumParameters, c_REVLib_kUint32, 137},
        {c_Spark_kDutyCyclePositionFactor, c_REVLib_kFloat32, 0x3f800000},
        {c_Spark_kDutyCycleVelocityFactor, c_REVLib_kFloat32, 0x3f800000},
        {c_Spark_kDutyCycleInverted, c_REVLib_kBool, false},
        {c_Spark_kUnused, c_REVLib_kUnused, 0},
        {c_Spark_kDutyCycleAverageDepth, c_REVLib_kUint32, 7},
        {c_Spark_kUnused, c_REVLib_kUnused, 0},
        {c_Spark_kDutyCycleOffsetLegacy, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kDutyCycleRangeOffset, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kUnused, c_REVLib_kUnused, 0},
        {c_Spark_kUnused, c_REVLib_kUnused, 0},
        {c_Spark_kPositionPIDWrapEnable, c_REVLib_kBool, false},
        {c_Spark_kPositionPIDMinInput, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kPositionPIDMaxInput, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kUnused, c_REVLib_kUnused, 0},
        {c_Spark_kDutyCycleSensorPrescaler, c_REVLib_kUint32, 17},
        {c_Spark_kDutyCycleOffset, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kProductId, c_REVLib_kUint32, 0},
        {c_Spark_kDeviceMajorVersion, c_REVLib_kUint32, 0},
        {c_Spark_kDeviceMinorVersion, c_REVLib_kUint32, 4294967295},
        {c_Spark_kStatus0Period, c_REVLib_kUint32, 10},
        {c_Spark_kStatus1Period, c_REVLib_kUint32, 250},
        {c_Spark_kStatus2Period, c_REVLib_kUint32, 20},
        {c_Spark_kStatus3Period, c_REVLib_kUint32, 20},
        {c_Spark_kStatus4Period, c_REVLib_kUint32, 20},
        {c_Spark_kStatus5Period, c_REVLib_kUint32, 20},
        {c_Spark_kStatus6Period, c_REVLib_kUint32, 20},
        {c_Spark_kStatus7Period, c_REVLib_kUint32, 20},
        {c_Spark_kMAXMotionCruiseVelocity_0, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kMAXMotionMaxAccel_0, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kMAXMotionMaxJerk_0, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kMAXMotionAllowedProfileError_0, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kMAXMotionPositionMode_0, c_REVLib_kUint32, c_Spark_kMAXMotionPositionMode_TRAPEZOIDAL},
        {c_Spark_kMAXMotionCruiseVelocity_1, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kMAXMotionMaxAccel_1, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kMAXMotionMaxJerk_1, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kMAXMotionAllowedProfileError_1, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kMAXMotionPositionMode_1, c_REVLib_kUint32, c_Spark_kMAXMotionPositionMode_TRAPEZOIDAL},
        {c_Spark_kMAXMotionCruiseVelocity_2, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kMAXMotionMaxAccel_2, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kMAXMotionMaxJerk_2, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kMAXMotionAllowedProfileError_2, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kMAXMotionPositionMode_2, c_REVLib_kUint32, c_Spark_kMAXMotionPositionMode_TRAPEZOIDAL},
        {c_Spark_kMAXMotionCruiseVelocity_3, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kMAXMotionMaxAccel_3, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kMAXMotionMaxJerk_3, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kMAXMotionAllowedProfileError_3, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kMAXMotionPositionMode_3, c_REVLib_kUint32, c_Spark_kMAXMotionPositionMode_TRAPEZOIDAL},
        {c_Spark_kForceEnableStatus_0, c_REVLib_kBool, false},
        {c_Spark_kForceEnableStatus_1, c_REVLib_kBool, false},
        {c_Spark_kForceEnableStatus_2, c_REVLib_kBool, false},
        {c_Spark_kForceEnableStatus_3, c_REVLib_kBool, false},
        {c_Spark_kForceEnableStatus_4, c_REVLib_kBool, false},
        {c_Spark_kForceEnableStatus_5, c_REVLib_kBool, false},
        {c_Spark_kForceEnableStatus_6, c_REVLib_kBool, false},
        {c_Spark_kForceEnableStatus_7, c_REVLib_kBool, false},
        {c_Spark_kFollowerModeLeaderId, c_REVLib_kUint32, 0},
        {c_Spark_kFollowerModeIsInverted, c_REVLib_kBool, false},
        {c_Spark_kDutyCycleEncoderStartPulseUs, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kDutyCycleEncoderEndPulseUs, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kParamTableVersion, c_REVLib_kUint32, 0},
        {c_Spark_kStatus8Period, c_REVLib_kUint32, 10},
        {c_Spark_kForceEnableStatus_8, c_REVLib_kBool, false},
        {c_Spark_kLimitSwitchPositionSensor, c_REVLib_kUint32, c_Spark_kSensor_NONE},
        {c_Spark_kLimitSwitchFwdPosition, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kLimitSwitchRevPosition, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kS_0, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kA_0, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kG_0, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kCos_0, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kCosRatio_0, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kS_1, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kA_1, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kG_1, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kCos_1, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kCosRatio_1, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kS_2, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kA_2, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kG_2, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kCos_2, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kCosRatio_2, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kS_3, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kA_3, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kG_3, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kCos_3, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kCosRatio_3, c_REVLib_kFloat32, 0x00000000},
        {c_Spark_kStatus9Period, c_REVLib_kUint32, 10},
        {c_Spark_kForceEnableStatus_9, c_REVLib_kBool, false},
        {c_Spark_kDetachedEncoderDeviceID, c_REVLib_kUint32, 0},
    };





typedef uInt32              REVMotorRef;

int32 LV_REVSparkSetAttribute(REVMotorRef refPtr, c_Spark_ConfigParameter id, void *ptr) {
	c_REVLib_ErrorCode revErr;
	int32_t i32Val;
	uint32_t u32Val;
	float f32Val;
	uint8_t boolVal;
	c_Spark_handle hdl;
	// get hdl from refPtr return if doesn't work

	// this property directly maps to a REV property, so use the above table
	if(id <= c_Spark_kDetachedEncoderDeviceID ) { // better with a sentinel
		switch(paramInfo[id].type) {
			default:
			case c_REVLib_kUnused:
			// debug printing, return error
			break;

			case c_REVLib_kInt32:
			i32Val = *(int32*)ptr;
			revErr = c_Spark_SetParameterInt32(hdl, id, i32Val);
			break;

			case c_REVLib_kUint32:
					u32Val = *(uInt32*)ptr;
			revErr = c_Spark_SetParameterUint32(hdl, id, u32Val);
			break;

			case c_REVLib_kFloat32:
					f32Val = *(float*)ptr;
			revErr = c_Spark_SetParameterFloat32(hdl, id, f32Val);
			break;

			case c_REVLib_kBool:
					boolVal = *(int8*)ptr;
			revErr = c_Spark_SetParameterBool(hdl, id, boolVal);
			break;
		}

	}
	else { // property wraps other function calls or aggregates multiple properties into a cluster
		int32 slotOfst = 0;

		switch(id) {
			case 0x200: { // VoltageCompensation -- Spark Voltage Compensation.ctl
				vCompType *vc = (vCompType*)ptr;
				LV_REVSparkSetAttribute(refPtr, c_Spark_kVoltageCompensationMode, &(vc->mode));
				LV_REVSparkSetAttribute(refPtr, c_Spark_kCompensatedNominalVoltage, &(vc->nominal));
			}
			break;
			case 0x205: {// FollowerModeInfo	-- cluster {2, "Leader CAN ID"=uL, "Inverted"=bool}
				followType *follow = (followType*)ptr;
				LV_REVSparkSetAttribute(refPtr, c_Spark_kFollowerModeLeaderId, &(follow->leader));
				LV_REVSparkSetAttribute(refPtr, c_Spark_kFollowerModeIsInverted, &(follow->invert));
			}
			break;

			case 0x206: { // PositionPIDWrap	-- cluster { 3, "Enable" = bool, "Min" = fD, "Max"=fD }
				posPIDWrapType *pos = (posPIDWrapType*)ptr;
				LV_REVSparkSetAttribute(refPtr, c_Spark_kPositionPIDWrapEnable, &(pos->enable));
				LV_REVSparkSetAttribute(refPtr, c_Spark_kPositionPIDMinInput, &(pos->min));
				LV_REVSparkSetAttribute(refPtr, c_Spark_kPositionPIDMaxInput, &(pos->max));
			}
			break;





			case 0x213: // PIDF_3	
				slotOfst ++;
			case 0x20f: // PIDF_2	
				slotOfst ++;
			case 0x20b: // PIDF_1	
				slotOfst ++;
			case 0x207: { // PIDF_0	-- cluster { 4, "kP" = fD, "kI" = fD, "kD"=fD, "kF"=fD }
				PIDFType *tune = (PIDFType*)ptr;
				tune += slotOfst;
				LV_REVSparkSetAttribute(refPtr, (c_Spark_ConfigParameter)(c_Spark_kP_0 + slotOfst*8), &(tune->kP));
				LV_REVSparkSetAttribute(refPtr, (c_Spark_ConfigParameter)(c_Spark_kI_0 + slotOfst*8), &(tune->kI));
				LV_REVSparkSetAttribute(refPtr, (c_Spark_ConfigParameter)(c_Spark_kD_0 + slotOfst*8), &(tune->kD));
				LV_REVSparkSetAttribute(refPtr, (c_Spark_ConfigParameter)(c_Spark_kV_0 + slotOfst*8), &(tune->kF));
			}
			break;

			case 0x214: // PIDOutRng_3
				slotOfst ++;
			case 0x210: // PIDOutRng_2
				slotOfst ++;
			case 0x20C: // PIDOutRng_1
				slotOfst ++;
			case 0x208: { // PIDOutRng_0	-- cluster { 2, "min" = fD, "max" = fD }
				PIDRngType *rng = (PIDRngType*)ptr;
				rng += slotOfst;
				LV_REVSparkSetAttribute(refPtr, (c_Spark_ConfigParameter)(c_Spark_kOutputMin_0 + slotOfst*8), &(rng->min));
				LV_REVSparkSetAttribute(refPtr, (c_Spark_ConfigParameter)(c_Spark_kOutputMax_0 + slotOfst*8), &(rng->max));
			}
			break;

			case 0x215: // PIDAdv_3
				slotOfst ++;
			case 0x211: // PIDAdv_2
				slotOfst ++;
			case 0x20D: // PIDAdv_1
				slotOfst ++;
			case 0x209: { // PIDAdv_0	-- { 4, "i Zone" = fD, "max I Accum" = fD, "d Filter"=fD, "error Limit"=fD }
				PIDAdvType *adv = (PIDAdvType*)ptr;
				adv += slotOfst;
				LV_REVSparkSetAttribute(refPtr, (c_Spark_ConfigParameter)(c_Spark_kIZone_0 + slotOfst*8), &(adv->iZone));
				LV_REVSparkSetAttribute(refPtr, (c_Spark_ConfigParameter)(c_Spark_kIMaxAccum_0 + slotOfst*2), &(adv->maxI));
				LV_REVSparkSetAttribute(refPtr, (c_Spark_ConfigParameter)(c_Spark_kDFilter_0 + slotOfst*8), &(adv->dFilter));
				LV_REVSparkSetAttribute(refPtr, (c_Spark_ConfigParameter)(c_Spark_kAllowedClosedLoopError_0 + slotOfst*2), &(adv->errLimit));
			}
			break;

			case 0x216: // PIDModel_3
				slotOfst ++;
			case 0x212: // PIDModel_2
				slotOfst ++;
			case 0x20E: // PIDModel_1
				slotOfst ++;
			case 0x20a: { // PIDModel_0	-- cluster { 5, "kS" = fD, "kA" = fD, "kG"=fD, "kCos"=fD, "Cos Ratio"=fD }
				PIDMdlType *model = (PIDMdlType*)ptr;
				model += slotOfst;
				LV_REVSparkSetAttribute(refPtr, (c_Spark_ConfigParameter)(c_Spark_kS_0 + slotOfst*5), &(model->kS));
				LV_REVSparkSetAttribute(refPtr, (c_Spark_ConfigParameter)(c_Spark_kA_0 + slotOfst*5), &(model->kA));
				LV_REVSparkSetAttribute(refPtr, (c_Spark_ConfigParameter)(c_Spark_kG_0 + slotOfst*5), &(model->kG));
				LV_REVSparkSetAttribute(refPtr, (c_Spark_ConfigParameter)(c_Spark_kCos_0 + slotOfst*5), &(model->kCos));
				LV_REVSparkSetAttribute(refPtr, (c_Spark_ConfigParameter)(c_Spark_kCosRatio_0 + slotOfst*5), &(model->kCosRatio));
			}
			break;
			case 0x225: // MAXMotionConfig_3	
				slotOfst ++;
			case 0x224: // MAXMotionConfig_2	
				slotOfst ++;
			case 0x223: // MAXMotionConfig_1	
				slotOfst ++;
			case 0x222: { // MAXMotionConfig_0	-- cluster {5, "Cruise Velocity"=fD, "Accel Limit"=fD, "Jerk Limit"=fD, "Goal Error"=fD, "Mode"=uL}
				MAXCfgType *mmCfg = (MAXCfgType*)ptr;
				mmCfg += slotOfst;
				LV_REVSparkSetAttribute(refPtr, (c_Spark_ConfigParameter)(c_Spark_kMAXMotionCruiseVelocity_0 + slotOfst*5), &(mmCfg->vel));
				LV_REVSparkSetAttribute(refPtr, (c_Spark_ConfigParameter)(c_Spark_kMAXMotionMaxAccel_0 + slotOfst*5), &(mmCfg->accel));
				LV_REVSparkSetAttribute(refPtr, (c_Spark_ConfigParameter)(c_Spark_kMAXMotionMaxJerk_0 + slotOfst*5), &(mmCfg->jerk));
				LV_REVSparkSetAttribute(refPtr, (c_Spark_ConfigParameter)(c_Spark_kMAXMotionAllowedProfileError_0 + slotOfst*5), &(mmCfg->goal));
				LV_REVSparkSetAttribute(refPtr, (c_Spark_ConfigParameter)(c_Spark_kMAXMotionPositionMode_0 + slotOfst*5), &(mmCfg->mode));
			}
			break;

			default:
			break;
		}
	}

}





int32 LV_REVSparkGetAttribute(REVMotorRef refPtr, c_Spark_ConfigParameter id, void *ptr) {
	c_REVLib_ErrorCode revErr;
	int32_t i32Val;
	uint32_t u32Val;
	float f32Val;
	uint8_t boolVal;
	c_Spark_handle hdl;

		// get hdl from refPtr return if doesn't work

	// this property directly maps to a REV property, so use the above table
	if(id <= c_Spark_kDetachedEncoderDeviceID ) { // better with a sentinel
		switch(paramInfo[id].type) {
			default:
			case c_REVLib_kUnused:
			// debug printing, return error ???
			break;

			case c_REVLib_kInt32:
			revErr = c_Spark_GetParameterInt32(hdl,id, &i32Val);
			*(int32*)ptr = i32Val;
			break;
			
			case c_REVLib_kUint32:
			revErr = c_Spark_GetParameterUint32(hdl, id, &u32Val);
			*(uInt32*)ptr = u32Val;
			break;

			case c_REVLib_kFloat32:
			revErr = c_Spark_GetParameterFloat32(hdl, id, &f32Val);
			*(float*)ptr = f32Val;
			break;

			case c_REVLib_kBool:
			revErr = c_Spark_GetParameterBool(hdl, id, &boolVal);
			*(int8*)ptr = boolVal;
			break;

		}
	}
	else { // property wraps other function calls or aggregates multiple properties into a cluster

#if 1
	c_Spark_PeriodicStatus0 status0;
	c_Spark_PeriodicStatus1 status1;
	c_Spark_PeriodicStatus2 status2;
	c_Spark_PeriodicStatus3 status3;
	c_Spark_PeriodicStatus4 status4;
	c_Spark_PeriodicStatus5 status5;
	c_Spark_PeriodicStatus6 status6;
	c_Spark_PeriodicStatus7 status7;
	c_Spark_PeriodicStatus8 status8;
	c_Spark_PeriodicStatus9 status9;
	int32 slotOfst = 0;


	switch(id) {
		case 0x200: { // VoltageCompensation -- Spark Voltage Compensation.ctl
			vCompType *vc = (vCompType*)ptr;
			LV_REVSparkGetAttribute(refPtr, c_Spark_kVoltageCompensationMode, &(vc->mode));
			LV_REVSparkGetAttribute(refPtr, c_Spark_kCompensatedNominalVoltage, &(vc->nominal));
		}
		break;

		case 0x201: // MotorOutputCurrent
			revErr =  c_Spark_GetPeriodicStatus0(hdl, &status0);
			*(float64*)ptr = status0.current;
		break;

		case 0x202: // MotorTemperature
			revErr =  c_Spark_GetPeriodicStatus0(hdl, &status0);
			*(float64*)ptr = status0.motorTemperature;
			break;

		case 0x203: // MotorBusVoltage	
			revErr =  c_Spark_GetPeriodicStatus0(hdl, &status0);
			*(float64*)ptr = status0.voltage;
			break;

		case 0x204: // MotorAppliedOutput
			revErr =  c_Spark_GetPeriodicStatus0(hdl, &status0);
			*(float64*)ptr = status0.appliedOutput;
		break;

		case 0x205: {// FollowerModeInfo	-- cluster {2, "Leader CAN ID"=uL, "Inverted"=bool}
			followType *follow = (followType*)ptr;
			LV_REVSparkGetAttribute(refPtr, c_Spark_kFollowerModeLeaderId, &(follow->leader));
			LV_REVSparkGetAttribute(refPtr, c_Spark_kFollowerModeIsInverted, &(follow->invert));
		}
		break;

		case 0x206: { // PositionPIDWrap	-- cluster { 3, "Enable" = bool, "Min" = fD, "Max"=fD }
			posPIDWrapType *pos = (posPIDWrapType*)ptr;
			LV_REVSparkGetAttribute(refPtr, c_Spark_kPositionPIDWrapEnable, &(pos->enable));
			LV_REVSparkGetAttribute(refPtr, c_Spark_kPositionPIDMinInput, &(pos->min));
			LV_REVSparkGetAttribute(refPtr, c_Spark_kPositionPIDMaxInput, &(pos->max));
		}
		break;


		// REV pidf slots are 8 propIds apart, different from LV attrIDs
		case 0x213: // PIDF_3	
			slotOfst ++;
		case 0x20f: // PIDF_2	
			slotOfst ++;
		case 0x20b: // PIDF_1	
			slotOfst ++;
		case 0x207: { // PIDF_0	-- cluster { 4, "kP" = fD, "kI" = fD, "kD"=fD, "kF"=fD }
			PIDFType *tune = (PIDFType*)ptr;
			tune += slotOfst;
			LV_REVSparkGetAttribute(refPtr, (c_Spark_ConfigParameter)(c_Spark_kP_0 + slotOfst*8), &(tune->kP));
			LV_REVSparkGetAttribute(refPtr, (c_Spark_ConfigParameter)(c_Spark_kI_0 + slotOfst*8), &(tune->kI));
			LV_REVSparkGetAttribute(refPtr, (c_Spark_ConfigParameter)(c_Spark_kD_0 + slotOfst*8), &(tune->kD));
			LV_REVSparkGetAttribute(refPtr, (c_Spark_ConfigParameter)(c_Spark_kV_0 + slotOfst*8), &(tune->kF));
		}
		break;

		case 0x214: // PIDOutRng_3
			slotOfst ++;
		case 0x210: // PIDOutRng_2
			slotOfst ++;
		case 0x20C: // PIDOutRng_1
			slotOfst ++;
		case 0x208: { // PIDOutRng_0	-- cluster { 2, "min" = fD, "max" = fD }
			PIDRngType *rng = (PIDRngType*)ptr;
			rng += slotOfst;
			LV_REVSparkGetAttribute(refPtr, (c_Spark_ConfigParameter)(c_Spark_kOutputMin_0 + slotOfst*8), &(rng->min));
			LV_REVSparkGetAttribute(refPtr, (c_Spark_ConfigParameter)(c_Spark_kOutputMax_0 + slotOfst*8), &(rng->max));
		}
		break;

		case 0x215: // PIDAdv_3
			slotOfst ++;
		case 0x211: // PIDAdv_2
			slotOfst ++;
		case 0x20D: // PIDAdv_1
			slotOfst ++;
		case 0x209: { // PIDAdv_0	-- { 4, "i Zone" = fD, "max I Accum" = fD, "d Filter"=fD, "error Limit"=fD }
			PIDAdvType *adv = (PIDAdvType*)ptr;
			adv += slotOfst;
			LV_REVSparkGetAttribute(refPtr, (c_Spark_ConfigParameter)(c_Spark_kIZone_0 + slotOfst*8), &(adv->iZone));
			LV_REVSparkGetAttribute(refPtr, (c_Spark_ConfigParameter)(c_Spark_kIMaxAccum_0 + slotOfst*2), &(adv->maxI));
			LV_REVSparkGetAttribute(refPtr, (c_Spark_ConfigParameter)(c_Spark_kDFilter_0 + slotOfst*8), &(adv->dFilter));
			LV_REVSparkGetAttribute(refPtr, (c_Spark_ConfigParameter)(c_Spark_kAllowedClosedLoopError_0 + slotOfst*2), &(adv->errLimit));
		}
		break;

		case 0x216: // PIDModel_3
			slotOfst ++;
		case 0x212: // PIDModel_2
			slotOfst ++;
		case 0x20E: // PIDModel_1
			slotOfst ++;
		case 0x20a: { // PIDModel_0	-- cluster { 5, "kS" = fD, "kA" = fD, "kG"=fD, "kCos"=fD, "Cos Ratio"=fD }
			PIDMdlType *model = (PIDMdlType*)ptr;
			model += slotOfst;
			LV_REVSparkGetAttribute(refPtr, (c_Spark_ConfigParameter)(c_Spark_kS_0 + slotOfst*5), &(model->kS));
			LV_REVSparkGetAttribute(refPtr, (c_Spark_ConfigParameter)(c_Spark_kA_0 + slotOfst*5), &(model->kA));
			LV_REVSparkGetAttribute(refPtr, (c_Spark_ConfigParameter)(c_Spark_kG_0 + slotOfst*5), &(model->kG));
			LV_REVSparkGetAttribute(refPtr, (c_Spark_ConfigParameter)(c_Spark_kCos_0 + slotOfst*5), &(model->kCos));
			LV_REVSparkGetAttribute(refPtr, (c_Spark_ConfigParameter)(c_Spark_kCosRatio_0 + slotOfst*5), &(model->kCosRatio));
		}
		break;

		case 0x217: // EncoderPositionValue	
			revErr =  c_Spark_GetPeriodicStatus2(hdl, &status2);
			*(float64*)ptr = status2.primaryEncoderPosition;
		break;

		case 0x218: // EncoderVelocityValue	
			revErr =  c_Spark_GetPeriodicStatus2(hdl, &status2);
			*(float64*)ptr = status2.primaryEncoderVelocity;
		break;

		case 0x22B: // DutyCyclePositionValue	
			revErr =  c_Spark_GetPeriodicStatus5(hdl, &status5);
			*(float64*)ptr = status5.dutyCycleEncoderPosition;
		break;

		case 0x22C: // DutyCycleVelocityValue	
			revErr =  c_Spark_GetPeriodicStatus5(hdl, &status5);
			*(float64*)ptr = status5.dutyCycleEncoderVelocity;
		break;

		case 0x219: // LimitSwitchFwdValue	
			revErr =  c_Spark_GetPeriodicStatus0(hdl, &status0);
			*(uInt8*)ptr = status0.hardForwardLimitReached;
		break;

		case 0x21a: // LimitSwitchRevValue	
			revErr =  c_Spark_GetPeriodicStatus0(hdl, &status0);
			*(uInt8*)ptr = status0.hardReverseLimitReached;
		break;

		case 0x21b: // SoftLimitForwardValue	
			revErr =  c_Spark_GetPeriodicStatus0(hdl, &status0);
			*(uInt8*)ptr = status0.softForwardLimitReached;

		break;

		case 0x21c: // SoftLimitReverseValue	
			revErr =  c_Spark_GetPeriodicStatus0(hdl, &status0);
			*(uInt8*)ptr = status0.softReverseLimitReached;
		break;

		case 0x21d: // AnalogPositionValue	
			revErr =  c_Spark_GetPeriodicStatus3(hdl, &status3);
			*(float64*)ptr = status3.analogPosition;
		break;

		case 0x21e: // AnalogVelocityValue	
			revErr =  c_Spark_GetPeriodicStatus3(hdl, &status3);
			*(float64*)ptr = status3.analogVelocity;
		break;

		case 0x21f: // AnalogVoltageValue	
			revErr =  c_Spark_GetPeriodicStatus3(hdl, &status3);
			*(float64*)ptr = status3.analogVoltage;
		break;

		case 0x220: // AltEncoderPositionValue	
			revErr =  c_Spark_GetPeriodicStatus4(hdl, &status4);
			*(float64*)ptr = status4.externalOrAltEncoderPosition;
		break;

		case 0x221: // AltEncoderVelocityValue	
			revErr =  c_Spark_GetPeriodicStatus4(hdl, &status4);
			*(float64*)ptr = status4.externalOrAltEncoderVelocity;
		break;

		case 0x225: // MAXMotionConfig_3	
			slotOfst ++;
		case 0x224: // MAXMotionConfig_2	
			slotOfst ++;
		case 0x223: // MAXMotionConfig_1	
			slotOfst ++;
		case 0x222: { // MAXMotionConfig_0	-- cluster {5, "Cruise Velocity"=fD, "Accel Limit"=fD, "Jerk Limit"=fD, "Goal Error"=fD, "Mode"=uL}
			MAXCfgType *mmCfg = (MAXCfgType*)ptr;
			mmCfg += slotOfst;
			LV_REVSparkGetAttribute(refPtr, (c_Spark_ConfigParameter)(c_Spark_kMAXMotionCruiseVelocity_0 + slotOfst*5), &(mmCfg->vel));
			LV_REVSparkGetAttribute(refPtr, (c_Spark_ConfigParameter)(c_Spark_kMAXMotionMaxAccel_0 + slotOfst*5), &(mmCfg->accel));
			LV_REVSparkGetAttribute(refPtr, (c_Spark_ConfigParameter)(c_Spark_kMAXMotionMaxJerk_0 + slotOfst*5), &(mmCfg->jerk));
			LV_REVSparkGetAttribute(refPtr, (c_Spark_ConfigParameter)(c_Spark_kMAXMotionAllowedProfileError_0 + slotOfst*5), &(mmCfg->goal));
			LV_REVSparkGetAttribute(refPtr, (c_Spark_ConfigParameter)(c_Spark_kMAXMotionPositionMode_0 + slotOfst*5), &(mmCfg->mode));
		}
		break;

		case 0x226: { // MotorFaults	-- other, Motor Type, Sensor, CAN, Temp, GateDrv, Esc EEProm, Firmware
			MotorFaultsType *faults = (MotorFaultsType*)ptr;
			revErr =  c_Spark_GetPeriodicStatus1(hdl, &status1);
			faults->other = status1.otherFault;
			faults->type = status1.motorTypeFault;
			faults->sensor = status1.sensorFault;
			faults->can = status1.canFault;
			faults->temp = status1.temperatureFault;
			faults->gateDrv = status1.drvFault;
			faults->escEEProm = status1.escEepromFault;
			faults->firmware = status1.firmwareFault;
		}
		break;
		case 0x227: { // MotorStickyFaults -- "<vilib>:REV Custom Controls\Spark Faults2.ctl"
			MotorFaultsType *faults = (MotorFaultsType*)ptr;
			revErr =  c_Spark_GetPeriodicStatus1(hdl, &status1);
			faults->other = status1.otherStickyFault;
			faults->type = status1.motorTypeStickyFault;
			faults->sensor = status1.sensorStickyFault;
			faults->can = status1.canStickyFault;
			faults->temp = status1.temperatureStickyFault;
			faults->gateDrv = status1.drvStickyFault;
			faults->escEEProm = status1.escEepromStickyFault;
			faults->firmware = status1.firmwareStickyFault;
		}
		break;

		case 0x228: { // MotorWarnings -- Brownout, overCurrent, EscEEProm, Ext EEProm, Sensor, Stall, Has Reset, Other
			MotorWarnType *warn = (MotorWarnType*)ptr;
			revErr =  c_Spark_GetPeriodicStatus1(hdl, &status1);
			warn->brown = status1.brownoutWarning;
			warn->over = status1.overcurrentWarning;
			warn->escEEProm = status1.escEepromWarning;
			warn->ext = status1.extEepromWarning;
			warn->sensor = status1.sensorWarning;
			warn->stall = status1.stallWarning;
			warn->hasReset = status1.hasResetWarning;
			warn->other = status1.otherWarning;
		}
		break;

		case 0x229: { // MotorStickyWarnings -- Brownout, overCurrent, EscEEProm, Ext EEProm, Sensor, Stall, Has Reset, Other
			MotorWarnType *warn = (MotorWarnType*)ptr;
			revErr =  c_Spark_GetPeriodicStatus1(hdl, &status1);
			warn->brown = status1.brownoutStickyWarning;
			warn->over = status1.overcurrentStickyWarning;
			warn->escEEProm = status1.escEepromStickyWarning;
			warn->ext = status1.extEepromStickyWarning;
			warn->sensor = status1.sensorStickyWarning;
			warn->stall = status1.stallStickyWarning;
			warn->hasReset = status1.hasResetStickyWarning;
			warn->other = status1.otherStickyWarning;
		}
		break;

		case 0x22A: { // MotorDetails	-- CANID, MajorVers, MinorVers, Type, InputMode, ProdID, kV, PolePairs, CommutationAdv
			MotorDetailsType *details = (MotorDetailsType*)ptr;
			LV_REVSparkGetAttribute(refPtr, c_Spark_kCANID, &(details->can));
			LV_REVSparkGetAttribute(refPtr, c_Spark_kDeviceMajorVersion, &(details->major));
			LV_REVSparkGetAttribute(refPtr, c_Spark_kDeviceMinorVersion, &(details->minor));
			LV_REVSparkGetAttribute(refPtr, c_Spark_kMotorType, &(details->type));
			LV_REVSparkGetAttribute(refPtr, c_Spark_kInputMode, &(details->mode));
			LV_REVSparkGetAttribute(refPtr, c_Spark_kProductId, &(details->prod));
			LV_REVSparkGetAttribute(refPtr, c_Spark_kMotorKv, &(details->kV));
			LV_REVSparkGetAttribute(refPtr, c_Spark_kPolePairs, &(details->pairs));
			LV_REVSparkGetAttribute(refPtr, c_Spark_kCommutationAdvance, &(details->adv));
		}
		break;
		default:
		// debugprintf or other, and return error ???
		break;
		}
#endif
	}
}



#if defined(_WIN32) && !defined(_WIN64)
#pragma pack(pop)
#endif
















