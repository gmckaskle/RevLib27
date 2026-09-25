/*
 * Copyright (c) 2024-2026 REV Robotics
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of REV Robotics nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * This file is auto-generated. Do NOT modify it directly.
 * See https://github.com/REVrobotics/SparkParameters
 */

#include "rev/CANSparkParameters.h"

#include <array>
#include <string>

namespace {

struct private_parameter_table_entry_t {
    c_Spark_ConfigParameter id;
    c_REVLib_ParameterType type;
    uint32_t defaultValue;
    std::string name;
};

const std::array<private_parameter_table_entry_t, c_Spark_NumParameters>
    s_Spark_ParameterTable = {{
        {c_Spark_kCANID, c_REVLib_kUint32, 0, "CAN ID: CAN bus device identifier in accordance with the FRC CAN Specification. Refer to: https://docs.wpilib.org/en/stable/docs/software/can-devices/can-addressing.html "},
        {c_Spark_kInputMode, c_REVLib_kUint32, c_Spark_kInputMode_PWM, "Input Mode: The Spark's current input interface for communication and control. Can not be modified directly."},
        {c_Spark_kMotorType, c_REVLib_kUint32, c_Spark_kMotorType_BRUSHLESS, "Motor Type: The motor type of the motor connected to the Spark."},
        {c_Spark_kCommutationAdvance, c_REVLib_kFloat32, 0x00000000, "Commutation Advance: The commutation hall state offset in degrees. Currently works in increments of 30 degrees and may help when the hall states & motor coils do not align with the commutation sequence."},
        {c_Spark_kUnused, c_REVLib_kUnused, 0, "Sensor Type - Unused"},
        {c_Spark_kControlType, c_REVLib_kUint32, c_Spark_kControlType_DUTY_CYCLE, "Control Type: The setpoint control type last accepted by the Spark. This should not be modified directly."},
        {c_Spark_kIdleMode, c_REVLib_kUint32, c_Spark_kIdleMode_COAST, "Idle Mode: Spark's disabled behavior type."},
        {c_Spark_kInputDeadband, c_REVLib_kFloat32, 0x3d4ccccd, "Input Deadband: The proportion size of the PWM input period region for idling/disabling the motor in PWM mode."},
        {c_Spark_kUnused, c_REVLib_kUnused, 0, "Reserved 8"},
        {c_Spark_kClosedLoopControlSensor, c_REVLib_kUint32, c_Spark_kSensor_NONE, "Closed Loop Control Sensor: The reference/feedback sensor type used for closed loop motor control."},
        {c_Spark_kPolePairs, c_REVLib_kUint32, 7, "Pole Pairs: Number of pole pairs within the motor connected to the Spark. Affects position and speed calculations as it determines the expected number of hall states to complete a full rotation"},
        {c_Spark_kCurrentChop, c_REVLib_kFloat32, 0x42e60000, "Current Chop: The phase current threshold for activating the current chopping protection mechanism. Current chopping will attempt to average out the phase currents of the motor to below the threshold by disabling the motor for the number of 'Current Chop Cycles' and resume afterwards."},
        {c_Spark_kCurrentChopCycles, c_REVLib_kUint32, 0, "Current Chop Cycles: The number of cycles that the motor will brake for when the motor phase currents reach above the current chop threshold."},
        {c_Spark_kP_0, c_REVLib_kFloat32, 0x00000000, "P 0: PID slot 0 error proportion factor for PID control. "},
        {c_Spark_kI_0, c_REVLib_kFloat32, 0x00000000, "I 0: PID slot 0 error integral factor for PID Control."},
        {c_Spark_kD_0, c_REVLib_kFloat32, 0x00000000, "D 0: PID slot 0 error derivative factor for PID Control."},
        {c_Spark_kV_0, c_REVLib_kFloat32, 0x00000000, "V 0: PID slot 0 velocity feed forward factor. Factor that adjusts the velocity output by the PID controller."},
        {c_Spark_kIZone_0, c_REVLib_kFloat32, 0x00000000, "IZone 0: PID slot 0 integral zone determines the upper limit error value for allowing the PID integral factor I to integrate. Setting this to 0 will disable the integral zone."},
        {c_Spark_kDFilter_0, c_REVLib_kFloat32, 0x00000000, "D Filter 0: PID slot 0 derivative filter factor. Sets the influence percentage of the newly calculated derivative in the calculation of the D factor. Values closer to 0 will result in stronger low-pass filtering. Setting the value outside the range of (0, 1] will disable the filter."},
        {c_Spark_kOutputMin_0, c_REVLib_kFloat32, 0xbf800000, "Output Min 0: PID slot 0 output minimum limit."},
        {c_Spark_kOutputMax_0, c_REVLib_kFloat32, 0x3f800000, "Output Max 0: PID slot 0 output maximum limit."},
        {c_Spark_kP_1, c_REVLib_kFloat32, 0x00000000, "P 1: PID slot 1 error proportion factor for PID control."},
        {c_Spark_kI_1, c_REVLib_kFloat32, 0x00000000, "I 1: PID slot 1 error integral factor for PID Control."},
        {c_Spark_kD_1, c_REVLib_kFloat32, 0x00000000, "D 1: PID slot 1 error derivative factor for PID Control."},
        {c_Spark_kV_1, c_REVLib_kFloat32, 0x00000000, "V 1: PID slot 1 velocity feed forward factor. Factor that adjusts the velocity output by the PID controller."},
        {c_Spark_kIZone_1, c_REVLib_kFloat32, 0x00000000, "IZone 1: PID slot 1 integral zone determines the minimum error value needed before allowing the PID integral factor I to integrate. Setting this to 0 will disable the integral zone."},
        {c_Spark_kDFilter_1, c_REVLib_kFloat32, 0x00000000, "D Filter 1: PID slot 1 derivative filter factor. Sets the influence percentage of the newly calculated derivative in the calculation of the D factor. Values closer to 0 will result in stronger low-pass filtering. Setting the value outside the range of (0, 1] will disable the filter."},
        {c_Spark_kOutputMin_1, c_REVLib_kFloat32, 0xbf800000, "Output Min 1: PID slot 1 output minimum limit."},
        {c_Spark_kOutputMax_1, c_REVLib_kFloat32, 0x3f800000, "Output Max 1: PID slot 1 output maximum limit."},
        {c_Spark_kP_2, c_REVLib_kFloat32, 0x00000000, "P 2: PID slot 2 error proportion factor for PID control."},
        {c_Spark_kI_2, c_REVLib_kFloat32, 0x00000000, "I 2: PID slot 2 error integral factor for PID Control."},
        {c_Spark_kD_2, c_REVLib_kFloat32, 0x00000000, "D 2: PID slot 2 error derivative factor for PID Control."},
        {c_Spark_kV_2, c_REVLib_kFloat32, 0x00000000, "V 2: PID slot 2 velocity feed forward factor. Factor that adjusts the velocity output by the PID controller."},
        {c_Spark_kIZone_2, c_REVLib_kFloat32, 0x00000000, "IZone 2: PID slot 2 integral zone determines the minimum error value needed before allowing the PID integral factor I to integrate. Setting this to 0 will disable the integral zone."},
        {c_Spark_kDFilter_2, c_REVLib_kFloat32, 0x00000000, "D Filter 2: PID slot 2 derivative filter factor. Sets the influence percentage of the newly calculated derivative in the calculation of the D factor. Values closer to 0 will result in stronger low-pass filtering. Setting the value outside the range of (0, 1] will disable the filter."},
        {c_Spark_kOutputMin_2, c_REVLib_kFloat32, 0xbf800000, "Output Min 2: PID slot 2 output minimum limit."},
        {c_Spark_kOutputMax_2, c_REVLib_kFloat32, 0x3f800000, "Output Max 2: PID slot 2 output maximum limit."},
        {c_Spark_kP_3, c_REVLib_kFloat32, 0x00000000, "P 3: PID slot 3 error proportion factor for PID control."},
        {c_Spark_kI_3, c_REVLib_kFloat32, 0x00000000, "I 3: PID slot 3 error integral factor for PID Control."},
        {c_Spark_kD_3, c_REVLib_kFloat32, 0x00000000, "D 3: PID slot 3 error derivative factor for PID Control."},
        {c_Spark_kV_3, c_REVLib_kFloat32, 0x00000000, "V 3: PID slot 3 velocity feed forward factor. Factor that adjusts the velocity output by the PID controller."},
        {c_Spark_kIZone_3, c_REVLib_kFloat32, 0x00000000, "IZone 3: PID slot 3 integral zone determines the minimum error value needed before allowing the PID integral factor I to integrate. Setting this to 0 will disable the integral zone."},
        {c_Spark_kDFilter_3, c_REVLib_kFloat32, 0x00000000, "D Filter 3: PID slot 3 derivative filter factor. Sets the influence percentage of the newly calculated derivative in the calculation of the D factor. Values closer to 0 will result in stronger low-pass filtering. Setting the value outside the range of (0, 1] will disable the filter."},
        {c_Spark_kOutputMin_3, c_REVLib_kFloat32, 0xbf800000, "Output Min 3: PID slot 3 output minimum limit."},
        {c_Spark_kOutputMax_3, c_REVLib_kFloat32, 0x3f800000, "Output Max 3: PID slot 3 output maximum limit."},
        {c_Spark_kInverted, c_REVLib_kBool, false, "Inverted: Sets whether the direction the motor is driven is inverted."},
        {c_Spark_kUnused, c_REVLib_kUnused, 0, "Reserved 46"},
        {c_Spark_kUnused, c_REVLib_kUnused, 0, "Reserved 47"},
        {c_Spark_kUnused, c_REVLib_kUnused, 0, "Reserved 48"},
        {c_Spark_kUnused, c_REVLib_kUnused, 0, "Reserved 49"},
        {c_Spark_kLimitSwitchFwdPolarity, c_REVLib_kBool, false, "Limit Switch Fwd Polarity: Sets whether the hardware forward limit switch is normally-open or normally-closed."},
        {c_Spark_kLimitSwitchRevPolarity, c_REVLib_kBool, false, "Limit Switch Rev Polarity: Sets whether the hardware reverse limit switch is normally-open or normally-closed."},
        {c_Spark_kHardLimitFwdEn, c_REVLib_kUint32, c_Spark_kLimitSwBehavior_STOP_MOVING_MOTOR, "Hard Limit Fwd En: The forward limit switch mode. Sets the behavior for when the limit switch is triggered."},
        {c_Spark_kHardLimitRevEn, c_REVLib_kUint32, c_Spark_kLimitSwBehavior_STOP_MOVING_MOTOR, "Hard Limit Rev En: The reverse limit switch mode. Sets the behavior for when the limit switch is triggered."},
        {c_Spark_kSoftLimitFwdEn, c_REVLib_kBool, false, "Soft Limit Fwd En: Sets whether the soft forward limit is enabled."},
        {c_Spark_kSoftLimitRevEn, c_REVLib_kBool, false, "Soft Limit Rev En: Sets whether the soft reverse limit is enabled."},
        {c_Spark_kOpenLoopRampRate, c_REVLib_kFloat32, 0x00000000, "Open Loop Ramp Rate: Ramp rate for the duty cycle applied to the motor with open-loop control. This value is the desired change in duty cycle per second."},
        {c_Spark_kLegacyFollowerID, c_REVLib_kUint32, 0, "Legacy Follower ID: [Deprecated] CAN message ID of the leader Spark to be followed."},
        {c_Spark_kLegacyFollowerConfig, c_REVLib_kUint32, 0, "Legacy Follower Config: [Deprecated] Follower mode configuration settings."},
        {c_Spark_kSmartCurrentStallLimit, c_REVLib_kUint32, 80, "Smart Current Stall Limit: The smart phase current upper limit."},
        {c_Spark_kSmartCurrentFreeLimit, c_REVLib_kUint32, 20, "Smart Current Free Limit: The expected phase current when the motor is free spinning. Used when the motor RPM has reached above the 'Smart Current Config' RPM threshold and will linearly reduce the current limit to the free limit current configured."},
        {c_Spark_kSmartCurrentConfig, c_REVLib_kUint32, 10000, "Smart Current Config: The Smart Current configuration settings. Currently sets the RPM threshold that will begin reducing the phase current upper limit to the 'Smart Current Free Limit'. Set to an RPM value higher than the motor free speed to disable this behavior."},
        {c_Spark_kSmartCurrentReserved, c_REVLib_kUint32, 0, "Smart Current Reserved: Reserved Smart Current parameter."},
        {c_Spark_kMotorKv, c_REVLib_kUint32, 480, "Motor Kv: Expected Motor Kv coefficient. Represents the RPM generated for every volt applied to the motor."},
        {c_Spark_kUnused, c_REVLib_kUnused, 0, "Reserved 64"},
        {c_Spark_kUnused, c_REVLib_kUnused, 0, "Reserved 65"},
        {c_Spark_kUnused, c_REVLib_kUnused, 0, "Reserved 66"},
        {c_Spark_kUnused, c_REVLib_kUnused, 0, "Reserved 67"},
        {c_Spark_kUnused, c_REVLib_kUnused, 0, "Reserved 68"},
        {c_Spark_kEncoderCountsPerRev, c_REVLib_kUint32, 8192, "Encoder Counts Per Rev: Sets the Primary Encoder's counts per revolution. Sparks only use this when in brushed mode. Note: Spark Flex will need a Flex Dock in order run in brushed mode."},
        {c_Spark_kEncoderAverageDepth, c_REVLib_kUint32, 8, "Encoder Average Depth: Configures the Primary Encoder's velocity averaging filter size."},
        {c_Spark_kEncoderSampleDelta, c_REVLib_kUint32, 20, "Encoder Sample Delta: Configures the Primary Encoder's time delta between collected samples for velocity averaging in increments of 500us."},
        {c_Spark_kEncoderInverted, c_REVLib_kBool, false, "Encoder Inverted: Sets whether the encoder direction is inverted."},
        {c_Spark_kUnused, c_REVLib_kUnused, 0, "Reserved 73"},
        {c_Spark_kVoltageCompensationMode, c_REVLib_kUint32, c_Spark_kVoltageCompMode_NO_VOLTAGE_COMP, "Voltage Compensation Mode: Sets the type of voltage compensation. 'Nominal Voltage Compensation Mode' will scale the applied output at the current voltage level to the configured nominal voltage level. This helps with driving consistency as voltage levels change."},
        {c_Spark_kCompensatedNominalVoltage, c_REVLib_kFloat32, 0x00000000, "Compensated Nominal Voltage: Sets the nominal voltage used in 'Nominal Voltage Compensation Mode' that the device will try to operate at."},
        {c_Spark_kUnused, c_REVLib_kUnused, 0, "Reserved 76"},
        {c_Spark_kUnused, c_REVLib_kUnused, 0, "Reserved 77"},
        {c_Spark_kUnused, c_REVLib_kUnused, 0, "Reserved 78"},
        {c_Spark_kUnused, c_REVLib_kUnused, 0, "Reserved 79"},
        {c_Spark_kUnused, c_REVLib_kUnused, 0, "Reserved 80"},
        {c_Spark_kUnused, c_REVLib_kUnused, 0, "Reserved 81"},
        {c_Spark_kUnused, c_REVLib_kUnused, 0, "Reserved 82"},
        {c_Spark_kUnused, c_REVLib_kUnused, 0, "Reserved 83"},
        {c_Spark_kUnused, c_REVLib_kUnused, 0, "Reserved 84"},
        {c_Spark_kUnused, c_REVLib_kUnused, 0, "Reserved 85"},
        {c_Spark_kUnused, c_REVLib_kUnused, 0, "Reserved 86"},
        {c_Spark_kUnused, c_REVLib_kUnused, 0, "Reserved 87"},
        {c_Spark_kUnused, c_REVLib_kUnused, 0, "Reserved 88"},
        {c_Spark_kUnused, c_REVLib_kUnused, 0, "Reserved 89"},
        {c_Spark_kUnused, c_REVLib_kUnused, 0, "Reserved 90"},
        {c_Spark_kUnused, c_REVLib_kUnused, 0, "Reserved 91"},
        {c_Spark_kUnused, c_REVLib_kUnused, 0, "Reserved 92"},
        {c_Spark_kUnused, c_REVLib_kUnused, 0, "Reserved 93"},
        {c_Spark_kUnused, c_REVLib_kUnused, 0, "Reserved 94"},
        {c_Spark_kUnused, c_REVLib_kUnused, 0, "Reserved 95"},
        {c_Spark_kIMaxAccum_0, c_REVLib_kFloat32, 0x00000000, "I Max Accum 0: PID Slot 0: Maximum value that the integrated value I can reach for PID control."},
        {c_Spark_kAllowedClosedLoopError_0, c_REVLib_kFloat32, 0x00000000, "Allowed Closed Loop Error 0: PID Slot 0: Range of error from the target value."},
        {c_Spark_kUnused, c_REVLib_kUnused, 0, "Reserved 98"},
        {c_Spark_kUnused, c_REVLib_kUnused, 0, "Reserved 99"},
        {c_Spark_kIMaxAccum_1, c_REVLib_kFloat32, 0x00000000, "I Max Accum 1: PID Slot 1: Maximum value that the integrated value I can reach for PID control."},
        {c_Spark_kAllowedClosedLoopError_1, c_REVLib_kFloat32, 0x00000000, "Allowed Closed Loop Error 1: PID Slot 1: Range of error from the target value."},
        {c_Spark_kUnused, c_REVLib_kUnused, 0, "Reserved 102"},
        {c_Spark_kUnused, c_REVLib_kUnused, 0, "Reserved 103"},
        {c_Spark_kIMaxAccum_2, c_REVLib_kFloat32, 0x00000000, "I Max Accum 2: PID Slot 2: Maximum value that the integrated value I can reach for PID control."},
        {c_Spark_kAllowedClosedLoopError_2, c_REVLib_kFloat32, 0x00000000, "Allowed Closed Loop Error 2: PID Slot 2: Range of error from the target value."},
        {c_Spark_kUnused, c_REVLib_kUnused, 0, "Reserved 106"},
        {c_Spark_kUnused, c_REVLib_kUnused, 0, "Reserved 107"},
        {c_Spark_kIMaxAccum_3, c_REVLib_kFloat32, 0x00000000, "I Max Accum 3: PID Slot 3: Maximum value that the integrated value I can reach for PID control."},
        {c_Spark_kAllowedClosedLoopError_3, c_REVLib_kFloat32, 0x00000000, "Allowed Closed Loop Error 3: PID Slot 3: Range of error from the target value."},
        {c_Spark_kUnused, c_REVLib_kUnused, 0, "Reserved 110"},
        {c_Spark_kUnused, c_REVLib_kUnused, 0, "Reserved 111"},
        {c_Spark_kPositionConversionFactor, c_REVLib_kFloat32, 0x3f800000, "Position Conversion Factor: The Primary Encoder position conversion factor. Can be used for converting rotations to other units of measurement."},
        {c_Spark_kVelocityConversionFactor, c_REVLib_kFloat32, 0x3f800000, "Velocity Conversion Factor: The Primary Encoder velocity conversion factor. Can be used for converting rotations/min to other units of measurement."},
        {c_Spark_kClosedLoopRampRate, c_REVLib_kFloat32, 0x00000000, "Closed Loop Ramp Rate: Ramp rate for the duty cycle applied to the motor with closed-loop control. This value is the desired change in duty cycle per second."},
        {c_Spark_kSoftLimitForward, c_REVLib_kFloat32, 0x00000000, "Soft Limit Forward: Software implemented forward limit. Will stop the motor from moving forward after crossing the set value."},
        {c_Spark_kSoftLimitReverse, c_REVLib_kFloat32, 0x00000000, "Soft Limit Reverse: Software implemented reverse limit. Will stop the motor from moving reverse after crossing the set value."},
        {c_Spark_kUnused, c_REVLib_kUnused, 0, "Reserved 117"},
        {c_Spark_kUnused, c_REVLib_kUnused, 0, "Reserved 118"},
        {c_Spark_kAnalogPositionConversion, c_REVLib_kFloat32, 0x3f800000, "Analog Position Conversion: Analog position conversion factor. Helps convert rotation(s) to other units of measurement."},
        {c_Spark_kAnalogVelocityConversion, c_REVLib_kFloat32, 0x3f800000, "Analog Velocity Conversion: Analog sensor velocity conversion factor. Helps convert RPM to other units of measurement."},
        {c_Spark_kAnalogAverageDepth, c_REVLib_kUint32, 64, "Analog Average Depth: Configures the analog sensor's velocity averaging filter size."},
        {c_Spark_kAnalogSensorMode, c_REVLib_kUint32, c_Spark_kAnalogMode_ABSOLUTE, "Analog Sensor Mode: Sets the analog sensor position mode. In absolute, the sensor position will wrap around with the voltage read and is unaffected by the set position command. In relative, the sensor position difference between samples are summed up to produce the relative position and can be set through the set position command."},
        {c_Spark_kAnalogInverted, c_REVLib_kBool, 0, "Analog Inverted: Sets whether the analog sensor values are inverted."},
        {c_Spark_kAnalogSampleDelta, c_REVLib_kUint32, 200, "Analog Sample Delta: Configures the analog sensor's time delta between collected samples for velocity averaging in increments of 500us."},
        {c_Spark_kUnused, c_REVLib_kUnused, 0, "Reserved 125"},
        {c_Spark_kUnused, c_REVLib_kUnused, 0, "Reserved 126"},
        {c_Spark_kCompatibilityPortConfig, c_REVLib_kUint32, c_Spark_kCompatibilityPort_DEFAULT, "Compatibility Port Config: Note: This is a Spark MAX exclusive parameter. This sets whether the Data Port of the MAX is configured to DEFAULT mode or ALTERNATE ENCODER mode. In default mode, the data port is configured for both an absolute encoder and limit switches. In Alternate Encoder Mode, the data port is configured for quadrature encoder inputs, limit switches and an analog sensor. Please refer to the section 'How to Connect an Encoder' section at 'https://docs.revrobotics.com/brushless/spark-max/encoders#data-port-breakout-board' for determining the correct adapter boards to use."},
        {c_Spark_kAltEncoderCountsPerRev, c_REVLib_kUint32, 8192, "Alt Encoder Counts Per Rev: External/Alternate Encoder's counts per revolution."},
        {c_Spark_kAltEncoderAverageDepth, c_REVLib_kUint32, 8, "Alt Encoder Average Depth: The External/Alternate Encoder's velocity averaging filter size. Sets the number of samples used in the averaging filter."},
        {c_Spark_kAltEncoderSampleDelta, c_REVLib_kUint32, 20, "Alt Encoder Sample Delta: Configures the External/Alternate Encoder's time delta between collected samples for velocity averaging in increments of 500us."},
        {c_Spark_kAltEncoderInverted, c_REVLib_kBool, false, "Alt Encoder Inverted: Sets whether the External/Alternate Encoder's values are inverted."},
        {c_Spark_kAltEncoderPositionConversion, c_REVLib_kFloat32, 0x3f800000, "Alt Encoder Position Conversion: External/Alternate Encoder position conversion factor. Helps convert rotations to other units of measurement."},
        {c_Spark_kAltEncoderVelocityConversion, c_REVLib_kFloat32, 0x3f800000, "Alt Encoder Velocity Conversion: External/Alternate Encoder velocity conversion factor. Helps convert RPM to other units of measurement."},
        {c_Spark_kUnused, c_REVLib_kUnused, 0, "Reserved 134"},
        {c_Spark_kUnused, c_REVLib_kUnused, 0, "Reserved 135"},
        {c_Spark_kUnused, c_REVLib_kUnused, 0, "Uvw Sensor Sample Rate - Unused"},
        {c_Spark_kUnused, c_REVLib_kUnused, 0, "Uvw Sensor Average Depth - Unused"},
        {c_Spark_kNumParameters, c_REVLib_kUint32, 137, "Num Parameters: Number of parameters stored in internal EEPROM on the Spark, including unused/removed/deprecated values."},
        {c_Spark_kDutyCyclePositionFactor, c_REVLib_kFloat32, 0x3f800000, "Duty Cycle Position Factor: Duty-Cycle Encoder position conversion factor. Helps convert rotations to other units of measurement."},
        {c_Spark_kDutyCycleVelocityFactor, c_REVLib_kFloat32, 0x3f800000, "Duty Cycle Velocity Factor: Duty-Cycle Encoder velocity conversion factor. Helps convert RPM to other units of measurement."},
        {c_Spark_kDutyCycleInverted, c_REVLib_kBool, false, "Duty Cycle Inverted: Sets whether the Duty Cycle Encoder values are inverted."},
        {c_Spark_kUnused, c_REVLib_kUnused, 0, "Reserved 142"},
        {c_Spark_kDutyCycleAverageDepth, c_REVLib_kUint32, 7, "Duty Cycle Average Depth: The Duty Cycle sensor's velocity averaging filter size. Sets the number of samples used in the averaging filter as a power of 2. For example, a value of 3 sets the filter size to 2^(3) = 8 samples."},
        {c_Spark_kUnused, c_REVLib_kUnused, 0, "Reserved 144"},
        {c_Spark_kDutyCycleOffsetLegacy, c_REVLib_kFloat32, 0x00000000, "Duty Cycle Offset Legacy: Duty Cycle sensor's legacy method of setting duty cycle offset. This parameter sets the offset after inversion is accounted for."},
        {c_Spark_kDutyCycleRangeOffset, c_REVLib_kFloat32, 0x00000000, "Duty Cycle Range Offset: Sets the Duty Cycle Encoder Range offset about zero. Moves the range between (-1, 0] and [0, 1), instead of the default range [0, 1), assuming the default units of rotations."},
        {c_Spark_kUnused, c_REVLib_kUnused, 0, "Reserved 147"},
        {c_Spark_kUnused, c_REVLib_kUnused, 0, "Reserved 148"},
        {c_Spark_kPositionPIDWrapEnable, c_REVLib_kBool, false, "Position PID Wrap Enable: Sets whether the position value used in PID wraps between the ranges of 'Position PID Min Input' and 'Position PID Max Input'."},
        {c_Spark_kPositionPIDMinInput, c_REVLib_kFloat32, 0x00000000, "Position PID Min Input: Lower bound for PID position wrapping."},
        {c_Spark_kPositionPIDMaxInput, c_REVLib_kFloat32, 0x00000000, "Position PID Max Input: Upper bound value for PID position wrapping."},
        {c_Spark_kUnused, c_REVLib_kUnused, 0, "Reserved 152"},
        {c_Spark_kDutyCycleSensorPrescaler, c_REVLib_kUint32, 17, "Duty Cycle Sensor Prescaler: Used for adjusting the maximum duty cycle period supported. Contact REV Robotics for more help."},
        {c_Spark_kDutyCycleOffset, c_REVLib_kFloat32, 0x00000000, "Duty Cycle Offset: Sets the Duty Cycle sensor position offset. Must be from 0 to 1."},
        {c_Spark_kProductId, c_REVLib_kUint32, 0, "Product Id: Product ID of the Spark device."},
        {c_Spark_kDeviceMajorVersion, c_REVLib_kUint32, 0, "Device Major Version: Major hardware version revision of the Spark device."},
        {c_Spark_kDeviceMinorVersion, c_REVLib_kUint32, 4294967295, "Device Minor Version: Minor hardware version revision of the Spark device."},
        {c_Spark_kStatus0Period, c_REVLib_kUint32, 10, "Status 0 Period: Status Frame period in ms for Status 0"},
        {c_Spark_kStatus1Period, c_REVLib_kUint32, 250, "Status 1 Period: Status Frame period in ms for Status 1"},
        {c_Spark_kStatus2Period, c_REVLib_kUint32, 20, "Status 2 Period: Status Frame period in ms for Status 2"},
        {c_Spark_kStatus3Period, c_REVLib_kUint32, 20, "Status 3 Period: Status Frame period in ms for Status 3"},
        {c_Spark_kStatus4Period, c_REVLib_kUint32, 20, "Status 4 Period: Status Frame period in ms for Status 4"},
        {c_Spark_kStatus5Period, c_REVLib_kUint32, 20, "Status 5 Period: Status Frame period in ms for Status 5"},
        {c_Spark_kStatus6Period, c_REVLib_kUint32, 20, "Status 6 Period: Status Frame period in ms for Status 6"},
        {c_Spark_kStatus7Period, c_REVLib_kUint32, 20, "Status 7 Period: Status Frame period in ms for Status 7"},
        {c_Spark_kMAXMotionCruiseVelocity_0, c_REVLib_kFloat32, 0x00000000, "MAXMotion Cruise Velocity 0: MAXMotion slot 0: Sets the maximum velocity reached when accelerating towards the setpoint. "},
        {c_Spark_kMAXMotionMaxAccel_0, c_REVLib_kFloat32, 0x00000000, "MAXMotion Max Accel 0: MAXMotion slot 0: Sets the maximum acceleration MAXMotion will reach to reach the setpoint set."},
        {c_Spark_kMAXMotionMaxJerk_0, c_REVLib_kFloat32, 0x00000000, "MAXMotion Max Jerk 0: [Not implemented yet] MAXMotion slot 0: Sets the maximum jerk level."},
        {c_Spark_kMAXMotionAllowedProfileError_0, c_REVLib_kFloat32, 0x00000000, "MAXMotion Allowed Profile Error 0: MAXMotion slot 0: Sets the maximum allowed error from the setpoint for determining whether the setpoint has been reached."},
        {c_Spark_kMAXMotionPositionMode_0, c_REVLib_kUint32, c_Spark_kMAXMotionPositionMode_TRAPEZOIDAL, "MAXMotion Position Mode 0: MAXMotion slot 0: Sets the motion profile type for position control. Currently only supports trapezoidal."},
        {c_Spark_kMAXMotionCruiseVelocity_1, c_REVLib_kFloat32, 0x00000000, "MAXMotion Cruise Velocity 1: MAXMotion slot 1: Sets the maximum velocity reached when accelerating towards the setpoint."},
        {c_Spark_kMAXMotionMaxAccel_1, c_REVLib_kFloat32, 0x00000000, "MAXMotion Max Accel 1: MAXMotion slot 1: Sets the maximum acceleration MAXMotion will reach to reach the setpoint set."},
        {c_Spark_kMAXMotionMaxJerk_1, c_REVLib_kFloat32, 0x00000000, "MAXMotion Max Jerk 1: [Not implemented yet] MAXMotion slot 0: Sets the maximum jerk level."},
        {c_Spark_kMAXMotionAllowedProfileError_1, c_REVLib_kFloat32, 0x00000000, "MAXMotion Allowed Profile Error 1: MAXMotion slot 1: Sets the maximum allowed error from the setpoint for determining whether the setpoint has been reached."},
        {c_Spark_kMAXMotionPositionMode_1, c_REVLib_kUint32, c_Spark_kMAXMotionPositionMode_TRAPEZOIDAL, "MAXMotion Position Mode 1: MAXMotion slot 1: Sets the motion profile type for position control. Currently only supports trapezoidal."},
        {c_Spark_kMAXMotionCruiseVelocity_2, c_REVLib_kFloat32, 0x00000000, "MAXMotion Cruise Velocity 2: MAXMotion slot 2: Sets the maximum velocity reached when accelerating towards the setpoint."},
        {c_Spark_kMAXMotionMaxAccel_2, c_REVLib_kFloat32, 0x00000000, "MAXMotion Max Accel 2: MAXMotion slot 2: Sets the maximum acceleration MAXMotion will reach to reach the setpoint set."},
        {c_Spark_kMAXMotionMaxJerk_2, c_REVLib_kFloat32, 0x00000000, "MAXMotion Max Jerk 2: [Not implemented yet] MAXMotion slot 0: Sets the maximum jerk level."},
        {c_Spark_kMAXMotionAllowedProfileError_2, c_REVLib_kFloat32, 0x00000000, "MAXMotion Allowed Profile Error 2: MAXMotion slot 2: Sets the maximum allowed error from the setpoint for determining whether the setpoint has been reached."},
        {c_Spark_kMAXMotionPositionMode_2, c_REVLib_kUint32, c_Spark_kMAXMotionPositionMode_TRAPEZOIDAL, "MAXMotion Position Mode 2: MAXMotion slot 2: Sets the motion profile type for position control. Currently only supports trapezoidal."},
        {c_Spark_kMAXMotionCruiseVelocity_3, c_REVLib_kFloat32, 0x00000000, "MAXMotion Cruise Velocity 3: MAXMotion slot 3: Sets the maximum velocity reached when accelerating towards the setpoint."},
        {c_Spark_kMAXMotionMaxAccel_3, c_REVLib_kFloat32, 0x00000000, "MAXMotion Max Accel 3: MAXMotion slot 3: Sets the maximum acceleration MAXMotion will reach to reach the setpoint set."},
        {c_Spark_kMAXMotionMaxJerk_3, c_REVLib_kFloat32, 0x00000000, "MAXMotion Max Jerk 3: [Not implemented yet] MAXMotion slot 0: Sets the maximum jerk level."},
        {c_Spark_kMAXMotionAllowedProfileError_3, c_REVLib_kFloat32, 0x00000000, "MAXMotion Allowed Profile Error 3: MAXMotion slot 3: Sets the maximum allowed error from the setpoint for determining whether the setpoint has been reached."},
        {c_Spark_kMAXMotionPositionMode_3, c_REVLib_kUint32, c_Spark_kMAXMotionPositionMode_TRAPEZOIDAL, "MAXMotion Position Mode 3: MAXMotion slot 3: Sets the motion profile type for position control. Currently only supports trapezoidal."},
        {c_Spark_kForceEnableStatus_0, c_REVLib_kBool, false, "Force Enable Status 0: Sets whether the status frame 0 is forced to be enabled. Note: Status 0 is always forced enabled because it acts as the identifier frame for Sparks."},
        {c_Spark_kForceEnableStatus_1, c_REVLib_kBool, false, "Force Enable Status 1: Sets whether the status frame 1 is forced to be enabled. Note: disabling this setting does not turn off the frame. A separate 'Set Status Frame Enabled' command needs to be sent."},
        {c_Spark_kForceEnableStatus_2, c_REVLib_kBool, false, "Force Enable Status 2: Sets whether the status frame 2 is forced to be enabled. Note: disabling this setting does not turn off the frame. A separate 'Set Status Frame Enabled' command needs to be sent."},
        {c_Spark_kForceEnableStatus_3, c_REVLib_kBool, false, "Force Enable Status 3: Sets whether the status frame 3 is forced to be enabled. Note: disabling this setting does not turn off the frame. A separate 'Set Status Frame Enabled' command needs to be sent."},
        {c_Spark_kForceEnableStatus_4, c_REVLib_kBool, false, "Force Enable Status 4: Sets whether the status frame 4 is forced to be enabled. Note: disabling this setting does not turn off the frame. A separate 'Set Status Frame Enabled' command needs to be sent."},
        {c_Spark_kForceEnableStatus_5, c_REVLib_kBool, false, "Force Enable Status 5: Sets whether the status frame 5 is forced to be enabled. Note: disabling this setting does not turn off the frame. A separate 'Set Status Frame Enabled' command needs to be sent."},
        {c_Spark_kForceEnableStatus_6, c_REVLib_kBool, false, "Force Enable Status 6: Sets whether the status frame 6 is forced to be enabled. Note: disabling this setting does not turn off the frame. A separate 'Set Status Frame Enabled' command needs to be sent."},
        {c_Spark_kForceEnableStatus_7, c_REVLib_kBool, false, "Force Enable Status 7: Sets whether the status frame 7 is forced to be enabled. Note: disabling this setting does not turn off the frame. A separate 'Set Status Frame Enabled' command needs to be sent."},
        {c_Spark_kFollowerModeLeaderId, c_REVLib_kUint32, 0, "Follower Mode Leader Id: Sets the CAN ID of the leader Spark for follower mode."},
        {c_Spark_kFollowerModeIsInverted, c_REVLib_kBool, false, "Follower Mode Is Inverted: Sets whether the duty cycle value received from the leader Spark should be inverted."},
        {c_Spark_kDutyCycleEncoderStartPulseUs, c_REVLib_kFloat32, 0x00000000, "Duty Cycle Encoder Start Pulse Us: Sets the expected start pulse of the input duty cycle signal in us."},
        {c_Spark_kDutyCycleEncoderEndPulseUs, c_REVLib_kFloat32, 0x00000000, "Duty Cycle Encoder End Pulse Us: Sets the expected end pulse of the input duty cycle signal in us."},
        {c_Spark_kParamTableVersion, c_REVLib_kUint32, 0, "Param Table Version: Contains the versioning for the parameter table stored in EEPROM."},
        {c_Spark_kStatus8Period, c_REVLib_kUint32, 10, "Status 8 Period: Status Frame period in ms for Status 8"},
        {c_Spark_kForceEnableStatus_8, c_REVLib_kBool, false, "Force Enable Status 8: Sets whether the status frame 8 is forced to be enabled. Note: disabling this setting does not turn off the frame. A separate 'Set Status Frame Enabled' command needs to be sent."},
        {c_Spark_kLimitSwitchPositionSensor, c_REVLib_kUint32, c_Spark_kSensor_NONE, "Limit Switch Position Sensor: Sets the hardware limit switch position sensor. This is used when the limit switch mode is set to set a position value on a limit switch trigger."},
        {c_Spark_kLimitSwitchFwdPosition, c_REVLib_kFloat32, 0x00000000, "Limit Switch Fwd Position: Sets the position value that the limit switch sensor will be set to when the forward limit switch is triggered."},
        {c_Spark_kLimitSwitchRevPosition, c_REVLib_kFloat32, 0x00000000, "Limit Switch Rev Position: Sets the position value that the limit switch sensor will be set to when the reverse limit switch is triggered."},
        {c_Spark_kS_0, c_REVLib_kFloat32, 0x00000000, "S 0: PID Slot 0: Feed forward static gain. Used for adjusting the PID output by the factor set when in motion."},
        {c_Spark_kA_0, c_REVLib_kFloat32, 0x00000000, "A 0: PID Slot 0: Feed forward acceleration gain. Used for adjusting the PID output by the factor set when accelerating."},
        {c_Spark_kG_0, c_REVLib_kFloat32, 0x00000000, "G 0: PID Slot 0: Feed forward linear gravity gain factor. Used for adjusting the PID output to account for a constant gravity forces."},
        {c_Spark_kCos_0, c_REVLib_kFloat32, 0x00000000, "Cos 0: PID Slot 0: Feed forward Cosine gravity gain. Used for adjusting the PID output to account for a non-linear gravity pull such as when an arm is extended sideways vs. when the arm is completely vertical."},
        {c_Spark_kCosRatio_0, c_REVLib_kFloat32, 0x00000000, "CosRatio 0: PID Slot 0: Feed forward cosine gravity ratio. Note: This is mainly used when the position sensor is converted to a different unit of measurement with its respective conversion factor, this setting's value is used to convert the position value back to absolute rotation positions to correctly apply the cosine gravity gain factor."},
        {c_Spark_kS_1, c_REVLib_kFloat32, 0x00000000, "S 1: PID Slot 1: Feed forward static gain. Used for adjusting the PID output by the factor set when in motion."},
        {c_Spark_kA_1, c_REVLib_kFloat32, 0x00000000, "A 1: PID Slot 1: Feed forward acceleration gain. Used for adjusting the PID output by the factor set when accelerating."},
        {c_Spark_kG_1, c_REVLib_kFloat32, 0x00000000, "G 1: PID Slot 1: Feed forward linear gravity gain factor. Used for adjusting the PID output to account for a constant gravity forces."},
        {c_Spark_kCos_1, c_REVLib_kFloat32, 0x00000000, "Cos 1: PID Slot 1: Feed forward Cosine gravity gain. Used for adjusting the PID output to account for a non-linear gravity pull such as when an arm is extended sideways vs. when the arm is completely vertical."},
        {c_Spark_kCosRatio_1, c_REVLib_kFloat32, 0x00000000, "CosRatio 1: PID Slot 1: Feed forward cosine gravity ratio. Note: This is mainly used when the position sensor is converted to a different unit of measurement with its respective conversion factor, this setting's value is used to convert the position value back to absolute rotation positions to correctly apply the cosine gravity gain factor."},
        {c_Spark_kS_2, c_REVLib_kFloat32, 0x00000000, "S 2: PID Slot 2: Feed forward static gain. Used for adjusting the PID output by the factor set when in motion."},
        {c_Spark_kA_2, c_REVLib_kFloat32, 0x00000000, "A 2: PID Slot 2: Feed forward acceleration gain. Used for adjusting the PID output by the factor set when accelerating."},
        {c_Spark_kG_2, c_REVLib_kFloat32, 0x00000000, "G 2: PID slot 2: Feed forward linear gravity gain factor. Used for adjusting the PID output to account for a constant gravity forces."},
        {c_Spark_kCos_2, c_REVLib_kFloat32, 0x00000000, "Cos 2: PID slot 2: Feed forward Cosine gravity gain. Used for adjusting the PID output to account for a non-linear gravity pull such as when an arm is extended sideways vs. when the arm is completely vertical."},
        {c_Spark_kCosRatio_2, c_REVLib_kFloat32, 0x00000000, "CosRatio 2: PID slot 2: Feed forward cosine gravity ratio. Note: This is mainly used when the position sensor is converted to a different unit of measurement with its respective conversion factor, this setting's value is used to convert the position value back to absolute rotation positions to correctly apply the cosine gravity gain factor."},
        {c_Spark_kS_3, c_REVLib_kFloat32, 0x00000000, "S 3: PID Slot 3: Feed forward static gain. Used for adjusting the PID output by the factor set when in motion."},
        {c_Spark_kA_3, c_REVLib_kFloat32, 0x00000000, "A 3: PID Slot 3: Feed forward acceleration gain. Used for adjusting the PID output by the factor set when accelerating."},
        {c_Spark_kG_3, c_REVLib_kFloat32, 0x00000000, "G 3: PID Slot 3: Feed forward linear gravity gain factor. Used for adjusting the PID output to account for a constant gravity forces."},
        {c_Spark_kCos_3, c_REVLib_kFloat32, 0x00000000, "Cos 3: PID Slot 3: Feed forward Cosine gravity gain. Used for adjusting the PID output to account for a non-linear gravity pull such as when an arm is extended sideways vs. when the arm is completely vertical."},
        {c_Spark_kCosRatio_3, c_REVLib_kFloat32, 0x00000000, "CosRatio 3: PID Slot 3: Feed forward cosine gravity ratio. Note: This is mainly used when the position sensor is converted to a different unit of measurement with its respective conversion factor, this setting's value is used to convert the position value back to absolute rotation positions to correctly apply the cosine gravity gain factor."},
        {c_Spark_kStatus9Period, c_REVLib_kUint32, 10, "Status 9 Period: Status Frame period in ms for Status 9"},
        {c_Spark_kForceEnableStatus_9, c_REVLib_kBool, false, "Force Enable Status 9: Sets whether the status frame 9 is forced to be enabled. Note: disabling this setting does not turn off the frame. A separate 'Set Status Frame Enabled' command needs to be sent."},
        {c_Spark_kDetachedEncoderDeviceID, c_REVLib_kUint32, 0, "Detached Encoder Device ID: Sets the Detached Encoder CAN ID for Closed-Loop PID control."},
    }};

}  // namespace

c_REVLib_ParameterType c_Spark_GetParameterType(
    c_Spark_ConfigParameter parameterId) {
    return s_Spark_ParameterTable[parameterId].type;
}

uint32_t c_Spark_GetParameterDefaultValue(c_Spark_ConfigParameter parameterId) {
    return s_Spark_ParameterTable[parameterId].defaultValue;
}

const char* c_Spark_GetParameterName(c_Spark_ConfigParameter parameterId) {
    return s_Spark_ParameterTable[parameterId].name.c_str();
}

c_Spark_ConfigParameter c_Spark_GetConfigParameter(
    c_Spark_ConfigParameter parameterId) {
    return s_Spark_ParameterTable[parameterId].id;
}
