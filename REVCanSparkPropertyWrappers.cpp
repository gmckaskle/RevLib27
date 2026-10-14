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

we currently have softLimit value, but not "wasReached" I think that is what we meant to publish


library: niREVSpark.dll

#include "rev/CANCommonParameters.h"
#include "rev/CANSparkParameters.h"
#include "rev/REVCommon.h"
#include "rev/REVLibErrors.h"
#include "rev/REVLibVersion.h"
#include "rev/REVUtils.h"
struct private_parameter_table_entry_t {
    c_Spark_ConfigParameter id;
    c_REVLib_ParameterType type;
    uint32_t defaultValue;
    std::string name;
};

// COPIED FROM CANSpartParameters.cpp because the removed the export of the info.
private_parameter_table_entry_t paramInfo[] = {
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
    };






LV_REVSparkSetAttribute(refPtr, id, ptr) {
	c_REVLib_ErrorCode revErr;
	int32_t i32Val;
	uint32_t u32Val;
	float f32Val;
	uint8_t boolVal;
	// get hdl from refPtr return if doesn't work

	// this property directly maps to a REV property, so use the above table
	if(id <= c_Spark_kDetachedEncoderDeviceID ) { // better with a sentinel
		switch(paramInfo[id].type)
		case c_REVLib_kUnused:
		// debug printing, return error
		break;

		case c_REVLib_kInt32:
		i32Val = *(int32*)ptr;
		revErr = c_Spark_SetParameterInt32(hdl,id, i32Val);
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
	else { // property wraps other function calls or aggregates multiple properties into a cluster
		switch(id) {
		}
	}

}





LV_REVSparkGetAttribute(refPtr, id, ptr) {
	c_REVLib_ErrorCode revErr;
	int32_t i32Val;
	uint32_t u32Val;
	float f32Val;
	uint8_t boolVal;

		// get hdl from refPtr return if doesn't work

	// this property directly maps to a REV property, so use the above table
	if(id <= c_Spark_kDetachedEncoderDeviceID ) { // better with a sentinel
		switch(paramInfo[id].type)
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
		revErr = GetParameterFloat32(hdl, id, &f32Val);
		*(float*)ptr = f32Val;
		break;

		case c_REVLib_kBool:
		revErr = c_Spark_GetParameterBool(hdl, id, &boolVal);
		*(int8*)ptr = boolVal;
		break;


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
		case 0x200: // VoltageCompensation -- Spark Voltage Compensation.ctl
			LV_REVSparkGetAttribute(refPtr, c_Spark_kVoltageCompensationMode, ptr);
			ptr += 4;
			LV_REVSparkGetAttribute(refPtr, c_Spark_kCompensatedNominalVoltage, ptr);
		break;

		case 0x201: // MotorOutputCurrent
			revErr =  c_Spark_GetPeriodicStatus0(hdl, &status0);
			*(float64*)ptr = status0->current;
		break;

		case 0x202: // MotorTemperature
			revErr =  c_Spark_GetPeriodicStatus0(hdl, &status0);
			*(float64*)ptr = status0->motorTemperature;
			break;

		case 0x203: // MotorBusVoltage	
			revErr =  c_Spark_GetPeriodicStatus0(hdl, &status0);
			*(float64*)ptr = status0->voltage;
			break;

		case 0x204: // MotorAppliedOutput
			revErr =  c_Spark_GetPeriodicStatus0(hdl, &status0);
			*(float64*)ptr = status0->appliedOutput;
		break;

		case 0x205: // FollowerModeInfo	-- cluster {2, "Leader CAN ID"=uL, "Inverted"=bool}
			LV_REVSparkGetAttribute(refPtr, c_Spark_kFollowerModeLeaderId, ptr);
			ptr = GetBoolTD().AlignDatum(ptr+sizeof(int32));
			LV_REVSparkGetAttribute(refPtr, c_Spark_kFollowerModeIsInverted, ptr);
		break;

		case 0x206: // PositionPIDWrap	-- cluster { 3, "Enable" = bool, "Min" = fD, "Max"=fD }
			LV_REVSparkGetAttribute(refPtr, c_Spark_kPositionPIDWrapEnable, ptr);
			ptr = GetfD().AlignDatum(ptr + sizeof(LVBoolean));
			LV_REVSparkGetAttribute(refPtr, c_Spark_kPositionPIDMinInput, ptr);
			ptr = GetfD().AlignDatum(ptr+sizeof(float64));
			LV_REVSparkGetAttribute(refPtr, c_Spark_kPositionPIDMaxInput, ptr);
		break;


		// REV pidf slots are 8 propIds apart, different from LV attrIDs
		case 0x213: // PIDF_3	
			slotOfst ++;
		case 0x20f: // PIDF_2	
			slotOfst ++;
		case 0x20b: // PIDF_1	
			slotOfst ++;
		case 0x207: // PIDF_0	-- cluster { 4, "kP" = fD, "kI" = fD, "kD"=fD, "kF"=fD }
			LV_REVSparkGetAttribute(refPtr, c_Spark_kP_0 + slotOfst*8, ptr);
			ptr = GetfD().AlignDatum(ptr + sizeof(float64));
			LV_REVSparkGetAttribute(refPtr, c_Spark_kI_0 + slotOfst*8, ptr);
			ptr = GetfD().AlignDatum(ptr + sizeof(float64));
			LV_REVSparkGetAttribute(refPtr, c_Spark_kD_0 + slotOfst*8, ptr);
			ptr = GetfD().AlignDatum(ptr + sizeof(float64));
			LV_REVSparkGetAttribute(refPtr, c_Spark_kV_0 + slotOfst*8, ptr);
		break;

		case 0x214: // PIDOutRng_3
			slotOfst ++;
		case 0x210: // PIDOutRng_2
			slotOfst ++;
		case 0x20C: // PIDOutRng_1
			slotOfst ++;
		case 0x208: // PIDOutRng_0	-- cluster { 2, "min" = fD, "max" = fD }
			LV_REVSparkGetAttribute(refPtr, c_Spark_kOutputMin_0 + slotOfst*8, ptr);
			ptr = GetfD().AlignDatum(ptr + sizeof(float64));
			LV_REVSparkGetAttribute(refPtr, c_Spark_kOutputMax_0 + slotOfst*8, ptr);
		break;

		case 0x215: // PIDAdv_3
			slotOfst ++;
		case 0x211: // PIDAdv_2
			slotOfst ++;
		case 0x20D: // PIDAdv_1
			slotOfst ++;
		case 0x209: // PIDAdv_0	-- { 4, "i Zone" = fD, "max I Accum" = fD, "d Filter"=fD, "error Limit"=fD }
			"PID:Slot 0:Advanced"
			cluster { 4, "i Zone" = fD, "max I Accum" = fD, "d Filter"=fD, "error Limit"=fD } readwrite	29	  209
			LV_REVSparkGetAttribute(refPtr, c_Spark_kIZone_0 + slotOfst*8, ptr);
			ptr = GetfD().AlignDatum(ptr + sizeof(float64));
			LV_REVSparkGetAttribute(refPtr, c_Spark_kIMaxAccum_0 + slotOfst*2, ptr);
			ptr = GetfD().AlignDatum(ptr + sizeof(float64));
			LV_REVSparkGetAttribute(refPtr, c_Spark_kDFilter_0 + slotOfst*8, ptr);
			ptr = GetfD().AlignDatum(ptr + sizeof(float64));
			LV_REVSparkGetAttribute(refPtr, c_Spark_kAllowedClosedLoopError_0 + slotOfst*2, ptr);
		break;

		case 0x216: // PIDModel_3
			slotOfst ++;
		case 0x212: // PIDModel_2
			slotOfst ++;
		case 0x20E: // PIDModel_1
			slotOfst ++;
		case 0x20a: // PIDModel_0	-- cluster { 5, "kS" = fD, "kA" = fD, "kG"=fD, "kCos"=fD, "Cos Ratio"=fD }
			LV_REVSparkGetAttribute(refPtr, c_Spark_kS_0 + slotOfst*5, ptr);
			ptr = GetfD().AlignDatum(ptr + sizeof(float64));
			LV_REVSparkGetAttribute(refPtr, c_Spark_kA_0 + slotOfst*5, ptr);
			ptr = GetfD().AlignDatum(ptr + sizeof(float64));
			LV_REVSparkGetAttribute(refPtr, c_Spark_kG_0 + slotOfst*5, ptr);
			ptr = GetfD().AlignDatum(ptr + sizeof(float64));
			LV_REVSparkGetAttribute(refPtr, c_Spark_kCos_0 + slotOfst*5, ptr);
			ptr = GetfD().AlignDatum(ptr + sizeof(float64));
			LV_REVSparkGetAttribute(refPtr, c_Spark_kCosRatio_0 + slotOfst*5, ptr);
		break;

		case 0x217: // EncoderPositionValue	
			revErr =  c_Spark_GetPeriodicStatus2(hdl, &status2);
			*(float64*)ptr = status2->primaryEncoderPosition;
		break;

		case 0x218: // EncoderVelocityValue	
			revErr =  c_Spark_GetPeriodicStatus2(hdl, &status2);
			*(float64*)ptr = status2->primaryEncoderVelocity;
		break;

		case 0x219: // LimitSwitchFwdValue	
			revErr =  c_Spark_GetPeriodicStatus0(hdl, &status0);
			*(uInt8*)ptr = status0->hardForwardLimitReached;
		break;

		case 0x21a: // LimitSwitchRevValue	
			revErr =  c_Spark_GetPeriodicStatus0(hdl, &status0);
			*(uInt8*)ptr = status0->hardReverseLimitReached;
		break;

		case 0x21b: // SoftLimitForwardValue	
			revErr =  c_Spark_GetPeriodicStatus0(hdl, &status0);
			*(uInt8*)ptr = status0->softForwardLimitReached;

		break;

		case 0x21c: // SoftLimitReverseValue	
			revErr =  c_Spark_GetPeriodicStatus0(hdl, &status0);
			*(uInt8*)ptr = status0->softReverseLimitReached;
		break;

		case 0x21d: // AnalogPositionValue	
			revErr =  c_Spark_GetPeriodicStatus3(hdl, &status3);
			*(float64*)ptr = status3->analogPosition;
		break;

		case 0x21e: // AnalogVelocityValue	
			revErr =  c_Spark_GetPeriodicStatus3(hdl, &status3);
			*(float64*)ptr = status3->analogVelocity;
		break;

		case 0x21f: // AnalogVoltageValue	
			revErr =  c_Spark_GetPeriodicStatus3(hdl, &status3);
			*(float64*)ptr = status3->analogVoltage;
		break;

		case 0x220: // AltEncoderPositionValue	
			revErr =  c_Spark_GetPeriodicStatus4(hdl, &status4);
			*(float64*)ptr = status4->externalOrAltEncoderPosition;
		break;

		case 0x221: // AltEncoderVelocityValue	
			revErr =  c_Spark_GetPeriodicStatus4(hdl, &status4);
			*(float64*)ptr = status4->externalOrAltEncoderVelocity;
		break;

		case 0x225: // MAXMotionConfig_3	
			slotOfst ++;
		case 0x224: // MAXMotionConfig_2	
			slotOfst ++;
		case 0x223: // MAXMotionConfig_1	
			slotOfst ++;
		case 0x222: // MAXMotionConfig_0	-- cluster {5, "Cruise Velocity"=fD, "Accel Limit"=fD, "Jerk Limit"=fD, "Goal Error"=fD, "Mode"=uL}
			LV_REVSparkGetAttribute(refPtr, kMAXMotionCruiseVelocity_0 + slotOfst*5, ptr);
			ptr = GetfD().AlignDatum(ptr + sizeof(float64));
			LV_REVSparkGetAttribute(refPtr, c_Spark_kMAXMotionMaxAccel_0 + slotOfst*5, ptr);
			ptr = GetfD().AlignDatum(ptr + sizeof(float64));
			LV_REVSparkGetAttribute(refPtr, c_Spark_kMAXMotionMaxJerk_0 + slotOfst*5, ptr);
			ptr = GetfD().AlignDatum(ptr + sizeof(float64));
			LV_REVSparkGetAttribute(refPtr, c_Spark_kMAXMotionAllowedProfileError_0 + slotOfst*5, ptr);
			ptr = GetuL().AlignDatum(ptr + sizeof(float64));
			LV_REVSparkGetAttribute(refPtr, c_Spark_kMAXMotionPositionMode_0 + slotOfst*5, ptr);
		break;

		case 0x226: // MotorFaults	-- other, Motor Type, Sensor, CAN, Temp, GateDrv, Esc EEProm, Firmware
			revErr =  c_Spark_GetPeriodicStatus1(hdl, &status1);
			*(LVBoolean*)ptr = status1->otherFault;
			ptr++;
			*(LVBoolean*)ptr = status1->motorTypeFault;
			ptr++;
			*(LVBoolean*)ptr = status1->sensorFault;
			ptr++;
			*(LVBoolean*)ptr = status1->canFault;
			ptr++;
			*(LVBoolean*)ptr = status1->temperatureFault;
			ptr++;
			*(LVBoolean*)ptr = status1->drvFault;
			ptr++;
			*(LVBoolean*)ptr = status1->escEepromFault;
			ptr++;
			*(LVBoolean*)ptr = status1->firmwareFault;
		break;
		case 0x227: // MotorStickyFaults -- "<vilib>:REV Custom Controls\Spark Faults2.ctl"
			revErr =  c_Spark_GetPeriodicStatus1(hdl, &status1);
			*(LVBoolean*)ptr = status1->otherStickyFault;
			ptr++;
			*(LVBoolean*)ptr = status1->motorTypeStickyFault;
			ptr++;
			*(LVBoolean*)ptr = status1->sensorStickyFault;
			ptr++;
			*(LVBoolean*)ptr = status1->canStickyFault;
			ptr++;
			*(LVBoolean*)ptr = status1->temperatureStickyFault;
			ptr++;
			*(LVBoolean*)ptr = status1->drvStickyFault;
			ptr++;
			*(LVBoolean*)ptr = status1->escEepromStickyFault;
			ptr++;
			*(LVBoolean*)ptr = status1->firmwareStickyFault;
		break;

		case 0x228: // MotorWarnings -- Brownout, overCurrent, EscEEProm, Ext EEProm, Sensor, Stall, Has Reset, Other
			revErr =  c_Spark_GetPeriodicStatus1(hdl, &status1);
			*(LVBoolean*)ptr = status1->brownoutWarning;
			ptr++;
			*(LVBoolean*)ptr = status1->overcurrentWarning;
			ptr++;
			*(LVBoolean*)ptr = status1->escEepromWarning;
			ptr++;
			*(LVBoolean*)ptr = status1->extEepromWarning;
			ptr++;
			*(LVBoolean*)ptr = status1->sensorWarning;
			ptr++;
			*(LVBoolean*)ptr = status1->stallWarning;
			ptr++;
			*(LVBoolean*)ptr = status1->hasResetWarning;
			ptr++;
			*(LVBoolean*)ptr = status1->otherWarning;
		break;

		case 0x229: // MotorStickyWarnings -- Brownout, overCurrent, EscEEProm, Ext EEProm, Sensor, Stall, Has Reset, Other
			revErr =  c_Spark_GetPeriodicStatus1(hdl, &status1);
			*(LVBoolean*)ptr = status1->brownoutStickyWarning;
			ptr++;
			*(LVBoolean*)ptr = status1->overcurrentStickyWarning;
			ptr++;
			*(LVBoolean*)ptr = status1->escEepromStickyWarning;
			ptr++;
			*(LVBoolean*)ptr = status1->extEepromStickyWarning;
			ptr++;
			*(LVBoolean*)ptr = status1->sensorStickyWarning;
			ptr++;
			*(LVBoolean*)ptr = status1->stallStickyWarning;
			ptr++;
			*(LVBoolean*)ptr = status1->hasResetStickyWarning;
			ptr++;
			*(LVBoolean*)ptr = status1->otherStickyWarning;
		break;

		case 0x22A: // MotorDetails	-- CANID, MajorVers, MinorVers, Type, InputMode, ProdID, kV, PolePairs, CommutationAdv
			LV_REVSparkGetAttribute(refPtr, c_Spark_kCANID, ptr);
			ptr = GetuW().AlignDatum(ptr + sizeof(int32));
			LV_REVSparkGetAttribute(refPtr, c_Spark_kDeviceMajorVersion, ptr);
			ptr = GetuW().AlignDatum(ptr + sizeof(int16));
			LV_REVSparkGetAttribute(refPtr, c_Spark_kDeviceMinorVersion, ptr);
			ptr = GetuL().AlignDatum(ptr + sizeof(int16));
			LV_REVSparkGetAttribute(refPtr, c_Spark_kInputMode, ptr);
			ptr = GetuL().AlignDatum(ptr + sizeof(int32));
			LV_REVSparkGetAttribute(refPtr, c_Spark_kProductId, ptr);
			ptr = GetuL().AlignDatum(ptr + sizeof(int32));
			LV_REVSparkGetAttribute(refPtr, c_Spark_kMotorKv, ptr);
			ptr = GetuL().AlignDatum(ptr + sizeof(int32));
			LV_REVSparkGetAttribute(refPtr, c_Spark_kPolePairs, ptr);
			ptr = GetuL().AlignDatum(ptr + sizeof(int32));
			LV_REVSparkGetAttribute(refPtr, c_Spark_kCommutationAdvance, ptr);
		break;
		default:
		// debugprintf or other, and return error ???
		break;
		}
#endif
	}
}





















#if 0
// here for documentation 
RCversion: 3
include: "RevSpark.rch"

 ///// REVSpark Class Definition /////
 class: RevSpark
genericType: RevSpark
passAttributeError: none
className: RevSpark
flags: 0




attribute:  Inverted	
    "Power:Inverted"
	bool	readwrite	29	  2D
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  IdleMode	
    "Power:Idle Mode"
	uL	enum	{"Coast"=0, "Brake"=1}    readwrite	29	  6
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  InputDeadband	
    "Power:Input Deadband"
	fD	readwrite	29	  7
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  OpenLoopRampRate	
    "Power:Open Loop Ramp Rate"
	fD	readwrite	29	  38
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  SmartCurrentStallLimit	
    "Power:Smart Current Stall Limit"
	uL	readwrite	29	  3B
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  SmartCurrentFreeLimit	
    "Power:Smart Current Free Limit"
	uL	readwrite	29	  3C
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  SmartCurrentConfig	
    "Power:Smart Current Config"
	uL	readwrite	29	  3D
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  VoltageCompensation	
    "Power:Voltage Compensation"
	typedef "<vilib>:REV Custom Controls\Spark Voltage Compensation.ctl"    readwrite	29	  24A
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  CurrentChop	
    "Power:Current Chop"
	fD	readwrite	29	  B
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  CurrentChopCycles	
    "Power:Current Chop Cycles"
	uL	readwrite	29	  C
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  MotorOutputCurrent	
    "Power:Output Current"
	typedef "<vilib>:REV Custom Controls\Spark Warnings2.ctl"  readwrite	29	  306
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  MotorTemperature
    "Power:Motor Temperature"
	fD  readwrite	29	  307
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"
	
attribute:  MotorBusVoltage	
    "Power:Bus Voltage"
	fD  readwrite	29	  308
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"
	
attribute:  MotorAppliedOutput
    "Power:Applied Output"
	fD  readwrite	29	  309
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"


attribute:  FollowerModeInfo	
    "Follow"
	cluster {2, "Leader CAN ID"=uL, "Inverted"=bool}	readwrite	29	  1C2
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"



attribute:  ControlType	
    "PID:Control Type"
	uL	enum	{"Duty Cycle"=0, "Velocity"=1, "Voltage"=2, "Position"=3, "Current"=4, "MaxMotion Position"=5, "MaxMotion Velocity"=6}    readwrite	29	  5
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  ClosedLoopControlSensor	
    "PID:Closed Loop Control Sensor"
	uL	enum	{"None"=0, "Main Encoder"=1, "Analog"=2, "Alt Encoder"=3, "Duty Cycle"=4, "Detached Absolute"=5, "Detached Relative"=6}    readwrite	29	  9
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"
"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  PositionPIDWrap	
    "PID:Position PID Wrap"
	cluster { 3, "Enable" = bool, "Min" = fD, "Max"=fD } readwrite	29	  4E4
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  DetachedEncoderDeviceID	
    "PID:Detached Encoder Device ID"
	uL	readwrite	29	  E2
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  PIDF_0	
    "PID:Slot 0:PIDF Gains"
	cluster { 4, "kP" = fD, "kI" = fD, "kD"=fD, "kF"=fD } readwrite	29	  1E3
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  PIDOutRng_0	
    "PID:Slot 0:Output Range"
	cluster { 2, "min" = fD, "max" = fD } readwrite	29	  2E3
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  PIDAdv_0	
    "PID:Slot 0:Advanced"
	cluster { 4, "i Zone" = fD, "max I Accum" = fD, "d Filter"=fD, "error Limit"=fD } readwrite	29	  3E3
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  PIDModel_0	
    "PID:Slot 0:Model Gains"
	cluster { 5, "kS" = fD, "kA" = fD, "kG"=fD, "kCos"=fD, "Cos Ratio"=fD } readwrite	29	  4E3
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  PIDF_1	
    "PID:Slot 1:PIDF Gains"
	cluster { 4, "kP" = fD, "kI" = fD, "kD"=fD, "kF"=fD } readwrite	29	  1E4
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  PIDOutRng_1	
    "PID:Slot 1:Output Range"
	cluster { 2, "min" = fD, "max" = fD } readwrite	29	  2E4
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  PIDAdv_1	
    "PID:Slot 1:Advanced"
	cluster { 4, "i Zone" = fD, "max I Accum" = fD, "d Filter"=fD, "error Limit"=fD } readwrite	29	  3E4
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  PIDModel_1	
    "PID:Slot 1:Model Gains"
	cluster { 5, "kS" = fD, "kA" = fD, "kG"=fD, "kCos"=fD, "Cos Ratio"=fD } readwrite	29	  4E4
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  PIDF_2	
    "PID:Slot 2:PIDF Gains"
	cluster { 4, "kP" = fD, "kI" = fD, "kD"=fD, "kF"=fD } readwrite	29	  1E5
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  PIDOutRng_2	
    "PID:Slot 2:Output Range"
	cluster { 2, "min" = fD, "max" = fD } readwrite	29	  2E5
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  PIDAdv_2	
    "PID:Slot 2:Advanced"
	cluster { 4, "i Zone" = fD, "max I Accum" = fD, "d Filter"=fD, "error Limit"=fD } readwrite	29	  3E5
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  PIDModel_2	
    "PID:Slot 2:Model Gains"
	cluster { 5, "kS" = fD, "kA" = fD, "kG"=fD, "kCos"=fD, "Cos Ratio"=fD } readwrite	29	  4E5
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  PIDF_3	
    "PID:Slot 3:PIDF Gains"
	cluster { 4, "kP" = fD, "kI" = fD, "kD"=fD, "kF"=fD } readwrite	29	  1E6
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  PIDOutRng_3	
    "PID:Slot 3:Output Range"
	cluster { 2, "min" = fD, "max" = fD } readwrite	29	  2E6
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  PIDAdv_3	
    "PID:Slot 3:Advanced"
	cluster { 4, "i Zone" = fD, "max I Accum" = fD, "d Filter"=fD, "error Limit"=fD } readwrite	29	  3E6
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  PIDModel_3	
    "PID:Slot 3:Model Gains"
	cluster { 5, "kS" = fD, "kA" = fD, "kG"=fD, "kCos"=fD, "Cos Ratio"=fD } readwrite	29	  4E6
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"



attribute:  EncoderPositionValue	
    "Sensor:Encoder:Position"
	fD	readwrite	29	  348
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  EncoderVelocityValue	
    "Sensor:Encoder:Velocity"
	fD	readonly	29	  349
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  EncoderInverted	
    "Sensor:Encoder:Inverted"
	bool	readwrite	29	  48
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  EncoderCountsPerRev	
    "Sensor:Encoder:Counts Per Rev"
	uL	readwrite	29	  45
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  EncoderAverageDepth	
    "Sensor:Encoder:Average Depth"
	uL	readwrite	29	  46
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  EncoderSampleDelta	
    "Sensor:Encoder:Sample Delta"
	uL	readwrite	29	  47
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  DutyCycleInverted	
    "Sensor:Abs Encoder:Inverted"
	bool	readwrite	29	  8D
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  DutyCycleAverageDepth	
    "Sensor:Abs Encoder:Average Depth"
	uL	readwrite	29	  8F
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  DutyCycleRangeOffset	
    "Sensor:Abs Encoder:Range Offset"
	fD	readwrite	29	  92
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  LimitSwitchFwdValue	
    "Sensor:Limit Switch:Forward Value"
	bool	readwrite	29	  332
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  LimitSwitchFwdPolarity	
    "Sensor:Limit Switch:Forward Polarity"
	bool	readwrite	29	  32
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  LimitSwitchRevValue	
    "Sensor:Limit Switch:Reverse Value"
	bool	readwrite	29	  333
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  LimitSwitchRevPolarity	
    "Sensor:Limit Switch:Reverse Polarity"
	bool	readwrite	29	  33
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  HardLimitFwdEn	
    "Sensor:Limit Switch:Forward Enable"
	uL    enum {"Move"=0, "Stop"=1, "Set Sensor"=2, "Stop & Set Sensor"=3}    readwrite	29	  34
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  HardLimitRevEn	
    "Sensor:Limit Switch:Reverse Enable"
	uL    enum {"Move"=0, "Stop"=1, "Set Sensor"=2, "Stop & Set Sensor"=3}    readwrite	29	  35
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  LimitSwitchPositionSensor	
    "Sensor:Limit Switch:Position Sensor"
	uL	enum	{"None"=0, "Main Encoder"=1, "Analog"=2, "Alt Encoder"=3, "Duty Cycle"=4, "Detached Absolute"=5, "Detached Relative"=6}    readwrite	29	  C9
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  SoftLimitForward	
    "Sensor:Limit Switch:Forward Setpoint"
	fD	readonly	29	  73
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  SoftLimitForwardValue	
    "Sensor:Limit Switch:Forward Value"
	fD	readonly	29	  373
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  SoftLimitReverse	
    "Sensor:Limit Switch:Reverse Setpoint"
	fD	readonly	29	  74
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  SoftLimitReverseValue	
    "Sensor:Limit Switch:Reverse Value"
	fD	readonly	29	  374
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  SoftLimitFwdEn	
    "Sensor:Soft Limit:Forward Enable"
	bool	readwrite	29	  36
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  SoftLimitRevEn	
    "Sensor:Soft Limit:Reverse Enable"
	bool	readwrite	29	  37
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  LimitSwitchFwdPosition	
    "Sensor:Soft Limit:Forward Value"
	fD	readonly	29	  CA
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  LimitSwitchRevPosition	
    "Sensor:Soft Limit:Reverse Value"
	fD	readonly	29	  CB
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  AnalogAverageDepth	
    "Sensor:Analog:Average Filter Depth"
	uL	readwrite	29	  79
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  AnalogSensorMode	
    "Sensor:Analog:Sensor Mode"
	uL	enum	{"Absolute"=0, "Relative"=1}	readwrite	29	  7A
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  AnalogPositionValue	
    "Sensor:Analog:Position"
	fD	readwrite	29	  548
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  AnalogVelocityValue	
    "Sensor:Analog:Velocity"
	fD	readwrite	29	  549
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  AnalogVoltageValue	
    "Sensor:Analog:Voltage"
	fD	readwrite	29	  54A
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  AnalogInverted	
    "Sensor:Analog:Inverted"
	bool	readwrite	29	  7B
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  AnalogSampleDelta	
    "Sensor:Analog:Sample Delta ms"
	uL	readwrite	29	  7C
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  AltEncoderPositionValue	
    "Sensor:Alt Encoder:Position"
	fD	readwrite	29	  448
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  AltEncoderVelocityValue	
    "Sensor:Alt Encoder:Velocity"
	fD	readwrite	29	  449
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  AltEncoderInverted	
    "Sensor:Alt Encoder:Inverted"
	bool	readwrite	29	  83
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  AltEncoderCountsPerRev	
    "Sensor:Alt Encoder:Counts Per Rev"
	uL	readwrite	29	  80
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  AltEncoderAverageDepth	
    "Sensor:Alt Encoder:Average Depth"
	uL	readwrite	29	  81
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  AltEncoderSampleDelta	
    "Sensor:Alt Encoder:Sample Delta ms"
	uL	readwrite	29	  82
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"




attribute:  DutyCycleSensorPrescaler	
    "Sensor:Abs Encoder:Sensor Prescaler"
	uL	readwrite	29	  99
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  DutyCycleOffset	
    "Sensor:Abs Encoder:Offset"
	fD	readwrite	29	  9A
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  DutyCycleEncoderStartPulseUs	
    "Sensor:Abs Encoder:Start Pulse microsec"
	fD	readwrite	29	  C4
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  DutyCycleEncoderEndPulseUs	
    "Sensor:Abs Encoder:End Pulse microsec"
	fD	readwrite	29	  C5
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  CompatibilityPortConfig	
    "Sensor:Spark Data Port Mode"
	uL enum {"Default"=0, "Alt Encoder"=1}	readwrite	29	  7F
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"



attribute:  MAXMotionConfig_0	
    "MAXMotion:Slot 0"
	cluster {5, "Cruise Velocity"=fD, "Accel Limit"=fD, "Jerk Limit"=fD, "Goal Error"=fD, "Mode"=uL}	readwrite	29	  1A6
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  MAXMotionConfig_1	
    "MAXMotion:Slot 1"
	cluster {5, "Cruise Velocity"=fD, "Accel Limit"=fD, "Jerk Limit"=fD, "Goal Error"=fD, "Mode"=uL}	readwrite	29	  1A7
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  MAXMotionConfig_2	
    "MAXMotion:Slot 2"
	cluster {5, "Cruise Velocity"=fD, "Accel Limit"=fD, "Jerk Limit"=fD, "Goal Error"=fD, "Mode"=uL}	readwrite	29	  1A8
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  MAXMotionConfig_3	
    "MAXMotion:Slot 3"
	cluster {5, "Cruise Velocity"=fD, "Accel Limit"=fD, "Jerk Limit"=fD, "Goal Error"=fD, "Mode"=uL}	readwrite	29	  1A9
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"


attribute:  Status0Period	
    "Status:Updates:Period 0"
	uL	readwrite	29	  9E
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  Status1Period	
    "Status:Updates:Period 1"
	uL	readwrite	29	  9F
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  Status2Period	
    "Status:Updates:Period 2"
	uL	readwrite	29	  A0
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  Status3Period	
    "Status:Updates:Period 3"
	uL	readwrite	29	  A1
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  Status4Period	
    "Status:Updates:Period 4"
	uL	readwrite	29	  A2
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  Status5Period	
    "Status:Updates:Period 5"
	uL	readwrite	29	  A3
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  Status6Period	
    "Status:Updates:Period 6"
	uL	readwrite	29	  A4
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  Status7Period	
    "Status:Updates:Period 7"
	uL	readwrite	29	  A5
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  Status8Period	
    "Status:Updates:Period 8"
	uL	readwrite	29	  C7
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  Status9Period	
    "Status:Updates:Period 9"
	uL	readwrite	29	  E0
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  ForceEnableStatus_0	
    "Status:Updates:Force 0"
	bool	readwrite	29	  BA
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  ForceEnableStatus_1	
    "Status:Updates:Force 1"
	bool	readwrite	29	  BB
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  ForceEnableStatus_2	
    "Status:Updates:Force 2"
	bool	readwrite	29	  BC
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  ForceEnableStatus_3	
    "Status:Updates:Force 3"
	bool	readwrite	29	  BD
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  ForceEnableStatus_4	
    "Status:Updates:Force 4"
	bool	readwrite	29	  BE
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  ForceEnableStatus_5	
    "Status:Updates:Force 5"
	bool	readwrite	29	  BF
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  ForceEnableStatus_6	
    "Status:Updates:Force 6"
	bool	readwrite	29	  C0
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  ForceEnableStatus_7	
    "Status:Updates:Force 7"
	bool	readwrite	29	  C1
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  ForceEnableStatus_8	
    "Status:Updates:Force 8"
	bool	readwrite	29	  C8
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  ForceEnableStatus_9	
    "Status:Updates:Force 9"
	bool	readwrite	29	  E1
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"





attribute:  MotorFaults	
    "Status:Faults"
	typedef "<vilib>:REV Custom Controls\Spark Faults2.ctl"  readwrite	29	  302
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  MotorStickyFaults	
    "Status:Sticky Faults"
	typedef "<vilib>:REV Custom Controls\Spark Faults2.ctl"  readwrite	29	  303
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  MotorWarnings	
    "Status:Warnings"
	typedef "<vilib>:REV Custom Controls\Spark Warnings2.ctl"  readwrite	29	  304
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"

attribute:  MotorStickyWarnings	
    "Status:Sticky Warnings"
	typedef "<vilib>:REV Custom Controls\Spark Warnings2.ctl"  readwrite	29	  305
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"




attribute:  MotorDetails	
    "Motor Details"
	typedef "<vilib>:REV Custom Controls\c_REVLib_MotorDetails.ctl"  readwrite	29	  102
	"INCOMPLETE"	"REVSpark_Properties.html"	"..\..\help\REVSpark_Reference.chm"
#endif
