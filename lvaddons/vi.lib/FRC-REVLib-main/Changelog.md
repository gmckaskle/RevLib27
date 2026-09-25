# REVLib Changelog

## Unreleased

- [SPARK] Deprecates AbsoluteEncoderConfig.zeroCentered() - use rangeOffset() instead.
- [SPARK] Removes velocity averaging configurations in favor of new firmware filtering system.
- [SPARK] Adjusts default encoder average depth to 8 and sample delta to 20
- [MAXSpline Encoder] Simplifies driver

## 2027.0.0-alpha-6

- [A301] Java: Fixes setCurrent()

## 2027.0.0-alpha-5

- [REVLib] Java: Fixes crash in getPeriodicStatus8()
- [A301] Adds IdleMode getter and setter
- [A301] Fixes inconsistencies when inverted
- [A301] Adds a busId-only constructor to C++
- [A301/SPARK] Adds ability to modify the absolute position range about zero -- (-1.0, 0] to [0, 1.0)

## 2027.0.0-alpha-4

- [REVLib] Removes deprecated functions
- [REVLib] Fixes links in documentation
- [REVLib] Fixes Sim classes to include CAN Bus ID in name
- [A301] - Adds new A301-specific CAN specifications
- [A301] - Removed isContinuous argument from setAbsolutePosition(); replaced with (En/Dis)ableAbsolutePositionContinuousInput()
- [A301] - Adds a setRelativePositionWithSpeed() and setAbsolutePositionWithSpeed() to run position closed loop control with a speed constraint
- [A301] - Updates A301(int busId) constructor to autodetect the device ID instead of requiring the device to be the factory default value of 3

## 2027.0.0-alpha-3

- [A301] Adds setRelativeEncoderPosition()
- [A301] Fixes setInverted() and getInverted()
- [ServoHub] Fixes internal crash when calling Status getters
- [A301] Renames getOutputCurrent() to getMotorCurrent()

## 2027.0.0-alpha-2

- [A301] Adds support for A301
- [REVLib] Java/C++: Creates a Signal wrapper type for signals, allowing a user to know if a value is outdated. This is a breaking change and will require user code to call `.get()` / `.get(default)` in Java or `Get()` in C++. User code can query `.isValid()` in Java or `IsValid()` in C++ to know whether the value is recent.

## 2027.0.0-alpha-1

- [REVLib] Supports 2027 WPILib and running on systemcore

## 2026.0.5

- [SPARK] Adds warning for SPARK devices not on 2026 firmware

## 2026.0.4

- [REVLib] Fixes issue where the SplineEncoder was not fetching its status periods at creation
- [REVLib] Updates SplineEncoder documentation

## 2026.0.3

- [REVLib] Java/C++ - Fixes MAXSpline Encoder issues/crashes

## 2026.0.2

- [REVLib] Adds SignalsAccessor for MAXSpline Encoders
- [REVLib] Fixes the issue where DetachedEncoderSignals were not applied
- [REVLib] Fetches status periods from device on object creation
- [REVLib] Updates SPARK parameter descriptions
- [REVLib] Adds new MAXSpline Encoder configuration parameters (start/end pulse and absolute period) and accessors

## 2026.0.1

- [REVLib] Java/C++: Fixes simulation crash on MacOS
- [REVLib] C++: Fixes cpp check warnings
- [REVLib] LabVIEW: Updates for 2026 FRC LabVIEW

## 2026.0.0

### Major Changes

- [MAXSpline Encoder] Java/C++: Adds initial support for MAXSpline Encoders with `DetachedEncoder` class
- [REVLib] Java/C++: Adds `StatusLogger`, the Official REV-Compatible Logger adapted from [URCL](https://github.com/Mechanical-Advantage/URCL)
  - Requires the [revlog-converter](https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/revlog-converter-0.1) cli tool to convert revlogs to wpilogs
- [SPARK] Adds support for new feedforward parameters: `kV` (formerly `kF`), `kA`, `kS`, `kG`, `kCos`, and `kCosRatio`
- [SPARK] Renames MAXMotion parameters to be more descriptive (`kMaxVelocity` -> `kCruiseVelocity`, `kAllowedClosedLoopError` -> `kAllowedProfileError`)
- [SPARK] Java/C++: Adds simulation support and parity for new MAXMotion and feedforward features
- [SPARK] Adds support for new MAXMotion status signals: `MAXMotionSetpointPosition` and `MAXMotionSetpointVelocity`
- [SPARK] Adds support for closed loop status signals: `isAtSetpoint`, `setpoint`, and `selectedClosedLoopSlot`
- [SPARK] Adds the ability to set allowed closed loop error when using regular PID control
- [SPARK] Java/C++: Adds `SparkSoftLimit` class for getting soft limit status
- [SPARK] Java/C++: Adds `getControlType()` to get the selected control type (last used in calling `setReference()`/`setSetpoint()`)
- [SPARK] Java/C++: Adds configuration presets for various motors
- [SPARK] Java/C++: Adds configuration presets for REV through-bore encoders (V1 and v2) and MAXSpline Encoder (when used via the 6-pin JST) for primary encoder, external/alternate encoder, and absolute encoders
- [SPARK] Java/C++: Removes automatic clear faults call when creating a SparkFlex/SparkMax object. You can still call clearFaults manually if you wish.
- [Servo Hub] Java/C++: Removes automatic clear faults call when creating a ServoHub object. You can still call clearFaults manually if you wish.
- [REVLib] Java/C++: Refactors `ResetMode` and `PersistMode` to be a single common enum instead of device specific
- [SPARK] Java/C++: Deprecates `setReference()` in favor of `setSetpoint()`
- [SPARK] Removes SmartMotion in favor of MAXMotion

### Other Changes

- [REVLib] Java/C++: Fixes potential dangling reference in configure async calls
- [REVLib] Java/C++: Fixes memory leaks in daemons
- [SPARK] C++: Fixes crash when setting three or more signals within the same periodic status
- [SPARK] Java/C++: Fixes memory leaks in simulation when certain devices are never created
- [ServoHub] Java: Fixes `getChannelDisableBehavior()` always returning `kDoNotSupplyPower` in Java
- [SPARK] Java/C++: Fixes bug causing stale parameter reads
- [REVLib] Java/C++: Fixes issue with booleans in simulation
- [REVLib] Java: Prevents segmentation fault in multithreaded Java code that may call close
- [SPARK] Java: Adds getters for Periodic Status Frames for SPARKs
- [SPARK] Java/C++: Improves SPARK model detection
- [Servo Hub] Java: Makes `ServoHub` class AutoCloseable
- [Servo Hub] Java: Return null for periodic status getters if reading the frame failed

## 2026.0.0-beta-2

- [Servo Hub] Removes automatic clear faults call when creating a ServoHub object. You can still call clearFaults manually if you wish.
- [SPARK] Removes automatic clear faults call when creating a SparkFlex/SparkMax object. You can still call clearFaults manually if you wish.
- [Spark] Removes SmartMotion
- [Driver]: Adds more descriptive message when a driver is not set
- [Spline Encoder] Java: Adds DetachedEncoder class
- [SPARK] Removes SmartMotion
- [SPARK] Java: Adds getters for Periodic Status Frames for SPARKs
- [SPARK] Java: Adds common RevDevice interface for REV devices
- [SPARK] Fixes bug causing stale parameter reads
- [SPARK] Improves SPARK model detection
- [SPARK, Servo Hub] Prevents SigSegv in multithreaded Java code that may call close
- [Servo Hub] Makes ServoHub's Java class AutoCloseable
- [ServoHub] Java: Return null for periodic status getters if reading the frame failed
- [REVLib] Fixes issue with booleans in simulation
- [REVLib] Adds support for MAXSpline Encoders
- [REVLib] Updates to latest CAN specifications


## 2026.0.0-alpha-1

### Major Changes

- [REVLib] Java/C++: Adds `StatusLogger`, the Official REV-Compatible Logger adapted from [URCL](https://github.com/Mechanical-Advantage/URCL)
  - Requires the [revlog-converter](https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/revlog-converter-0.1) cli tool to convert revlogs to wpilogs
- [SPARK] Adds support for new feedforward parameters: `kV` (formerly `kF`), `kA`, `kS`, `kG`, `kCos`, and `kCosRatio`
- [SPARK] Renames MAXMotion parameters to be more descriptive (`kMaxVelocity` -> `kCruiseVelocity`, `kAllowedClosedLoopError` -> `kAllowedProfileError`)
- [SPARK] Java/C++: Adds simulation support and parity for new MAXMotion and feedforward features
- [SPARK] Adds support for new MAXMotion status signals: `MAXMotionSetpointPosition` and `MAXMotionSetpointVelocity`
- [SPARK] Adds support for closed loop status signals: `isAtSetpoint`, `setpoint`, and `selectedClosedLoopSlot`
- [SPARK] Adds the ability to set allowed closed loop error when using regular PID control
- [SPARK] Java/C++: Adds `SparkSoftLimit` class for getting soft limit status
- [SPARK] Java/C++: Adds `getControlType()` to get the selected control type (last used in calling `setReference()`/`setSetpoint()`)
- [SPARK] Java/C++: Deprecates `setReference()` in favor of `setSetpoint()`

### Fixes

- [REVLib] Java/C++: Fixes potential dangling reference in configure async calls
- [REVLib] Java/C++: Fixes memory leaks in daemons
- [SPARK] C++: Fixes crash when setting three or more signals within the same periodic status
- [SPARK] Java/C++: Fixes memory leaks in simulation when certain devices are never created
- [ServoHub] Java: Fixes `getChannelDisableBehavior()` always returning `kDoNotSupplyPower` in Java

## 2025.0.3

### Changes for Java and C++

- [SPARK] Improves documentation concerning Relative Encoders and Position and Velocity Conversion Factors
- [SPARK] Removes `setPositionConversionFactor()` and `setVelocityConversionFactor()` methods from the sim classes
  - Instead, use the appropriate Config objects and `positionConversionFactor()` and `velocityConversionFactor()` methods
- [SPARK] Fixes crash when calling `Spark[Flex, Max].configureAsync()` in simulation
- [SPARK] Adds ability to automatically set the position when a limit switch is triggered
  - Adds LimitSwitchConfig.forwardLimitSwitchTriggerBehavior and LimitSwitchConfig.reverseLimitSwitchTriggerBehavior
  - Deprecates LimitSwitchConfig.forwardLimitSwitchEnabled and LimitSwitchConfig.reverseLimitSwitchEnabled
  - Adds LimitSwitchConfig.forwardLimitSwitchPosition and LimitSwitchConfig.reverseLimitSwitchPosition methods to set the position
  - Adds LimitSwitchConfig.limitSwitchPositionSensor to select the feedback sensor to set the position on
- [SPARK] Improves validation and descriptions of parameters

### Changes for LabVIEW

- Fixes issue where an error would be incorrectly generated when configuring follower mode

## 2025.0.2

### Changes for Java and C++

- [SPARK] Improves SPARK error messages by adding the invalid value which caused the error
- [SPARK] Improves SPARK error messages by displaying the parameter name as well as its ID
- [Servo Hub] Fixes uninitialized variables
- [REVLib] Fixes issue where REVLib doesn't clear a previous error (as viewed through `GetLastError`)
- [REVLib] Fixes threading issues encountered while running Googletest unit tests

## 2025.0.1

### Changes for Java and C++

- [SPARK] Fixes issue where enabling a limit switch in Java simulation would cause it to always return that it was pressed
- [SPARK] Fixes issue causing REVLib to have higher than normal CPU usage when retrieving SPARK status frames
- [SPARK] Fixes issue where a status frame timeout would cause a segmentation fault

## 2025.0.0

### Major Changes

- [REVLib] Requires non-prerelease versions of SPARK and Servo Hub firmware v25.0.0 or higher
- [SPARK] Java/C++: Moves to a more declarative approach for configuring devices
  - Adds `SparkFlexConfig`, `SparkMaxConfig` which includes settings for different aspects of each device
  - Adds `configure()` method to apply a config object's settings to one or more devices of the correct type
  - Adds `configureAsync()` to configure a device without blocking the program
  - Adds a `configAccessor` field to device classes for reading configuration parameters directly from the device
- [SPARK] Java/C++: Adds better support for simulation
  - Moves away from `REVPhysicsSim` to offer better support for WPILib physics simulation instead
  - Revamps simulation GUI data, including brand new fields for auxiliary devices
  - Adds Sim classes for each auxiliary device, allowing for more thorough simulation in the WPILib injection style
  - Adds `SparkSim.iterate()` method which features simulated current limits, closed-loop control, and more
  - Adds `SparkSimFaultManager` for throwing simulated faults
- [SPARK] Adds support for MAXMotion
  - Adds control types `MAXMotionPositionControl` and `MAXMotionVelocityControl`
  - Adds `MAXMotionConfig`. Only trapezoidal profile is available at this time.
  - MAXMotion is not a drop-in replacement for Smart Motion, as you will need to retune PID gains.
- [SPARK] Improves experience with managing status signals from SPARK devices
  - Adds `SignalsConfig` to adjust signal periods and always on setting
  - Automatically enables relevant status frames if a signal is requested by the user
- [Servo Hub] Java/C++: Adds initial support for Servo Hub
  - Follows the same paradigms used for SPARK
  - Includes basic simulation support for Servo Hub

### Breaking Changes

- [SPARK] Renames `CANSparkFlex` and `CANSparkMax` to `SparkFlex` and `SparkMax` respectively
- [SPARK] Renames `SparkPIDController` to `SparkClosedLoopController`
- [SPARK] Removes configuration parameter setter/getter methods. Use `SparkBase.configure()` and `SparkBase.configAccessor` instead.
- [SPARK] Removes `burnFlash()` and `restoreFactoryDefaults()`. Use the `ResetMode` and `PersistMode` options in `SparkBase.configure()` instead.
- [SPARK] Removes `REVPhysicsSim` in favor of new simulation system
- [SPARK] Removes async mechanism for setting parameters by setting CAN timeout to 0 in favor of `configureAsync()`
- [SPARK] Moves all SPARK related classes into a `spark` package in Java and namespace in C++
- [SPARK] LabVIEW: Reworks entire VI palette
  - Improves organization of VI palette by separating VIs by configuration, device status, and utility
  - Moves towards increased usage of polymorphic VIs for easier navigation of the palette

### Other Changes

- [SPARK] Fixes issue where multiple setpoint commands would be sent when switching control types on a SPARK, resulting in the motor oscillating between the different setpoints
- [SPARK] Deprecates `kSmartMotion` and `kSmartVelocity` control types in favor of `kMAXMotionPositionControl` and `kMAXMotionVelocityControl` respectively.
- [SPARK] Deprecates `SparkBase.setInverted()` and `SparkBase.getInverted()` in favor of using the new configuration system
- [SPARK] Updates `ClosedLoopController.setReference()` to use the `ClosedLoopSlot` enum instead of an int
- [SPARK] Improves error description when attempting to persist parameters while the robot is enabled
- [SPARK] Improves getting faults/warnings by returning a `Faults` or `Warnings` object
  - The raw bits of faults and warnings are available as a field in the respective struct
- [SPARK] Adds `hasActiveFault()`, `hasStickyFault()`, `hasActiveWarning()`, and `hasStickyWarning()` to check if there is a fault/warning present at all on the SPARK device
- [SPARK] Adds `pauseFollowerMode()` and `resumeFollowerMode()`
- [SPARK] Adds ability for follower mode to work even if the follower is not referenced in user code
- [SPARK] Adds support for specifying an absolute encoder's duty cycle start and end pulse widths in `AbsoluteEncoderConfig`
- [SPARK] Adds configuration option for setting whether the absolute encoder is zero-centered
- [REVLib] Fixes potential memory leaks in string handling
- [SPARK] LabVIEW: Improves reliability of CAN transactions by adding a retry mechanism

## 2025.0.0-rc-1

### Changes for Java and C++

- Moves `ClosedLoopSlot` out of config to the top level
  - Imports for `ClosedLoopSlot` will have to be updated
- Updates `ClosedLoopController.setReference()` to use the `ClosedLoopSlot` enum
- Improves error description when attempting to persist parameters while the robot is enabled
- Fixes issue where the Start Follower Mode Command reported a CAN Error
- Adds simulation support to Servo Hub
- Fixes error reporting to show a ServoHub prefix before Servo hub errors
- Optimizes Servo Hub parameter resetting
- Adds usage reporting for Servo Hub
- Adds async versions of certain methods like `configure()` for SPARK and Servo Hub
- Requires non-prerelease versions of SPARK and Servo Hub firmware v25.0.0 or higher
- Fixes issue where setpoint would always appear to be 0 when simulating in C++
- Fixes issue where having an absolute encoder as a feedback sensor would have incorrect conversion factors when simulating in C++

## 2025.0.0-beta-4

### Changes for Java and C++

- Adds support for Servo Hub
- Adds missing config accessor for absolute encoder zero offset for SPARK
- Fixes memory leaks in simulation for SPARK
- Fixes potential memory leaks in string handling
- Fixes issue where default control mode for SPARK was not being set correctly in simulation, causing errors in console with `iterate()`
- Fixes issue where the SPARK primary encoder simulation device does not get freed properly

### Known Issues

- Setting configurations on Servo Hub in simulation will fail and result in an error to the console

## 2025.0.0-beta-3

### Improvements for Java and C++

- Adds fields to `Faults` and `Warnings` structs for the raw bits representation of faults and warnings
- Adds configuration option for setting whether the absolute encoder is zero-centered
- Corrects simulation Conversion Factor default values to 1.0
  - Fixes issue where sensor `.iterate` methods would not set the position correctly
- Fixes documentation for `SparkMax.configure()`
- Fixes issue where multiple setpoint commands would be sent when switching control types on a SPARK, resulting in the motor oscillating between the different setpoints
- Deprecates `SparkBase.setInverted()` and `SparkBase.getInverted()` in favor of using the new configuration system

## 2025.0.0-beta-2

### Fixes for Java and C++

- Fixes issue where setting certain parameters (follower mode, MAXMotion, and status signals) in simulation erroneously throws an exception
- Fixes issue where retrieving auxiliary objects from a SPARK MAX (alternate encoder, absolute encoder, and limit switches) before setting the data port config with `configure()` can erroneously throw an exception

## 2025.0.0-beta-1

### Major Changes

- Moves to a more declarative approach for configuring SPARK devices
  - Adds `SparkFlexConfig` and `SparkMaxConfig` which includes settings for different aspects of the SPARK
  - Adds `SparkBase.configure()` method to apply a SPARK config object's settings to one or more SPARK devices
- Adds support for MAXMotion
  - Adds control types `MAXMotionPositionControl` and `MAXMotionVelocityControl`
  - Adds `MAXMotionConfig`. Only trapezoidal profile is available at this time.
  - MAXMotion is not a drop-in replacement for Smart Motion, as you will need to retune PID gains.
- Adds better support for simulation
  - Moves away from `REVPhysicsSim` to offer better support for WPILib physics simulation instead
  - Revamps simulation GUI data, including brand new fields for auxiliary devices
  - Adds Sim classes for each auxiliary device, allowing for more thorough simulation in the WPILib injection style
  - Adds `SparkSim.iterate()` method which features simulated current limits, closed-loop control, and more
  - Adds `SparkSimFaultManager` for throwing simulated faults
- Improves experience with managing status signals from SPARK devices
  - Adds `SignalsConfig`
  - Automatically enables relevant status frames if a signal is requested by the user
- Adds a `configAccessor` field to SPARK classes for reading configuration parameters directly from the device

### Breaking Changes

- Renames `CANSparkFlex` and `CANSparkMax` to `SparkFlex` and `SparkMax` respectively
- Renames `SparkPIDController` to `SparkClosedLoopController`
- Removes configuration parameter setter/getter methods. Use `SparkBase.configure()` and `SparkBase.configAccessor` instead.
- Removes `burnFlash()` and `restoreFactoryDefaults()`. Use the `ResetMode` and `PersistMode` options in `SparkBase.configure()` instead.
- Removes `REVPhysicsSim` in favor of new simulation system
- Moves all SPARK related classes into a `spark` package in Java and namespace in C++

### Other Changes

- Deprecates `kSmartMotion` and `SmartVelocity` control types in favor of `MAXMotionPositionControl` and `MAXMotionVelocityControl` respectively.
- Adds `pauseFollowerMode()` and `resumeFollowerMode()`
- Allows follower mode to work even if the follower is not referenced in user code
- Improves getting faults/warnings by returning a `Faults` or `Warnings` object
- Adds `hasActiveFault()`, `hasStickyFault()`, `hasActiveWarning()`, and `hasStickyWarning()` to check if there is a fault/warning present at all on the SPARK device
- Adds support for specifying an absolute encoder's duty cycle start and end pulse widths in `AbsoluteEncoderConfig`

## 2024.2.4

### Changes to C++ and Java

- Increases the default timeout to wait for a periodic status from 2*framePeriodMs to 500ms.
  - Reduces possibility for large, inaccurate jumps in data to occur when retrieving from status frames.
  - Reduces amount of "timed out while waiting for periodic status X" errors in driver station.
  - Adds `setPeriodicFrameTimeout()` to configure the CAN timeout for periodic status frames. See code docs for more information.
- Improves reliability of RTR CAN frames like setting parameters and other commands that expect a response from the device.
  - Adds mechanism to retry requests if sending the request or receiving the response failed. The default value for maximum number of retries is 5.
  - Adds `setCANMaxRetries()` to configure the value for maximum number of retries. See code docs for more information.
- Fixes undefined behavior when SPARK motor controller information cannot be retrieved during initialization.

## 2024.2.3

### Changes to Java

- Fixes issue introduced in v2024.2.2 where calling getEncoder() multiple times can cause a fatal exception in certain circumstances.

### Changes to C++ and Java

- Removes dynamic check for SPARK model when calling getEncoder(), causing unnecessary CAN traffic.
- Moves zero argument CANSparkBase.getEncoder() to CANSparkMax and CANSparkFlex subclasses to determine default encoder values.

## 2024.2.2

### Changes to C++ and Java

- Fixes issue where configuring the velocity filter for the default relative encoder of a SPARK Flex would not set the correct parameters.

### Changes to Java

- Improves memory allocation performance.

## 2024.2.1

- C++/Java: Changes behavior of SPARK Flex and MAX initialization errors to throw exceptions rather than terminating the robot program.
- C++/Java: Fixes issue where initializing a SPARK Flex or MAX in brushed mode while the device is disconnected from the CAN bus causes the robot program to terminate.
- C++/Java: Fixes issue where initializing a SPARK Flex or MAX in brushed mode causes robot simulation to terminate.
- C++/Java: Fixes warning about using the wrong class for a SPARK Flex or MAX during robot simulation.
- C++: Fixes ambiguous overload error when no parameters are supplied when calling `GetAnalogSensor()`.
- C++: Fixes ambiguous overload error when no parameters are supplied when calling `GetEncoder()`.

## 2024.2.0

### Changes to C++, Java, and LabVIEW

- Throws an error if firmware version is less than 24.0.0
- Throws an error if the motor type is set to Brushed on a SPARK Flex while a SPARK Flex Dock is not connected
- Gets main encoder position with enhanced precision

### Changes to C++ and Java

- Sends a warning to the Driver Station if the wrong class is used for the type of SPARK that is connected
- Adds `CanSparkBase` class that exposes functionality that is common to both the SPARK MAX and the SPARK Flex
- Adds `CanSparkFlex` class that exposes all functionality of the SPARK Flex
  - `CanSparkFlex` has a `getExternalEncoder()` method that returns a `SparkFlexExternalEncoder` instead of a `getAlternateEncoder()` method that returns a `SparkMaxAlternateEncoder`.
  - This is because Alternate Encoder Mode is not necessary for SPARK Flex, and has been replaced by the External Encoder Data Port feature:
    - Can be used simultaneously with the internal encoders in NEO class motors
    - Can be used simultaneously with an absolute encoder and limit switches
    - Virtually no RPM limit
    - No special configuration
- The following items have been deprecated in favor of new equivalents:
  - Instead of `CANSparkMaxLowLevel`, use `CANSparkLowLevel`
  - Instead of `SparkMaxAbsoluteEncoder`, use `SparkAbsoluteEncoder`
  - Instead of `SparkMaxAnalogSensor`, use `SparkAnalogSensor`
  - Instead of `SparkMaxLimitSwitch`, use `SparkLimitSwitch`
  - Instead of `SparkMaxPIDController`, use `SparkPIDController`
  - Instead of `SparkMaxRelativeEncoder`, use `SparkRelativeEncoder`
  - Instead of `ExternalFollower.kFollowerSparkMax`, use `ExternalFollower.kFollowerSpark`
    - The `ExternalFollower` enum can be accessed at `CANSparkMax.ExternalFollower`, `CANSparkFlex.ExternalFollower`, or `CANSparkBase.ExternalFollower`
- Adds a `CANSparkBase.getSparkModel()` method that returns a `SparkModel` enum

### Changes to LabVIEW

- Deprecates old VIs that are prefixed with "Spark MAX" and replaces them with VIs prefixed with "SPARK"
  - Deprecated icons are "grayed out"
  - Help context (documentation) for deprecated VIs point the user to the equivalent new VI
  - New icons say "SPARK" instead of "REV MAX"
- Adds `SPARK Get Model.vi`
- Fixes `SPARK Get Analog Sensor Voltage.vi` when used with a SPARK Flex
- Updates `SPARK Get I Accum.vi` to get I Accum from status 7 instead of status 2
- Updates "Alternate Encoder" VIs to be "Alternate or External Encoder"
  - Only throw the data port config warnings when the device is a SPARK MAX

## 2024.1.1

- Compatible with SPARK Flex firmware 23.x.x and SPARK MAX firmware 1.6.x
- Adds `CanSparkBase` class that exposes functionality that is common to both the SPARK MAX and the SPARK Flex
- Adds `CanSparkFlex` class that exposes all functionality of the SPARK Flex
  - `CanSparkFlex` has a `getExternalEncoder()` method that returns a `SparkFlexExternalEncoder` instead of a `getAlternateEncoder()` method that returns a `SparkMaxAlternateEncoder`.
  - This is because Alternate Encoder Mode is not necessary for SPARK Flex, and has been replaced by the External Encoder Data Port feature:
    - Can be used simultaneously with the internal encoders in NEO class motors
    - Can be used simultaneously with an absolute encoder and limit switches
    - Virtually no RPM limit
    - No special configuration
- The following items have been deprecated in favor of new equivalents:
  - Instead of `CANSparkMaxLowLevel`, use `CANSparkLowLevel`
  - Instead of `SparkMaxAbsoluteEncoder`, use `SparkAbsoluteEncoder`
  - Instead of `SparkMaxAnalogSensor`, use `SparkAnalogSensor`
  - Instead of `SparkMaxLimitSwitch`, use `SparkLimitSwitch`
  - Instead of `SparkMaxPIDController`, use `SparkPIDController`
  - Instead of `SparkMaxRelativeEncoder`, use `SparkRelativeEncoder`
  - Instead of `ExternalFollower.kFollowerSparkMax`, use `ExternalFollower.kFollowerSpark`
    - The `ExternalFollower` enum can be accessed at `CANSparkMax.ExternalFollower`, `CANSparkFlex.ExternalFollower`, or `CANSparkBase.ExternalFollower`
- Adds a `CANSparkBase.getSparkModel()` method that returns a `SparkModel` enum

## 2024.0.0

- Updates library to be compatible with WPILib 2024 beta

## 2023.1.3

- Improves documentation for the setZeroOffset() and getZeroOffset() methods on Absolute Encoder objects
- Fixes issue where reading an absolute encoder’s zero offset could return an incorrect value in certain conditions

## 2023.1.2

- Adds support to configure the hall sensor's velocity measurement
  - C++/Java: Updates `SetMeasurementPeriod()` and `SetAverageDepth()` in the `SparkMaxRelativeEncoder` class to be used when the relative encoder is configured to be of type `kHallSensor`.
  - LabVIEW: Adds `SPARK MAX Configure Hall Sensor.vi` and `SPARK MAX Get Hall Sensor Config.vi` to set and get the hall sensor's measurement period and average depth.

## 2023.1.1

- Adds support for using a duty cycle absolute encoder as a feedback device for the SPARK MAX.
  - C++/Java: Adds `SparkMaxAbsoluteEncoder` class.
  - LabVIEW: Adds VIs for configuring and getting the values from a duty cycle absolute encoder.
- Adds Position PID Wrapping to allow continuous input for the SPARK MAX PID controller.
  - C++/Java: Adds `PositionPIDWrapping` methods to the `SparkMaxPIDController` class.
  - LabVIEW: Adds VIs for setting and getting the Position PID Wrapping configuration.
- Allows configuring the periodic frame rates for status frames 4-6.

## 2023.0.1

- Adds support for osxuniversal

## 2023.0.0

- Updates library to be compatible with WPILib 2023 beta

## 2022.1.2

- LabVIEW: Adds `Spark MAX Set Inverted.vi` and `Spark MAX Get Inverted.vi`

## 2022.1.1

- Adds Linux aarch64 (64-bit ARM) build
- C++: Adds missing `GetAlternateEncoder(int countsPerRev)` method

## 2022.1.0

### Enhancements

- Java: Adds initial WPILib simulation support
  - Supports `ControlType.kVelocity` and `ControlType.kVoltage`
  - To use, make the following modifications to your Robot class (adjust parameters as necessary):
    - Call `RevPhysicsSim.getInstance().addSparkMax(sparkMax, DCMotor.getNEO(1))` from `simulationInit()`
    - Call `RevPhysicsSim.GetInstance.run()` from `simulationPeriodic()`
    - These changes will keep the simulated position value up-to-date.
  - Limitations
    - When in simulation mode, calling `setReference()` will only update the velocity of the primary encoder, even if `SparkMaxPIDController.setFeedbackDevice()` was called with a different feedback sensor

### Fixes

- C++: Fixes move semantics for supported classes

## 2022.0.0

### Breaking changes

- C++/Java: `CANError` has been renamed to `REVLibError`.
- Java: `ColorMatch.makeColor()` and the `ColorShim` class have been removed. Use the WPILib `Color` class instead.
- C++/Java: Deleted deprecated constructors, methods, and types
  - Replace deprecated constructors with `CANSparkMax.getX()` functions.
  - Replace `CANEncoder.getCPR()` with `getCountsPerRevolution()`.
  - Remove all usages of `CANDigitalInput.LimitSwitch`.
  - Replace `CANSparkMax.getAlternateEncoder()` with `CANSparkMax.getAlternateEncoder(int countsPerRev)`.
  - Remove all usages of `CANSparkMax.setMotorType()`. You can only set the motor type in the constructor now.
  - Replace `SparkMax` with `PWMSparkMax`, which is built into WPILib.
- Java: `CANSparkMax.get()` now returns the velocity setpoint set by `set(double speed)` rather than the actual velocity, in accordance with the WPILib `MotorController` API contract.
- C++/Java: `CANPIDController.getSmartMotionAccelStrategy()` now returns `SparkMaxPIDController.AccelStrategy`.
- C++/Java: Trying to do the following things will now throw an exception:
  - Creating a `CANSparkMax` object for a device that already has one
  - Specifying an incorrect `countsPerRev` value for a NEO hall sensor
  - Java: Calling a `CANSparkMax.getX()` method using different settings than were used previously in the program
  - Java: Trying to use a `CANSparkMax` (or another object retrieved from it) after `close()` has been called
  - C++: Calling a `CANSparkMax.getX()` method more than once for a single device
- C++/Java: Deprecated classes in favor of renamed versions
  - C++ users will get `cannot declare field to be of abstract type` errors until they replace their object declarations with ones for the new classes. Java users will be able to continue to use the old classes through the 2022 season.
  - `AlternateEncoderType` is replaced by `SparkMaxAlternateEncoder.Type`.
  - `CANAnalog` is replaced by `SparkMaxAnalogSensor`.
  - `CANDigitalInput` is replaced by `SparkMaxLimitSwitch`.
  - Java: `CANEncoder` is replaced by `RelativeEncoder`.
  - C++: `CANEncoder is replaced by `SparkMaxRelativeEncoder` and `SparkMaxAlternateEncoder`.
  - `CANPIDController` is replaced by `SparkMaxPIDController`.
  - `CANSensor` is replaced by `MotorFeedbackSensor`.
  - `ControlType` is replaced by `CANSparkMax.ControlType`.
  - `EncoderType` is replaced by `SparkMaxRelativeEncoder.Type`.

### Enhancements

- C++/Java: Added the ability to set the rate of periodic frame 3

### Fixes

- C++/Java: `CANSparkMax.getMotorType()` no longer uses the Get Parameter API, which means that it is safe to call frequently
- Java: The `CANSparkMax.getX()` methods no longer create a new object on every call

## 1.5.0

- Adds the ability to use an alternate enocder as a feedback device when connected to the top port of the SPARK MAX. When using an alternate encoder, Hard Limit Switches cannot be used on that SPARK MAX.
- Adds the ability to send telemetry data back from a SPARK MAX. This is done by opening a Telemetry Stream for a particular subset of telemetry data (categorized by TelemetryIDs). A TelemetryMessage contains the ID, value, timestamp, name, units, and bounds of a particular TelemetryID. Additionally, users can list what subset of telemtry data is available for each SPARK MAX.
- Adds a DeviceScanner that will scan the CANBus for other CAN Devices. Will filter what devices to look for, and users can specify filters to let through different devices.
- Addresses bug with CANEncoder backwards compatability
- Addresses bug with the Java Control Frame Period not being properly set

## 1.4.0

- Adds ability to use both absolute and relative analog sensors as feedback devices
- Adds non-roboRIO heartbeat for non-roboRIO targets
- Adds analog filtering for velocity readings
- Moved the device inversion logic to the firmware level. Limit switch and soft limit directions now follow the current inversion state of the device as opposed to the original state of the device
- Adds a configurable range for absolute feedback devices (currently only applicable to an absolute mode analog sensor), which also prevents users from setting a setpoint out of range
- Adds ability to configure how errors are tracked and handled by the user. Calls can be automatically registered and tracked, with any errors displayed to the DriverStation or users can use the GetLastError after calls to determine if an error has been thrown. This is done by changing the error timeout through SetCANTimeout(), where a timeout of 0 means that the calls are automatically tracked

## 1.3.0

- Ability to configure the feedback device for the PIDController
- Addition of CANAnalog which will function as a possible feedback device
- Added API for using encoders with brushed DC motors
- Will unfollow SPARK MAX leader regardless of CAN ID passed to method

## 1.2.6

- Creates new implementation for buffering and sending error messages
- Adds error codes to expand CANError class to more generic errors
- Vendor deps now allows building Windowsx86-64 based build target
- Moves entire implementation to C level, C++ and Java libraries are now simple wrappers
- Adds API to enable and set soft limits
- Fixes possible driver set out of bounds if given a CAN ID less than 0
- CANSparkMaxLowLeveL::SetEncPosition() and CANSparkMaxLowLeveL::SetIAccum() have been moved to protected and should be accessed via their respective objects (CANEncoder.SetEncoderPosition() and CANPIDController::SetIAccum() respectively)
- CANSparkMaxLowLeveL::SetParameter\* and GetParameter\* are removed and are not needed. APIs expose the detail needed for these functions, if there is no API then the function is not available anyway (this is an implementation detail that should have been hidden from the start)

## 1.2.1

- Fixes possible overflow issue in Java motor current measurement

## 1.2.0

- Adds initial CAN bootloading functionality. Requires continuous power during update and requires USB recovery if the update fails after erasing the flash (i.e. power is lost during update.)
- Adds ability to change units for arbFF between voltage and percent bus voltage (or percent compensated voltage)
- API adds check if its own version is too old based on major and minor revision numbers
- Adds version number to API

## 1.1.9

- Moves all CAN calls in set()/setReference() to a thread and called non-blocking, in JNI for Java
- Other improvements to set()/setReference()
- Improvements to timing for heartbeat implementation, all moved into C/JNI layer

## 1.1.8

- Adds smart velocity mode
- Fixes inversion issues for IMaxAccum and Output Range
- Updates heartbeat implementation
- Adds default slotID for setDFilter() functions
- Updates heartbeat rate from 50ms to 25ms
- Changes fault flag for Overvoltage to IWDTReset

## 1.1.6

- Improvements to setInverted()
- New API calls for new firmware features
- Better error message in Java when CAN is not connected
- Doc updates are in progress
