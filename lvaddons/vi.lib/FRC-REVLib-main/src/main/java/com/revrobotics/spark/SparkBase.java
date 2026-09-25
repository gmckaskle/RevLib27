/*
 * Copyright (c) 2018-2026 REV Robotics
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

package com.revrobotics.spark;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.jni.CANSparkJNI;
import com.revrobotics.jni.REVLibJNI;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.util.Signal;
import org.wpilib.driverstation.DriverStationErrors;

public abstract class SparkBase extends SparkLowLevel {
  public static class Faults {
    public final boolean other;
    public final boolean motorType;
    public final boolean sensor;
    public final boolean can;
    public final boolean temperature;
    public final boolean gateDriver;
    public final boolean escEeprom;
    public final boolean firmware;
    public final int rawBits;

    public Faults(int faults) {
      rawBits = faults;
      other = (faults & 0x1) != 0;
      motorType = (faults & 0x2) != 0;
      sensor = (faults & 0x4) != 0;
      can = (faults & 0x8) != 0;
      temperature = (faults & 0x10) != 0;
      gateDriver = (faults & 0x20) != 0;
      escEeprom = (faults & 0x40) != 0;
      firmware = (faults & 0x80) != 0;
    }

    public Faults(
        boolean other,
        boolean motorType,
        boolean sensor,
        boolean can,
        boolean temperature,
        boolean gateDriver,
        boolean escEeprom,
        boolean firmware) {
      this.other = other;
      this.motorType = motorType;
      this.sensor = sensor;
      this.can = can;
      this.temperature = temperature;
      this.gateDriver = gateDriver;
      this.escEeprom = escEeprom;
      this.firmware = firmware;

      rawBits =
          getBit(other, 0)
              | getBit(motorType, 1)
              | getBit(sensor, 2)
              | getBit(can, 3)
              | getBit(temperature, 4)
              | getBit(gateDriver, 5)
              | getBit(escEeprom, 6)
              | getBit(firmware, 7);
    }

    private int getBit(boolean isSet, int index) {
      return isSet ? (1 << index) : 0;
    }
  }

  public static class Warnings {
    public final boolean brownout;
    public final boolean overcurrent;
    public final boolean escEeprom;
    public final boolean extEeprom;
    public final boolean sensor;
    public final boolean stall;
    public final boolean hasReset;
    public final boolean other;
    public final int rawBits;

    public Warnings(int warnings) {
      rawBits = warnings;
      brownout = (warnings & 0x1) != 0;
      overcurrent = (warnings & 0x2) != 0;
      escEeprom = (warnings & 0x4) != 0;
      extEeprom = (warnings & 0x8) != 0;
      sensor = (warnings & 0x10) != 0;
      stall = (warnings & 0x20) != 0;
      hasReset = (warnings & 0x40) != 0;
      other = (warnings & 0x80) != 0;
    }

    public Warnings(
        boolean brownout,
        boolean overcurrent,
        boolean escEeprom,
        boolean extEeprom,
        boolean sensor,
        boolean stall,
        boolean hasReset,
        boolean other) {
      this.brownout = brownout;
      this.overcurrent = overcurrent;
      this.escEeprom = escEeprom;
      this.extEeprom = extEeprom;
      this.sensor = sensor;
      this.stall = stall;
      this.hasReset = hasReset;
      this.other = other;
      rawBits =
          getBit(brownout, 0)
              | getBit(overcurrent, 1)
              | getBit(escEeprom, 2)
              | getBit(extEeprom, 3)
              | getBit(sensor, 4)
              | getBit(stall, 5)
              | getBit(hasReset, 6)
              | getBit(other, 7);
    }

    private int getBit(boolean isSet, int index) {
      return isSet ? (1 << index) : 0;
    }
  }

  protected SparkRelativeEncoder encoder;
  protected final Object encoderLock = new Object();

  protected SparkAnalogSensor analogSensor;
  protected final Object analogSensorLock = new Object();

  protected SparkAbsoluteEncoder absoluteEncoder;
  protected final Object absoluteEncoderLock = new Object();

  protected SparkClosedLoopController closedLoopController;
  protected final Object closedLoopControllerLock = new Object();

  protected SparkLimitSwitch forwardLimitSwitch;
  protected final Object forwardLimitSwitchLock = new Object();

  protected SparkLimitSwitch reverseLimitSwitch;
  protected final Object reverseLimitSwitchLock = new Object();

  protected SparkSoftLimit forwardSoftLimit;
  protected final Object forwardSoftLimitLock = new Object();

  protected SparkSoftLimit reverseSoftLimit;
  protected final Object reverseSoftLimitLock = new Object();

  // Only used for get() and set() API
  private double m_setpoint = 0.0;

  /**
   * Create a new object to control a SPARK motor Controller
   *
   * @param busId The CAN bus ID this device will be on.
   * @param deviceId The device ID.
   * @param type The motor type connected to the controller. Brushless motor wires must be connected
   *     to their matching colors and the hall sensor must be plugged in. Brushed motors must be
   *     connected to the Red and Black terminals only.
   */
  public SparkBase(int busId, int deviceId, MotorType type, SparkModel model) {
    super(busId, deviceId, type, model);
  }

  /** ** Motor Controller Interface *** */
  /**
   * Sets the throttle of the motor controller.
   *
   * @param throttle The throttle where -1 indicates full reverse and 1 indicates full forward.
   */
  @Override
  public void setThrottle(double throttle) {
    throwIfClosed();
    // Only for 'get' API
    m_setpoint = throttle;
    setpointCommand(throttle, ControlType.kDutyCycle);
  }

  /**
   * Sets the voltage output of the SpeedController. This is equivillant to a call to
   * SetReference(output, rev::ControlType::kVoltage). The behavior of this call differs slightly
   * from the WPILib documetation for this call since the device internally sets the desired voltage
   * (not a compensation value). That means that this *can* be a 'set-and-forget' call.
   *
   * @param outputVolts The voltage to output.
   */
  @Override
  public void setVoltage(double outputVolts) {
    throwIfClosed();
    setpointCommand(outputVolts, ControlType.kVoltage);
  }

  /**
   * Gets the throttle of the motor controller.
   *
   * @return The throttle where -1 represents full reverse and 1 represents full forward.
   */
  @Override
  public double getThrottle() {
    throwIfClosed();
    return m_setpoint;
  }

  /**
   * Common interface for inverting direction of a speed controller.
   *
   * <p>This call has no effect if the controller is a follower. To invert a follower, see the
   * follow() method.
   *
   * @param isInverted The state of inversion, true is inverted.
   * @deprecated Use {@link SparkBaseConfig#inverted(boolean)} with {@link
   *     #configure(SparkBaseConfig, ResetMode, PersistMode)} instead.
   */
  @Deprecated
  @Override
  public void setInverted(boolean isInverted) {
    throwIfClosed();
    DriverStationErrors.reportWarning(
        "The inversion setting should be set via a SparkMaxConfig or a SparkFlexConfig object",
        true);
    CANSparkJNI.c_Spark_SetInverted(sparkHandle, isInverted);
  }

  /**
   * Common interface for returning the inversion state of a speed controller.
   *
   * <p>This call has no effect if the controller is a follower.
   *
   * @return isInverted The state of inversion, true is inverted.
   * @deprecated Use {@link com.revrobotics.spark.config.SparkBaseConfigAccessor#getInverted()
   *     SparkBaseConfigAccessor.getInverted()} via {@link SparkMax#configAccessor} or {@link
   *     SparkFlex#configAccessor} instead.
   */
  @Deprecated
  @Override
  public boolean getInverted() {
    throwIfClosed();
    DriverStationErrors.reportWarning(
        "The inversion setting should be retrieved via the configAccessor field of a SparkMax or SparkFlex object",
        true);
    return CANSparkJNI.c_Spark_GetInverted(sparkHandle);
  }

  /** Common interface for disabling a motor. */
  @Override
  public void disable() {
    throwIfClosed();
    setThrottle(0);
  }

  public void stopMotor() {
    throwIfClosed();
    setThrottle(0);
  }

  /** ***** Extended Functions ****** */

  /**
   * Set the configuration for the SPARK.
   *
   * <p>If {@code resetMode} is {@link com.revrobotics.ResetMode#kResetSafeParameters}, this method
   * will reset safe writable parameters to their default values before setting the given
   * configuration. The following parameters will not be reset by this action: CAN ID, Motor Type,
   * Idle Mode, PWM Input Deadband, and Duty Cycle Offset.
   *
   * <p>If {@code persistMode} is {@link com.revrobotics.PersistMode#kPersistParameters}, this
   * method will save all parameters to the SPARK's non-volatile memory after setting the given
   * configuration. This will allow parameters to persist across power cycles.
   *
   * @param config The desired SPARK configuration
   * @param resetMode Whether to reset safe parameters before setting the configuration
   * @param persistMode Whether to persist the parameters after setting the configuration
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError configure(
      SparkBaseConfig config, ResetMode resetMode, PersistMode persistMode) {
    throwIfClosed();
    REVLibError status =
        REVLibError.fromInt(
            CANSparkJNI.c_Spark_Configure(
                this.sparkHandle,
                config.flatten(),
                resetMode == ResetMode.kResetSafeParameters,
                persistMode == PersistMode.kPersistParameters));

    if (status != REVLibError.kOk) {
      // Check if fatal error
      if (status == REVLibError.kTimeout
          || status == REVLibError.kCannotPersistParametersWhileEnabled) {
        return status;
      }

      throw new IllegalStateException(REVLibJNI.c_REVLib_ErrorFromCode(status.value));
    }

    return status;
  }

  /**
   * Set the configuration for the SPARK without waiting for a response.
   *
   * <p>If {@code resetMode} is {@link com.revrobotics.ResetMode#kResetSafeParameters}, this method
   * will reset safe writable parameters to their default values before setting the given
   * configuration. The following parameters will not be reset by this action: CAN ID, Motor Type,
   * Idle Mode, PWM Input Deadband, and Duty Cycle Offset.
   *
   * <p>If {@code persistMode} is {@link com.revrobotics.PersistMode#kPersistParameters}, this
   * method will save all parameters to the SPARK's non-volatile memory after setting the given
   * configuration. This will allow parameters to persist across power cycles.
   *
   * <p>NOTE: This method will immediately return {@link REVLibError#kOk} and the action will be
   * done in the background. Any errors that occur will be reported to the driver station.
   *
   * @param config The desired SPARK configuration
   * @param resetMode Whether to reset safe parameters before setting the configuration
   * @param persistMode Whether to persist the parameters after setting the configuration
   * @return {@link REVLibError#kOk}
   */
  public REVLibError configureAsync(
      SparkBaseConfig config, ResetMode resetMode, PersistMode persistMode) {
    throwIfClosed();
    return REVLibError.fromInt(
        CANSparkJNI.c_Spark_ConfigureAsync(
            this.sparkHandle,
            config.flatten(),
            resetMode == ResetMode.kResetSafeParameters,
            persistMode == PersistMode.kPersistParameters));
  }

  /**
   * Returns an object for interfacing with the encoder connected to the encoder pins or front port
   * of the SPARK MAX or the motor interface of the SPARK Flex.
   *
   * @return An object for interfacing with an encoder
   */
  public RelativeEncoder getEncoder() {
    throwIfClosed();
    synchronized (encoderLock) {
      if (encoder == null) {
        encoder = new SparkRelativeEncoder(this);
      }
      return encoder;
    }
  }

  /**
   * Returns an object for interfacing with a connected analog sensor.
   *
   * @return An object for interfacing with a connected analog sensor.
   */
  public SparkAnalogSensor getAnalog() {
    throwIfClosed();
    synchronized (analogSensorLock) {
      if (analogSensor == null) {
        analogSensor = new SparkAnalogSensor(this);
      }
      return analogSensor;
    }
  }

  /**
   * Returns an object for interfacing with a connected absolute encoder.
   *
   * @return An object for interfacing with a connected absolute encoder
   */
  public SparkAbsoluteEncoder getAbsoluteEncoder() {
    throwIfClosed();
    synchronized (absoluteEncoderLock) {
      if (absoluteEncoder == null) {
        absoluteEncoder = new SparkAbsoluteEncoder(this);
      }
      return absoluteEncoder;
    }
  }

  /**
   * @return An object for interfacing with the integrated closed loop controller.
   */
  public SparkClosedLoopController getClosedLoopController() {
    throwIfClosed();
    synchronized (closedLoopControllerLock) {
      if (closedLoopController == null) {
        closedLoopController = new SparkClosedLoopController(this);
      }
      return closedLoopController;
    }
  }

  /**
   * Returns an object for interfacing with the forward limit switch connected to the appropriate
   * pins on the data port.
   *
   * @return An object for interfacing with the forward limit switch.
   */
  public SparkLimitSwitch getForwardLimitSwitch() {
    throwIfClosed();
    synchronized (forwardLimitSwitchLock) {
      if (forwardLimitSwitch == null) {
        forwardLimitSwitch = new SparkLimitSwitch(this, SparkLimitSwitch.Direction.kForward);
      }
      return forwardLimitSwitch;
    }
  }

  /**
   * Returns an object for interfacing with the reverse limit switch connected to the appropriate
   * pins on the data port.
   *
   * @return An object for interfacing with the reverse limit switch.
   */
  public SparkLimitSwitch getReverseLimitSwitch() {
    throwIfClosed();
    synchronized (reverseLimitSwitchLock) {
      if (reverseLimitSwitch == null) {
        reverseLimitSwitch = new SparkLimitSwitch(this, SparkLimitSwitch.Direction.kReverse);
      }
      return reverseLimitSwitch;
    }
  }

  /**
   * Returns an object for interfacing with the forward soft limit.
   *
   * @return An object for interfacing with the forward soft limit.
   */
  public SparkSoftLimit getForwardSoftLimit() {
    throwIfClosed();
    synchronized (forwardSoftLimitLock) {
      if (forwardSoftLimit == null) {
        forwardSoftLimit = new SparkSoftLimit(this, SparkSoftLimit.SoftLimitDirection.kForward);
      }
      return forwardSoftLimit;
    }
  }

  /**
   * Returns an object for interfacing with the reverse soft limit.
   *
   * @return An object for interfacing with the reverse soft limit.
   */
  public SparkSoftLimit getReverseSoftLimit() {
    throwIfClosed();
    synchronized (reverseSoftLimitLock) {
      if (reverseSoftLimit == null) {
        reverseSoftLimit = new SparkSoftLimit(this, SparkSoftLimit.SoftLimitDirection.kReverse);
      }
      return reverseSoftLimit;
    }
  }

  /** Get the Motor Interface type */
  // package-private
  int getMotorInterface() {
    throwIfClosed();
    return CANSparkJNI.c_Spark_GetMotorInterface(sparkHandle);
  }

  /**
   * Resume follower mode if the SPARK has a valid follower mode config.
   *
   * <p>NOTE: Follower mode will start automatically upon configuring follower mode. If the SPARK
   * experiences a power cycle and has follower mode configured, follower mode will automatically
   * restart. This method is only useful after {@link #pauseFollowerMode()} has been called.
   *
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError resumeFollowerMode() {
    throwIfClosed();
    return REVLibError.fromInt(CANSparkJNI.c_Spark_StartFollowerMode(sparkHandle));
  }

  /**
   * Resume follower mode if the SPARK has a valid follower mode config without waiting for a
   * response.
   *
   * <p>NOTE: Follower mode will start automatically upon configuring follower mode. If the SPARK
   * experiences a power cycle and has follower mode configured, follower mode will automatically
   * restart. This method is only useful after {@link #pauseFollowerMode()} has been called.
   *
   * <p>NOTE: This method will immediately return {@link REVLibError#kOk} and the action will be
   * done in the background. Any errors that occur will be reported to the driver station.
   *
   * @return {@link REVLibError#kOk}
   * @see #resumeFollowerMode()
   */
  public REVLibError resumeFollowerModeAsync() {
    throwIfClosed();
    return REVLibError.fromInt(CANSparkJNI.c_Spark_StartFollowerModeAsync(sparkHandle));
  }

  /**
   * Pause follower mode.
   *
   * <p>NOTE: If the SPARK experiences a power cycle and has follower mode configured, follower mode
   * will automatically restart.
   *
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError pauseFollowerMode() {
    throwIfClosed();
    return REVLibError.fromInt(CANSparkJNI.c_Spark_StopFollowerMode(sparkHandle));
  }

  /**
   * Pause follower mode without waiting for a response
   *
   * <p>NOTE: If the SPARK experiences a power cycle and has follower mode configured, follower mode
   * will automatically restart.
   *
   * <p>NOTE: This method will immediately return {@link REVLibError#kOk} and the action will be
   * done in the background. Any errors that occur will be reported to the driver station.
   *
   * @return {@link REVLibError#kOk}
   * @see #pauseFollowerMode()
   */
  public REVLibError pauseFollowerModeAsync() {
    throwIfClosed();
    return REVLibError.fromInt(CANSparkJNI.c_Spark_StopFollowerModeAsync(sparkHandle));
  }

  /**
   * Returns whether the controller is following another controller
   *
   * @return True if this device is following another controller false otherwise
   */
  public Signal<Boolean> isFollower() {
    throwIfClosed();
    var status = getPeriodicStatus1();
    return status.map(PeriodicStatus1::isFollower);
  }

  /**
   * Get whether the SPARK has one or more active faults.
   *
   * @return true if there is an active fault
   * @see #getFaults()
   */
  public Signal<Boolean> hasActiveFault() {
    throwIfClosed();
    return getFaults().map(status -> status.rawBits != 0);
  }

  /**
   * Get whether the SPARK has one or more sticky faults.
   *
   * @return true if there is a sticky fault
   * @see #getStickyFaults()
   */
  public Signal<Boolean> hasStickyFault() {
    throwIfClosed();
    return getStickyFaults().map(status -> status.rawBits != 0);
  }

  /**
   * Get whether the SPARK has one or more active warnings.
   *
   * @return true if there is an active warning
   * @see #getWarnings()
   */
  public Signal<Boolean> hasActiveWarning() {
    throwIfClosed();
    return getWarnings().map(status -> status.rawBits != 0);
  }

  /**
   * Get whether the SPARK has one or more sticky warnings.
   *
   * @return true if there is a sticky warning
   * @see #getStickyWarnings()
   */
  public Signal<Boolean> hasStickyWarning() {
    throwIfClosed();
    return getStickyWarnings().map(status -> status.rawBits != 0);
  }

  /**
   * Get the active faults that are currently present on the SPARK. Faults are fatal errors that
   * prevent the motor from running.
   *
   * @return A struct with each fault and their active value
   */
  public Signal<Faults> getFaults() {
    throwIfClosed();
    var status = getPeriodicStatus1();
    return status.map(
        (statusFrame) ->
            new Faults(
                statusFrame.otherFault(),
                statusFrame.motorTypeFault(),
                statusFrame.sensorFault(),
                statusFrame.canFault(),
                statusFrame.temperatureFault(),
                statusFrame.drvFault(),
                statusFrame.escEepromFault(),
                statusFrame.firmwareFault()));
  }

  /**
   * Get the sticky faults that were present on the SPARK at one point since the sticky faults were
   * last cleared. Faults are fatal errors that prevent the motor from running.
   *
   * <p>Sticky faults can be cleared with {@link SparkBase#clearFaults()}.
   *
   * @return A struct with each fault and their sticky value
   */
  public Signal<Faults> getStickyFaults() {
    throwIfClosed();
    var status = getPeriodicStatus1();
    return status.map(
        (statusFrame) ->
            new Faults(
                statusFrame.otherStickyFault(),
                statusFrame.motorTypeStickyFault(),
                statusFrame.sensorStickyFault(),
                statusFrame.canStickyFault(),
                statusFrame.temperatureStickyFault(),
                statusFrame.drvStickyFault(),
                statusFrame.escEepromStickyFault(),
                statusFrame.firmwareStickyFault()));
  }

  /**
   * Get the active warnings that are currently present on the SPARK. Warnings are non-fatal errors.
   *
   * @return A struct with each warning and their active value
   */
  public Signal<Warnings> getWarnings() {
    throwIfClosed();
    var status = getPeriodicStatus1();
    return status.map(
        statusFrame ->
            new Warnings(
                statusFrame.brownoutWarning(),
                statusFrame.overcurrentWarning(),
                statusFrame.escEepromWarning(),
                statusFrame.extEepromWarning(),
                statusFrame.sensorWarning(),
                statusFrame.stallWarning(),
                statusFrame.hasResetWarning(),
                statusFrame.otherWarning()));
  }

  /**
   * Get the sticky warnings that were present on the SPARK at one point since the sticky warnings
   * were last cleared. Warnings are non-fatal errors.
   *
   * <p>Sticky warnings can be cleared with {@link SparkBase#clearFaults()}.
   *
   * @return A struct with each warning and their sticky value
   */
  public Signal<Warnings> getStickyWarnings() {
    throwIfClosed();
    var status = getPeriodicStatus1();
    return status.map(
        statusFrame ->
            new Warnings(
                statusFrame.brownoutStickyWarning(),
                statusFrame.overcurrentStickyWarning(),
                statusFrame.escEepromStickyWarning(),
                statusFrame.extEepromStickyWarning(),
                statusFrame.sensorStickyWarning(),
                statusFrame.stallStickyWarning(),
                statusFrame.hasResetStickyWarning(),
                statusFrame.otherStickyWarning()));
  }

  /**
   * @return The voltage fed into the motor controller.
   */
  public Signal<Double> getBusVoltage() {
    throwIfClosed();
    var status = getPeriodicStatus0();
    return status.map(PeriodicStatus0::voltage);
  }

  /**
   * Simulation note: this value will not be updated during simulation unless {@link
   * SparkSim#iterate} is called
   *
   * @return The motor controller's applied output duty cycle.
   */
  public Signal<Double> getAppliedOutput() {
    throwIfClosed();
    var status = getPeriodicStatus0();
    return status.map(PeriodicStatus0::appliedOutput);
  }

  /**
   * @return The motor controller's output current in Amps.
   */
  public Signal<Double> getOutputCurrent() {
    throwIfClosed();
    var status = getPeriodicStatus0();
    return status.map(PeriodicStatus0::current);
  }

  /**
   * @return The motor temperature in Celsius.
   */
  public Signal<Double> getMotorTemperature() {
    throwIfClosed();
    var status = getPeriodicStatus0();
    return status.map(statusFrame -> (double) statusFrame.motorTemperature());
  }

  /**
   * Clears all sticky faults.
   *
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError clearFaults() {
    throwIfClosed();
    return REVLibError.fromInt(CANSparkJNI.c_Spark_ClearFaults(sparkHandle));
  }

  /**
   * Sets the timeout duration for waiting for CAN responses from the device.
   *
   * @param milliseconds The timeout in milliseconds.
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError setCANTimeout(int milliseconds) {
    throwIfClosed();
    return REVLibError.fromInt(CANSparkJNI.c_Spark_SetCANTimeout(sparkHandle, milliseconds));
  }

  /**
   * All device errors are tracked on a per thread basis for all devices in that thread. This is
   * meant to be called immediately following another call that has the possibility of returning an
   * error to validate if an error has occurred.
   *
   * @return the last error that was generated.
   */
  public REVLibError getLastError() {
    throwIfClosed();
    return REVLibError.fromInt(CANSparkJNI.c_Spark_GetLastError(sparkHandle));
  }

  SparkModel getSparkModel() {
    throwIfClosed();
    int model = CANSparkJNI.c_Spark_GetSparkModel(sparkHandle);

    return SparkModel.fromId(model);
  }

  // package-private
  enum DataPortConfig {
    kNone(-1, "none"),
    kLimitSwitchesAndAbsoluteEncoder(0, "limit switch and/or absolute encoder"),
    kAltEncoder(1, "alternate encoder");

    final int m_value;
    final String m_name;

    DataPortConfig(int value, String name) {
      m_value = value;
      m_name = name;
    }

    public static DataPortConfig fromInt(int id) {
      for (DataPortConfig type : values()) {
        if (type.m_value == id) {
          return type;
        }
      }
      return kNone;
    }
  }
}
