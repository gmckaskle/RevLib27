/*
 * Copyright (c) 2026 REV Robotics
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

import com.revrobotics.MutableInt;
import com.revrobotics.REVLibError;
import com.revrobotics.StatusFrame;
import com.revrobotics.jni.CANA301JNI;
import com.revrobotics.util.Signal;
import java.nio.ByteBuffer;
import java.util.concurrent.atomic.AtomicBoolean;
import org.wpilib.hardware.motor.MotorController;

public class A301 implements MotorController {
  public enum PeriodicFrame {
    kStatus0(0),
    kStatus1(1),
    kStatus2(2),
    kStatus3(3);

    @SuppressWarnings("MemberName")
    public final int value;

    PeriodicFrame(int value) {
      this.value = value;
    }

    public static PeriodicFrame fromId(int id) {
      return switch (id) {
        case 0 -> kStatus0;
        case 1 -> kStatus1;
        case 2 -> kStatus2;
        case 3 -> kStatus3;
        default -> throw new IllegalArgumentException("Unknown PeriodicFrame id: " + id);
      };
    }
  }

  public enum GearboxRPM {
    kUnknown(0),
    k215(1),
    k500(2);

    @SuppressWarnings("MemberName")
    public final int value;

    GearboxRPM(int value) {
      this.value = value;
    }

    public static GearboxRPM fromId(int id) {
      return switch (id) {
        case 0 -> kUnknown;
        case 1 -> k215;
        case 2 -> k500;
        default -> throw new IllegalArgumentException("Unknown GearboxRPM id: " + id);
      };
    }
  }

  public record PeriodicStatus0(
      double appliedOutput,
      double voltage,
      double current,
      int motorTemperature,
      boolean inverted,
      boolean relativeHeartbeatLock,
      int gearboxRPM,
      int revlibError,
      long timestamp)
      implements StatusFrame {}

  public record PeriodicStatus1(
      boolean otherFault,
      boolean motorTypeFault,
      boolean sensorFault,
      boolean canFault,
      boolean temperatureFault,
      boolean drvFault,
      boolean escEepromFault,
      boolean firmwareFault,
      boolean brownoutWarning,
      boolean overcurrentWarning,
      boolean escEepromWarning,
      boolean extEepromWarning,
      boolean sensorWarning,
      boolean stallWarning,
      boolean hasResetWarning,
      boolean otherWarning,
      boolean otherStickyFault,
      boolean motorTypeStickyFault,
      boolean sensorStickyFault,
      boolean canStickyFault,
      boolean temperatureStickyFault,
      boolean drvStickyFault,
      boolean escEepromStickyFault,
      boolean firmwareStickyFault,
      boolean brownoutStickyWarning,
      boolean overcurrentStickyWarning,
      boolean escEepromStickyWarning,
      boolean extEepromStickyWarning,
      boolean sensorStickyWarning,
      boolean stallStickyWarning,
      boolean hasResetStickyWarning,
      boolean otherStickyWarning,
      boolean isFollower,
      int revlibError,
      long timestamp)
      implements StatusFrame {}

  public record PeriodicStatus2(
      double relativeEncoderVelocity,
      double relativeEncoderPosition,
      int revlibError,
      long timestamp)
      implements StatusFrame {}

  public record PeriodicStatus3(double absoluteEncoderPosition, int revlibError, long timestamp)
      implements StatusFrame {}

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

  private final long a301Handle;
  protected final AtomicBoolean isClosed = new AtomicBoolean(false);
  private final int busId;
  private final int deviceId;
  private String firmwareString;

  // Only used for MotorController get/setDutyCycle() API
  private double m_setpoint = 0.0;

  // The default A301 device ID is set to 3 out of the factory.
  public static final int defaultDeviceId = 3;

  /**
   * Create a new object to control a FIRST A301 motor
   *
   * <p>NOTE: If busId indicates a Motioncore bus, the device ID will be auto-detected, otherwise,
   * the defaultDeviceId is used.
   *
   * @param busId The CAN bus ID this device will be on.
   */
  public A301(int busId) {
    this(busId, defaultDeviceId);
  }

  /**
   * Create a new object to control a FIRST A301 motor
   *
   * <p>NOTE: If busId indicates a Motioncore bus, the device ID will be auto-detected and deviceId
   * will be ignored.
   *
   * @param busId The CAN bus ID this device will be on.
   * @param deviceId The device ID.
   */
  public A301(int busId, int deviceId) {
    this.busId = busId;
    firmwareString = "";

    MutableInt actualDeviceId = new MutableInt(0);
    if (CANA301JNI.c_A301_RegisterId(busId, deviceId, actualDeviceId)
        == REVLibError.kDuplicateCANId.value) {
      String device;
      if (deviceId != actualDeviceId.value) {
        device = "auto-detected device ID: " + actualDeviceId.value;
      } else {
        device = " deviceID: " + deviceId;
      }

      throw new IllegalStateException(
          "A FIRST A301 instance has already been created with this "
              + device
              + " on Bus: "
              + busId);
    }
    this.deviceId = actualDeviceId.value;

    MutableInt status = new MutableInt(0);
    a301Handle = CANA301JNI.c_A301_Create(busId, deviceId, status);

    if (REVLibError.fromInt(status.value) != REVLibError.kOk) {
      switch (REVLibError.fromInt(status.value)) {
        case kCantFindFirmware:
          // Don't throw exception when no firmware is found. It's
          // possible the device is disconnected and we don't want to stop
          // the program if that is the case.
          break;
        case kFirmwareTooOld:
          throw new IllegalStateException(
              "The firmware version of Bus #"
                  + busId
                  + " A301 #"
                  + deviceId
                  + " is too old and must be updated to 27.0.0-prerelease-11 or later.");
        case kFirmwareTooNew:
          throw new IllegalStateException(
              "The firmware version of Bus #"
                  + busId
                  + " A301 #"
                  + deviceId
                  + " is too new for this version of REVLib");
        default:
          throw new IllegalStateException(
              "Error (" + status.value + ") creating Bus #" + busId + " FIRST A301 #" + deviceId);
      }
    }
  }

  /** Closes the FIRST A301 motor controller */
  public void close() {
    boolean wasClosed = isClosed.getAndSet(true);
    if (wasClosed) {
      return;
    }
    CANA301JNI.c_A301_Close(a301Handle);
    CANA301JNI.c_A301_Destroy(a301Handle);
  }

  /**
   * Get the configured CAN Bus ID of the FIRST A301.
   *
   * @return int CAN bus ID
   */
  public int getBusId() {
    throwIfClosed();
    return busId;
  }

  /**
   * Get the configured Device ID of the FIRST A301.
   *
   * @return int device ID
   */
  public int getDeviceId() {
    throwIfClosed();
    return deviceId;
  }

  /**
   * Get the firmware version of the FIRST A301.
   *
   * @return uint32_t Firmware version integer. Value is represented as 4 bytes, Major.Minor.Build
   *     H.Build L
   */
  public int getFirmwareVersion() {
    throwIfClosed();
    return CANA301JNI.c_A301_GetFirmwareVersion(a301Handle);
  }

  /**
   * Get the firmware version of the FIRST A301 as a string.
   *
   * @return String Human readable firmware version string
   */
  public String getFirmwareString() {
    throwIfClosed();
    if (firmwareString == "") {
      int version = getFirmwareVersion();
      ByteBuffer b = ByteBuffer.allocate(4);
      b.putInt(version);

      byte[] verBytes = b.array();

      StringBuilder firmwareString = new StringBuilder();
      firmwareString
          .append("v")
          .append((int) verBytes[0])
          .append(".")
          .append((int) verBytes[1])
          .append(".")
          .append((int) verBytes[2] << 8 | (int) verBytes[3]);

      this.firmwareString = firmwareString.toString();
    }
    return firmwareString;
  }

  /**
   * Get whether the A301 has one or more active faults.
   *
   * @return true if there is an active fault
   * @see #getFaults()
   */
  public Signal<Boolean> hasActiveFault() {
    throwIfClosed();
    return getFaults().map(status -> status.rawBits != 0);
  }

  /**
   * Get whether the A301 has one or more sticky faults.
   *
   * @return true if there is a sticky fault
   * @see #getStickyFaults()
   */
  public Signal<Boolean> hasStickyFault() {
    throwIfClosed();
    return getStickyFaults().map(status -> status.rawBits != 0);
  }

  /**
   * Get whether the A301 has one or more active warnings.
   *
   * @return true if there is an active warning
   * @see #getWarnings()
   */
  public Signal<Boolean> hasActiveWarning() {
    throwIfClosed();
    return getWarnings().map(status -> status.rawBits != 0);
  }

  /**
   * Get whether the A301 has one or more sticky warnings.
   *
   * @return true if there is a sticky warning
   * @see #getStickyWarnings()
   */
  public Signal<Boolean> hasStickyWarning() {
    throwIfClosed();
    return getStickyWarnings().map(status -> status.rawBits != 0);
  }

  /**
   * Get the active faults that are currently present on the A301. Faults are fatal errors that
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
   * Get the sticky faults that were present on the A301 at one point since the sticky faults were
   * last cleared. Faults are fatal errors that prevent the motor from running.
   *
   * <p>Sticky faults can be cleared with {@link A301#clearFaults()}.
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
   * Get the active warnings that are currently present on the A301. Warnings are non-fatal errors.
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
   * Get the sticky warnings that were present on the A301 at one point since the sticky warnings
   * were last cleared. Warnings are non-fatal errors.
   *
   * <p>Sticky warnings can be cleared with {@link A301#clearFaults()}.
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
   * Clears all sticky faults.
   *
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError clearFaults() {
    throwIfClosed();
    return REVLibError.fromInt(CANA301JNI.c_A301_ClearFaults(a301Handle));
  }

  /**
   * Returns the A301 bus voltage in Volts.
   *
   * @return The voltage fed into the motor controller.
   */
  public Signal<Double> getBusVoltage() {
    throwIfClosed();
    var status = getPeriodicStatus0();
    return status.map(PeriodicStatus0::voltage);
  }

  /**
   * Returns the A301's output duty cycle.
   *
   * @return The motor controller's applied output duty cycle.
   */
  public Signal<Double> getAppliedOutput() {
    throwIfClosed();
    var status = getPeriodicStatus0();
    return status.map(PeriodicStatus0::appliedOutput);
  }

  /**
   * Returns A301's motor current in Amps.
   *
   * @return The motor's current in Amps.
   */
  public Signal<Double> getMotorCurrent() {
    throwIfClosed();
    var status = getPeriodicStatus0();
    return status.map(PeriodicStatus0::current);
  }

  /**
   * Returns the motor temperature in Celsius.
   *
   * @return The motor temperature in Celsius.
   */
  public Signal<Double> getMotorTemperature() {
    throwIfClosed();
    var status = getPeriodicStatus0();
    return status.map(statusFrame -> (double) statusFrame.motorTemperature());
  }

  /**
   * Returns the RPM of the A301's attached gearbox.
   *
   * @return Signal containing the gearbox RPM (@see GearboxRPM)
   */
  public Signal<GearboxRPM> getGearboxRPM() {
    throwIfClosed();
    var status = getPeriodicStatus0();
    return status.map(statusFrame -> GearboxRPM.fromId(statusFrame.gearboxRPM()));
  }

  /**
   * Get the position of the motor. This returns the native units of 'rotations'.
   *
   * @return Signal containing the number of rotations of the motor
   */
  public Signal<Double> getRelativeEncoderPosition() {
    throwIfClosed();
    var status = getPeriodicStatus2();
    return status.map(A301.PeriodicStatus2::relativeEncoderPosition);
  }

  /**
   * Get the velocity of the motor. This returns the native units of 'RPM'.
   *
   * @return Signal containing the RPM of the motor
   */
  public Signal<Double> getEncoderVelocity() {
    throwIfClosed();
    var status = getPeriodicStatus2();
    return status.map(A301.PeriodicStatus2::relativeEncoderVelocity);
  }

  public Signal<Double> getAbsoluteEncoderPosition() {
    throwIfClosed();
    var status = getPeriodicStatus3();
    return status.map(A301.PeriodicStatus3::absoluteEncoderPosition);
  }

  /**
   * Set the position of the relative encoder.
   *
   * @param position Number of rotations of the motor
   * @return REVLibError::kOk if successful
   */
  public REVLibError setRelativeEncoderPosition(double position) {
    throwIfClosed();
    return REVLibError.fromInt(
        CANA301JNI.c_A301_SetRelativeEncoderPosition(a301Handle, (float) position));
  }

  /**
   * Set the position of the absolute encoder.
   *
   * @param position Number of rotations of the motor
   * @return REVLibError::kOk if successful
   */
  public REVLibError setAbsoluteEncoderPosition(double position) {
    throwIfClosed();
    return REVLibError.fromInt(
        CANA301JNI.c_A301_SetAbsoluteEncoderPosition(a301Handle, (float) position));
  }

  /**
   * Sets the velocity of the A301.
   *
   * @param velocity The velocity (in RPM) to set
   * @return REVLibError.kOk if successful
   */
  public REVLibError setVelocity(double velocity) {
    return setSetpoint(ControlType.kVelocity, velocity);
  }

  /**
   * Sets the relative position of the A301 (with maximum speed).
   *
   * @param position The relative position (in Rotations) to set
   * @return REVLibError.kOk if successful
   */
  public REVLibError setRelativePosition(double position) {
    return setSetpoint(ControlType.kRelativePosition, position);
  }

  /**
   * Sets the relative position of the A301 with a specific speed.
   *
   * <p>Setting speed less than or equal to 0 will apply maximum speed)
   *
   * @param position The relative position (in Rotations) to set
   * @param speed The speed to approach the position at (in RPM)
   * @return REVLibError.kOk if successful
   */
  public REVLibError setRelativePositionWithSpeed(double position, double speed) {
    return setSetpoint(ControlType.kRelativePosition, position, speed);
  }

  /**
   * Sets the absolute position of the A301 (with maximum speed).
   *
   * @param absPosition The absolute position (-0.5 to 0.5) to set
   * @return REVLibError.kOk if successful
   */
  public REVLibError setAbsolutePosition(double absPosition) {
    return setSetpoint(ControlType.kAbsolutePosition, absPosition);
  }

  /**
   * Sets the absolute position of the A301 with a specific speed.
   *
   * <p>Setting speed less than or equal to 0 will apply maximum speed)
   *
   * @param absPosition The absolute position (-0.5 to 0.5) to set
   * @param speed The speed to approach the position at (in RPM)
   * @return REVLibError.kOk if successful
   */
  public REVLibError setAbsolutePositionWithSpeed(double absPosition, double speed) {
    return setSetpoint(ControlType.kAbsolutePosition, absPosition, speed);
  }

  public REVLibError setIdleMode(IdleMode idleMode) {
    throwIfClosed();
    return REVLibError.fromInt(CANA301JNI.c_A301_SetIdleMode(a301Handle, idleMode.value));
  }

  public int getIdleMode() {
    return CANA301JNI.c_A301_GetIdleMode(a301Handle);
  }

  /**
   * Enables continuous input for absolute position control.
   *
   * @return REVLibError::kOk if successful
   */
  public REVLibError enableAbsolutePositionContinuousInput() {
    throwIfClosed();
    return REVLibError.fromInt(
        CANA301JNI.c_A301_SetAbsolutePositionContinuousInput(a301Handle, true));
  }

  /**
   * Disables continuous input for absolute position control.
   *
   * @return REVLibError::kOk if successful
   */
  public REVLibError disableAbsolutePositionContinuousInput() {
    throwIfClosed();
    return REVLibError.fromInt(
        CANA301JNI.c_A301_SetAbsolutePositionContinuousInput(a301Handle, false));
  }

  /**
   * Queries the continuous input for absolute position control.
   *
   * @return true if continuous input is enabled.
   */
  public boolean isAbsolutePositionContinuousInputEnabled() {
    throwIfClosed();
    return CANA301JNI.c_A301_GetAbsolutePositionContinuousInput(a301Handle);
  }

  /**
   * Set the offset of the range about zero of the absolute encoder. Moves the range between (-1,
   * 0], and [0, 1), instead of the default range [-0.5, 0.5), assuming the default units of
   * rotations.
   *
   * @param offset Fraction of a rotation [-0.5, 0.5] to shift from 0
   * @return REVLibError::kOk if successful
   */
  public REVLibError setAbsoluteEncoderRangeOffset(double offset) {
    throwIfClosed();
    return REVLibError.fromInt(
        CANA301JNI.c_A301_SetAbsoluteEncoderRangeOffset(a301Handle, (float) offset));
  }

  /**
   * Get the offset of the range about zero of the absolute encoder. This returns the native units
   * of 'rotations'.
   *
   * @return The range offset (in rotations) of the absolute encoder
   */
  public double getAbsoluteEncoderRangeOffset() {
    throwIfClosed();
    return CANA301JNI.c_A301_GetAbsoluteEncoderRangeOffset(a301Handle);
  }

  /**
   * Sets the motor current of the A301.
   *
   * @param current The current (in Amps) to drive the motor at
   * @return REVLibError.kOk if successful
   */
  public REVLibError setCurrent(double current) {
    return setSetpoint(ControlType.kCurrent, current);
  }

  /** MotorController Interface */

  /**
   * Sets the throttle of the motor controller.
   *
   * @param throttle The throttle where -1.0 indicates full reverse and 1.0 indicates full forward.
   */
  @Override
  public void setThrottle(double throttle) {
    throwIfClosed();
    // Only for 'get' API
    m_setpoint = throttle;
    setSetpoint(ControlType.kDutyCycle, throttle);
  }

  /**
   * Gets the throttle of the motor controller.
   *
   * @return The throttle where -1.0 represents full reverse and 1.0 represents full forward.
   */
  @Override
  public double getThrottle() {
    throwIfClosed();
    return m_setpoint;
  }

  /**
   * Sets the voltage output of the MotorController. This is equivalent to a call to
   * setSetpoint(output, ControlType.kVoltage). The behavior of this call differs slightly from the
   * WPILib documetation for this call since the device internally sets the desired voltage (not a
   * compensation value). That means that this *can* be a 'set-and-forget' call.
   *
   * @param outputVolts The voltage to output.
   */
  @Override
  public void setVoltage(double outputVolts) {
    throwIfClosed();
    m_setpoint = outputVolts / 12.0;
    setSetpoint(ControlType.kVoltage, outputVolts);
  }

  /**
   * Common interface for inverting direction of a speed controller.
   *
   * @param isInverted The state of inversion, true is inverted.
   */
  @Override
  public void setInverted(boolean isInverted) {
    throwIfClosed();
    CANA301JNI.c_A301_SetInverted(a301Handle, isInverted);
  }

  /**
   * Common interface for returning the inversion state of a speed controller.
   *
   * @return The state of inversion, true is inverted.
   */
  @Override
  public boolean getInverted() {
    throwIfClosed();
    return CANA301JNI.c_A301_GetInverted(a301Handle);
  }

  /** Common interface for disabling a motor. */
  @Override
  public void disable() {
    throwIfClosed();
    setSetpoint(ControlType.kDutyCycle, 0.0);
  }

  /**** Signals Configuration Interface ****/

  /**
   * Set the period (ms) of the status frame that provides the signal returned by getFaults() and
   * getStickyFaults().
   *
   * <p>The default period is 50ms. The maximum period is 1000ms (1s).
   *
   * <p>If multiple periods are set for signals within the same status frame, the minimum given
   * value will be used.
   *
   * @param period_ms The period in milliseconds
   * @return The modified A301 object for method chaining
   */
  public A301 faultsPeriodMs(int period_ms) {
    throwIfClosed();
    CANA301JNI.c_A301_SetStatusFramePeriod(a301Handle, PeriodicFrame.kStatus1.value, period_ms);
    return this;
  }

  /**
   * Get the period (ms) of the status frame that provides the signal returned by getFaults() and
   * getStickyFaults().
   *
   * @return The period in milliseconds
   */
  public int getFaultsPeriodMs() {
    throwIfClosed();
    return CANA301JNI.c_A301_GetStatusFramePeriod(a301Handle, PeriodicFrame.kStatus1.value);
  }

  /**
   * Set the period (ms) of the status frame that provides the signal returned by getWarnings() and
   * getStickyWarnings().
   *
   * <p>The default period is 50ms. The maximum period is 1000ms (1s).
   *
   * <p>If multiple periods are set for signals within the same status frame, the minimum given
   * value will be used.
   *
   * @param period_ms The period in milliseconds
   * @return The modified A301 object for method chaining
   */
  public A301 warningsPeriodMs(int period_ms) {
    throwIfClosed();
    CANA301JNI.c_A301_SetStatusFramePeriod(a301Handle, PeriodicFrame.kStatus1.value, period_ms);
    return this;
  }

  /**
   * Get the period (ms) of the status frame that provides the signal returned by getWarnings() and
   * getStickyWarnings().
   *
   * @return The period in milliseconds
   */
  public int getWarningsPeriodMs() {
    throwIfClosed();
    return CANA301JNI.c_A301_GetStatusFramePeriod(a301Handle, PeriodicFrame.kStatus1.value);
  }

  /**
   * Set the period (ms) of the status frame that provides the signal returned by getBusVoltage().
   *
   * <p>The default period is 10ms. The maximum period is 1000ms (1s).
   *
   * <p>If multiple periods are set for signals within the same status frame, the minimum given
   * value will be used.
   *
   * @param period_ms The period in milliseconds
   * @return The modified A301 object for method chaining
   */
  public A301 busVoltagePeriodMs(int period_ms) {
    throwIfClosed();
    CANA301JNI.c_A301_SetStatusFramePeriod(a301Handle, PeriodicFrame.kStatus0.value, period_ms);
    return this;
  }

  /**
   * Get the period (ms) of the status frame that provides the signal returned by getBusVoltage().
   *
   * @return The period in milliseconds
   */
  public int getBusVoltagePeriodMs() {
    throwIfClosed();
    return CANA301JNI.c_A301_GetStatusFramePeriod(a301Handle, PeriodicFrame.kStatus0.value);
  }

  /**
   * Set the period (ms) of the status frame that provides the signal returned by
   * getAppliedOutput().
   *
   * <p>The default period is 10ms. The maximum period is 1000ms (1s).
   *
   * <p>If multiple periods are set for signals within the same status frame, the minimum given
   * value will be used.
   *
   * @param period_ms The period in milliseconds
   * @return The modified A301 object for method chaining
   */
  public A301 appliedOutputPeriodMs(int period_ms) {
    throwIfClosed();
    CANA301JNI.c_A301_SetStatusFramePeriod(a301Handle, PeriodicFrame.kStatus0.value, period_ms);
    return this;
  }

  /**
   * Get the period (ms) of the status frame that provides the signal returned by
   * getAppliedOutput().
   *
   * @return The period in milliseconds
   */
  public int getAppliedOutputPeriodMs() {
    throwIfClosed();
    return CANA301JNI.c_A301_GetStatusFramePeriod(a301Handle, PeriodicFrame.kStatus0.value);
  }

  /**
   * Set the period (ms) of the status frame that provides the signal returned by getMotorCurrent().
   *
   * <p>The default period is 10ms. The maximum period is 1000ms (1s).
   *
   * <p>If multiple periods are set for signals within the same status frame, the minimum given
   * value will be used.
   *
   * @param period_ms The period in milliseconds
   * @return The modified A301 object for method chaining
   */
  public A301 motorCurrentPeriodMs(int period_ms) {
    throwIfClosed();
    CANA301JNI.c_A301_SetStatusFramePeriod(a301Handle, PeriodicFrame.kStatus0.value, period_ms);
    return this;
  }

  /**
   * Get the period (ms) of the status frame that provides the signal returned by getMotorCurrent().
   *
   * @return The period in milliseconds
   */
  public int getMotorCurrentPeriodMs() {
    throwIfClosed();
    return CANA301JNI.c_A301_GetStatusFramePeriod(a301Handle, PeriodicFrame.kStatus0.value);
  }

  /**
   * Set the period (ms) of the status frame that provides the signal returned by
   * getMotorTemperature().
   *
   * <p>The default period is 10ms. The maximum period is 1000ms (1s).
   *
   * <p>If multiple periods are set for signals within the same status frame, the minimum given
   * value will be used.
   *
   * @param period_ms The period in milliseconds
   * @return The modified A301 object for method chaining
   */
  public A301 motorTemperaturePeriodMs(int period_ms) {
    throwIfClosed();
    CANA301JNI.c_A301_SetStatusFramePeriod(a301Handle, PeriodicFrame.kStatus0.value, period_ms);
    return this;
  }

  /**
   * Get the period (ms) of the status frame that provides the signal returned by
   * getMotorTemperature().
   *
   * @return The period in milliseconds
   */
  public int getMotorTemperaturePeriodMs() {
    throwIfClosed();
    return CANA301JNI.c_A301_GetStatusFramePeriod(a301Handle, PeriodicFrame.kStatus0.value);
  }

  /**
   * Set the period (ms) of the status frame that provides the signal returned by
   * getRelativeEncoderPosition().
   *
   * <p>The default period is 20ms. The maximum period is 1000ms (1s).
   *
   * <p>If multiple periods are set for signals within the same status frame, the minimum given
   * value will be used.
   *
   * @param period_ms The period in milliseconds
   * @return The modified A301 object for method chaining
   */
  public A301 relativeEncoderPositionPeriodMs(int period_ms) {
    throwIfClosed();
    CANA301JNI.c_A301_SetStatusFramePeriod(a301Handle, PeriodicFrame.kStatus2.value, period_ms);
    return this;
  }

  /**
   * Get the period (ms) of the status frame that provides the signal returned by
   * getRelativeEncoderPosition().
   *
   * @return The period in milliseconds
   */
  public int getRelativeEncoderPositionPeriodMs() {
    throwIfClosed();
    return CANA301JNI.c_A301_GetStatusFramePeriod(a301Handle, PeriodicFrame.kStatus2.value);
  }

  /**
   * Set the period (ms) of the status frame that provides the signal returned by
   * getEncoderVelocity().
   *
   * <p>The default period is 20ms. The maximum period is 1000ms (1s).
   *
   * <p>If multiple periods are set for signals within the same status frame, the minimum given
   * value will be used.
   *
   * @param period_ms The period in milliseconds
   * @return The modified A301 object for method chaining
   */
  public A301 encoderVelocityPeriodMs(int period_ms) {
    throwIfClosed();
    CANA301JNI.c_A301_SetStatusFramePeriod(a301Handle, PeriodicFrame.kStatus2.value, period_ms);
    return this;
  }

  /**
   * Get the period (ms) of the status frame that provides the signal returned by
   * getEncoderVelocity().
   *
   * @return The period in milliseconds
   */
  public int getEncoderVelocityPeriodMs() {
    throwIfClosed();
    return CANA301JNI.c_A301_GetStatusFramePeriod(a301Handle, PeriodicFrame.kStatus2.value);
  }

  /**
   * Set the period (ms) of the status frame that provides the signal returned by
   * getAbsoluteEncoderPosition().
   *
   * <p>The default period is 20ms. The maximum period is 1000ms (1s).
   *
   * <p>If multiple periods are set for signals within the same status frame, the minimum given
   * value will be used.
   *
   * @param period_ms The period in milliseconds
   * @return The modified A301 object for method chaining
   */
  public A301 absoluteEncoderPositionPeriodMs(int period_ms) {
    throwIfClosed();
    CANA301JNI.c_A301_SetStatusFramePeriod(a301Handle, PeriodicFrame.kStatus3.value, period_ms);
    return this;
  }

  /**
   * Get the period (ms) of the status frame that provides the signal returned by
   * getAbsoluteEncoderPosition().
   *
   * @return The period in milliseconds
   */
  public int getAbsoluteEncoderPositionPeriodMs() {
    throwIfClosed();
    return CANA301JNI.c_A301_GetStatusFramePeriod(a301Handle, PeriodicFrame.kStatus3.value);
  }

  /**
   * Get Periodic Status 0 for the FIRST A301.
   *
   * @return periodic status 0, or null if it's not found
   */
  private Signal<PeriodicStatus0> getPeriodicStatus0() {
    throwIfClosed();
    return Signal.create(CANA301JNI.c_A301_GetPeriodicStatus0(a301Handle));
  }

  /**
   * Get Periodic Status 1 for the FIRST A301.
   *
   * @return periodic status 1, or null if it's not found
   */
  private Signal<PeriodicStatus1> getPeriodicStatus1() {
    throwIfClosed();
    return Signal.create(CANA301JNI.c_A301_GetPeriodicStatus1(a301Handle));
  }

  /**
   * Get Periodic Status 2 for the FIRST A301.
   *
   * @return periodic status 2, or null if it's not found
   */
  private Signal<PeriodicStatus2> getPeriodicStatus2() {
    throwIfClosed();
    return Signal.create(CANA301JNI.c_A301_GetPeriodicStatus2(a301Handle));
  }

  /**
   * Get Periodic Status 3 for the FIRST A301.
   *
   * @return periodic status 3, or null if it's not found
   */
  private Signal<PeriodicStatus3> getPeriodicStatus3() {
    throwIfClosed();
    return Signal.create(CANA301JNI.c_A301_GetPeriodicStatus3(a301Handle));
  }

  private enum ControlType {
    kDutyCycle(0),
    kVelocity(1),
    kVoltage(2),
    kRelativePosition(3),
    kAbsolutePosition(4),
    kCurrent(5);

    @SuppressWarnings("MemberName")
    public final int value;

    ControlType(int value) {
      this.value = value;
    }
  }

  /**
   * Set the controller setpoint based on the selected control mode.
   *
   * @param setpoint The setpoint to set depending on the control mode.
   *     <pre>
   *    For:
   *    - Duty Cycle (-1.0 to 1.0)
   *    - Velocity Control: Velocity (RPM)
   *    - Voltage Control: Voltage (volts) (-12.0 to 12.0)
   *    - Relative Position Control: Position (Rotations)
   *    - Absolute Position Control: Position (-0.5 to 0.5)
   *    - Current Control: Current (Amps).
   *                      </pre>
   *
   * @param ctrl Is the control type
   * @param positionSpeed The speed to approach the position at (position control only)
   * @return REVLibError.kOk if successful
   */
  private REVLibError setSetpoint(ControlType ctrl, double setpoint, double positionSpeed) {
    throwIfClosed();
    return REVLibError.fromInt(
        CANA301JNI.c_A301_SetpointCommand(
            a301Handle, (float) setpoint, ctrl.value, (float) positionSpeed));
  }

  private static final double kDefaultPositionSpeed = 0.0; // maximum speed

  /**
   * Set the controller setpoint based on the selected control mode.
   *
   * @param setpoint The setpoint to set depending on the control mode.
   *     <pre>
   *    For:
   *    - Duty Cycle (-1.0 to 1.0)
   *    - Velocity Control: Velocity (RPM)
   *    - Voltage Control: Voltage (volts) (-12.0 to 12.0)
   *    - Relative Position Control: Position (Rotations)
   *    - Absolute Position Control: Position (-0.5 to 0.5)
   *    - Current Control: Current (Amps).
   *                 </pre>
   *
   * @param ctrl Is the control type
   * @return REVLibError.kOk if successful
   */
  private REVLibError setSetpoint(ControlType ctrl, double setpoint) {
    throwIfClosed();
    return setSetpoint(ctrl, setpoint, kDefaultPositionSpeed);
  }

  private void throwIfClosed() {
    if (isClosed.get()) {
      throw new IllegalStateException("This FIRST A301 object has previously been closed.");
    }
  }

  public enum IdleMode {
    kCoast(0),
    kBrake(1),
    ;

    private int value;

    IdleMode(int value) {
      this.value = value;
    }
  }
}
