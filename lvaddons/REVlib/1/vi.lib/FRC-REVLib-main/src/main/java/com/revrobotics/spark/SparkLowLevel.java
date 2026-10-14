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

import static com.revrobotics.util.Signal.create;

import com.revrobotics.*;
import com.revrobotics.jni.CANSparkJNI;
import com.revrobotics.util.Signal;
import java.nio.ByteBuffer;
import java.util.concurrent.atomic.AtomicBoolean;
import org.wpilib.hardware.motor.MotorController;

public abstract class SparkLowLevel extends NativeResourceCleaner
    implements MotorController, REVDevice, AutoCloseable {
  public static final int kAPIMajorVersion = CANSparkJNI.c_Spark_GetAPIMajorRevision();
  public static final int kAPIMinorVersion = CANSparkJNI.c_Spark_GetAPIMinorRevision();
  public static final int kAPIBuildVersion = CANSparkJNI.c_Spark_GetAPIBuildRevision();
  public static final int kAPIVersion = CANSparkJNI.c_Spark_GetAPIVersion();

  public enum MotorType {
    kBrushed(0),
    kBrushless(1);

    @SuppressWarnings("MemberName")
    public final int value;

    MotorType(int value) {
      this.value = value;
    }

    public static MotorType fromId(int id) {
      for (MotorType type : values()) {
        if (type.value == id) {
          return type;
        }
      }
      return null;
    }
  }

  public enum ControlType {
    kDutyCycle(0),
    kVelocity(1),
    kVoltage(2),
    kPosition(3),
    kCurrent(4),
    kMAXMotionPositionControl(5),
    kMAXMotionVelocityControl(6);

    @SuppressWarnings("MemberName")
    public final int value;

    ControlType(int value) {
      this.value = value;
    }
  }

  public enum PeriodicFrame {
    kStatus0(0),
    kStatus1(1),
    kStatus2(2),
    kStatus3(3),
    kStatus4(4),
    kStatus5(5),
    kStatus6(6),
    kStatus7(7),
    kStatus8(8);

    @SuppressWarnings("MemberName")
    public final int value;

    PeriodicFrame(int value) {
      this.value = value;
    }

    public static PeriodicFrame fromId(int id) {
      for (PeriodicFrame type : values()) {
        if (type.value == id) {
          return type;
        }
      }
      return null;
    }
  }

  public record PeriodicStatus0(
      double appliedOutput,
      double voltage,
      double current,
      int motorTemperature,
      boolean hardForwardLimitReached,
      boolean hardReverseLimitReached,
      boolean softForwardLimitReached,
      boolean softReverseLimitReached,
      boolean inverted,
      boolean primaryHeartbeatLock,
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
      double primaryEncoderVelocity, double primaryEncoderPosition, int revlibError, long timestamp)
      implements StatusFrame {}

  public record PeriodicStatus3(
      double analogVoltage,
      double analogVelocity,
      double analogPosition,
      int revlibError,
      long timestamp)
      implements StatusFrame {}

  public record PeriodicStatus4(
      double externalOrAltEncoderVelocity,
      double externalOrAltEncoderPosition,
      int revlibError,
      long timestamp)
      implements StatusFrame {}

  public record PeriodicStatus5(
      double dutyCycleEncoderVelocity,
      double dutyCycleEncoderPosition,
      int revlibError,
      long timestamp)
      implements StatusFrame {}

  public record PeriodicStatus6(
      double unadjustedDutyCycle,
      double dutyCyclePeriod,
      boolean dutyCycleNoSignal,
      int revlibError,
      long timestamp)
      implements StatusFrame {}

  public record PeriodicStatus7(double iAccumulation, int revlibError, long timestamp)
      implements StatusFrame {}

  public record PeriodicStatus8(
      double setpoint, boolean isAtSetpoint, int selectedPidSlot, int revlibError, long timestamp)
      implements StatusFrame {}

  public record PeriodicStatus9(
      double maxmotionSetpointPosition,
      double maxmotionSetpointVelocity,
      int revlibError,
      long timestamp)
      implements StatusFrame {}

  protected enum SparkModel {
    Unknown(0),
    SparkFlex(1),
    SparkMax(2);

    final int id;

    SparkModel(int id) {
      this.id = id;
    }

    static SparkModel fromId(int id) {
      for (SparkModel model : values()) {
        if (model.id == id) return model;
      }
      return Unknown;
    }
  }

  protected final long sparkHandle;
  protected final AtomicBoolean isClosed = new AtomicBoolean(false);
  private final int busId;
  private final int deviceId;
  private String firmwareString;
  protected final MotorType motorType;
  protected final SparkModel expectedSparkModel;

  /**
   * Create a new SPARK Controller
   *
   * <p>Package-private, so that only other classes in our package can create
   *
   * @param busId The CAN bus ID this device will be on.
   * @param deviceId The device ID
   * @param type The motor type connected to the controller. Brushless motors must be connected to
   *     their matching color and the hall sensor plugged in. Brushed motors must be connected to
   *     the Red and Black terminals only.
   */
  SparkLowLevel(int busId, int deviceId, MotorType type, SparkModel model) {
    if (type == null) {
      throw new IllegalArgumentException("type must not be null");
    }
    this.busId = busId;
    this.deviceId = deviceId;
    firmwareString = "";
    motorType = type;
    expectedSparkModel = model;
    if (CANSparkJNI.c_Spark_RegisterId(busId, deviceId) == REVLibError.kDuplicateCANId.value) {
      throw new IllegalStateException(
          "A CANSparkMax instance has already been created with this device ID: "
              + deviceId
              + " on Bus: "
              + busId);
    }
    MutableInt status = new MutableInt(0);
    sparkHandle = CANSparkJNI.c_Spark_Create(busId, deviceId, type.value, model.id, status);

    if (REVLibError.fromInt(status.value) != REVLibError.kOk) {
      switch (REVLibError.fromInt(status.value)) {
        case kCantFindFirmware:
          // Don't throw exception when no firmware is found. It's
          // possible the device is disconnected and we don't want to stop
          // the program if that is the case.
          break;
        case kFirmwareTooOld:
        case kFirmwareTooNew:
          // Don't throw for 2026 season. We added this late into the season. The driver
          // will print
          // an error in the driver station but won't throw an exception.
          break;
        case kSparkFlexBrushedWithoutDock:
          throw new IllegalStateException(
              "Cannot set motor type to kBrushed for Bus #"
                  + busId
                  + " SPARK #"
                  + deviceId
                  + " without a dock connected.");
        default:
          throw new IllegalStateException(
              "Error (" + status.value + ") creating Bus #" + busId + " SPARK #" + deviceId);
      }
    }

    registerCleaner(sparkHandle);
  }

  /** Closes the SPARK Controller */
  @Override
  public void close() {
    boolean wasClosed = isClosed.getAndSet(true);
    if (wasClosed) {
      return;
    }
    // Logically close the Spark
    CANSparkJNI.c_Spark_Close(sparkHandle);
  }

  @Override
  protected OnClean getCleanAction() {
    return CANSparkJNI::c_Spark_Destroy;
  }

  /**
   * Get the firmware version of the SPARK.
   *
   * @return uint32_t Firmware version integer. Value is represented as 4 bytes, Major.Minor.Build
   *     H.Build L
   */
  public int getFirmwareVersion() {
    throwIfClosed();
    return CANSparkJNI.c_Spark_GetFirmwareVersion(sparkHandle);
  }

  /**
   * Set the control frame send period for the native CAN Send thread.
   *
   * @param periodMs The send period in milliseconds between 1ms and 100ms or set to 0 to disable
   *     periodic sends. Note this is not updated until the next call to Set() or SetReference().
   */
  public void setControlFramePeriodMs(int periodMs) {
    throwIfClosed();
    CANSparkJNI.c_Spark_SetControlFramePeriod(sparkHandle, periodMs);
  }

  /**
   * Get the firmware version of the SPARK as a string.
   *
   * @return std::string Human readable firmware version string
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
   * Get the unique serial number of the SPARK. Not currently available.
   *
   * @return byte[] Vector of bytes representig the unique serial number
   */
  public byte[] getSerialNumber() {
    throwIfClosed();
    return new byte[0];
  }

  /**
   * Get the configured CAN Bus ID of the SPARK.
   *
   * @return int CAN bus ID
   */
  public int getBusId() {
    throwIfClosed();
    return busId;
  }

  /**
   * Get the configured Device ID of the SPARK.
   *
   * @return int device ID
   */
  public int getDeviceId() {
    throwIfClosed();
    return deviceId;
  }

  /**
   * Get the motor type setting for the SPARK
   *
   * @return MotorType Motor type setting
   */
  public MotorType getMotorType() {
    throwIfClosed();
    return motorType;
  }

  /**
   * Set the amount of time to wait for a periodic status frame before returning a timeout error.
   * This timeout will apply to all periodic status frames for the SPARK motor controller.
   *
   * <p>To prevent invalid timeout errors, the minimum timeout for a given periodic status is 2.1
   * times its period. To use the minimum timeout for all status frames, set timeoutMs to 0.
   *
   * <p>The default timeout is 500ms.
   *
   * @param timeoutMs The timeout in milliseconds
   */
  public void setPeriodicFrameTimeout(int timeoutMs) {
    throwIfClosed();
    CANSparkJNI.c_Spark_SetPeriodicFrameTimeout(sparkHandle, timeoutMs);
  }

  /**
   * @return periodic status 0 signal wrapper
   */
  public Signal<PeriodicStatus0> getPeriodicStatus0() {
    throwIfClosed();
    return create(CANSparkJNI.c_Spark_GetPeriodicStatus0(sparkHandle));
  }

  /**
   * @return periodic status 1 signal wrapper
   */
  public Signal<PeriodicStatus1> getPeriodicStatus1() {
    throwIfClosed();
    return create(CANSparkJNI.c_Spark_GetPeriodicStatus1(sparkHandle));
  }

  /**
   * @return periodic status 2 signal wrapper
   */
  public Signal<PeriodicStatus2> getPeriodicStatus2() {
    throwIfClosed();
    return create(CANSparkJNI.c_Spark_GetPeriodicStatus2(sparkHandle));
  }

  /**
   * @return periodic status 3 signal wrapper
   */
  public Signal<PeriodicStatus3> getPeriodicStatus3() {
    throwIfClosed();
    return create(CANSparkJNI.c_Spark_GetPeriodicStatus3(sparkHandle));
  }

  /**
   * @return periodic status 4 signal wrapper
   */
  public Signal<PeriodicStatus4> getPeriodicStatus4() {
    throwIfClosed();
    return create(CANSparkJNI.c_Spark_GetPeriodicStatus4(sparkHandle));
  }

  /**
   * @return periodic status 5 signal wrapper
   */
  public Signal<PeriodicStatus5> getPeriodicStatus5() {
    throwIfClosed();
    return create(CANSparkJNI.c_Spark_GetPeriodicStatus5(sparkHandle));
  }

  /**
   * @return periodic status 6 signal wrapper
   */
  public Signal<PeriodicStatus6> getPeriodicStatus6() {
    throwIfClosed();
    return create(CANSparkJNI.c_Spark_GetPeriodicStatus6(sparkHandle));
  }

  /**
   * @return periodic status 7 signal wrapper
   */
  public Signal<PeriodicStatus7> getPeriodicStatus7() {
    throwIfClosed();
    return create(CANSparkJNI.c_Spark_GetPeriodicStatus7(sparkHandle));
  }

  /**
   * @return periodic status 8 signal wrapper
   */
  public Signal<PeriodicStatus8> getPeriodicStatus8() {
    throwIfClosed();
    return create(CANSparkJNI.c_Spark_GetPeriodicStatus8(sparkHandle));
  }

  /**
   * @return periodic status 9 signal wrapper
   */
  public Signal<PeriodicStatus9> getPeriodicStatus9() {
    throwIfClosed();
    return create(CANSparkJNI.c_Spark_GetPeriodicStatus9(sparkHandle));
  }

  /**
   * Set the maximum number of times to retry an RTR CAN frame. This applies to calls such as
   * SetParameter* and GetParameter* where a request is made to the SPARK motor controller and a
   * response is expected. Anytime sending the request or receiving the response fails, it will
   * retry the request a number of times, no more than the value set by this method. If an attempt
   * succeeds, it will immediately return. The minimum number of retries is 0, where only a single
   * attempt will be made and will return regardless of success or failure.
   *
   * <p>The default maximum is 5 retries.
   *
   * @param numRetries The maximum number of retries
   */
  public void setCANMaxRetries(int numRetries) {
    throwIfClosed();
    CANSparkJNI.c_Spark_SetCANMaxRetries(sparkHandle, numRetries);
  }

  protected REVLibError setpointCommand(double value, ControlType ctrl) {
    throwIfClosed();
    return setpointCommand(value, ctrl, 0, 0.0);
  }

  protected REVLibError setpointCommand(
      double value, ControlType ctrl, int pidSlot, double arbFeedforward) {
    throwIfClosed();
    return setpointCommand(value, ctrl, pidSlot, arbFeedforward, 0);
  }

  protected REVLibError setpointCommand(
      double value, ControlType ctrl, int pidSlot, double arbFeedforward, int arbFFUnits) {
    throwIfClosed();
    return REVLibError.fromInt(
        CANSparkJNI.c_Spark_SetpointCommand(
            sparkHandle, (float) value, ctrl.value, pidSlot, (float) arbFeedforward, arbFFUnits));
  }

  public float getSafeFloat(float f) {
    throwIfClosed();
    if (Float.isNaN(f) || Float.isInfinite(f)) return 0;

    return f;
  }

  /** Create the sim gui Fault Manager for this Spark Device */
  public void createSimFaultManager() {
    CANSparkJNI.c_Spark_CreateSimFaultManager(sparkHandle);
  }

  protected void throwIfClosed() {
    if (isClosed.get()) {
      throw new IllegalStateException("This SPARK object has previously been closed.");
    }
  }
}
