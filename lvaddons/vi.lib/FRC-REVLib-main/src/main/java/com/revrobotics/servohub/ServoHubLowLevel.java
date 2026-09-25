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

package com.revrobotics.servohub;

import static com.revrobotics.util.Signal.create;

import com.revrobotics.NativeResourceCleaner;
import com.revrobotics.REVDevice;
import com.revrobotics.REVLibError;
import com.revrobotics.StatusFrame;
import com.revrobotics.jni.CANServoHubJNI;
import com.revrobotics.util.Signal;
import java.util.concurrent.atomic.AtomicBoolean;

public abstract class ServoHubLowLevel extends NativeResourceCleaner
    implements REVDevice, AutoCloseable {
  protected final long servoHubHandle;
  private final AtomicBoolean isClosed = new AtomicBoolean(false);
  private final int busId;
  private final int deviceId;
  private String firmwareString = "";

  /**
   * Create a new object to control a ServoHub Servo Controller
   *
   * @param busId The CAN bus ID this device will be on.
   * @param deviceId The device ID.
   */
  public ServoHubLowLevel(int busId, int deviceId) {
    this.busId = busId;
    this.deviceId = deviceId;

    if (CANServoHubJNI.c_ServoHub_RegisterId(busId, deviceId)
        == REVLibError.kDuplicateCANId.value) {
      throw new IllegalStateException(
          "A CANServoHub instance has already been created on Bus "
              + busId
              + " with this device ID: "
              + deviceId);
    }
    servoHubHandle = CANServoHubJNI.c_ServoHub_Create(busId, deviceId);
    registerCleaner(servoHubHandle);
  }

  /** Closes the ServoHub Controller */
  @Override
  public void close() {
    boolean wasClosed = isClosed.getAndSet(true);
    if (wasClosed) {
      return;
    }

    CANServoHubJNI.c_ServoHub_Close(servoHubHandle);
  }

  @Override
  protected OnClean getCleanAction() {
    return CANServoHubJNI::c_ServoHub_Destroy;
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
   * Get the configured Device ID of the ServoHub.
   *
   * @return int device ID
   */
  public int getDeviceId() {
    throwIfClosed();
    return deviceId;
  }

  public class FirmwareVersion {
    private int firmwareFix;
    private int firmwareMinor;
    private int firmwareYear;
    private int hardwareMinor;
    private int hardwareMajor;

    public int getYear() {
      return firmwareYear;
    }

    public int getMinor() {
      return firmwareMinor;
    }

    public int getFix() {
      return firmwareFix;
    }

    public int hardwareMajor() {
      return hardwareMajor;
    }

    public int hardwareMinor() {
      return hardwareMinor;
    }
  }

  /**
   * Get the firmware version of the ServoHub.
   *
   * @return FirmwareVersion tbd
   */
  public FirmwareVersion getFirmwareVersion() {
    throwIfClosed();
    FirmwareVersion version = new FirmwareVersion();
    CANServoHubJNI.c_ServoHub_GetFirmwareVersion(servoHubHandle, version);
    return version;
  }

  /**
   * Get the firmware version of the ServoHub as a string.
   *
   * @return String Human readable firmware version string
   */
  public String getFirmwareVersionString() {
    throwIfClosed();
    if (firmwareString == "") {
      FirmwareVersion version = getFirmwareVersion();

      StringBuilder firmwareString = new StringBuilder();
      firmwareString
          .append("fw v")
          .append(version.firmwareYear)
          .append(".")
          .append(version.firmwareMinor)
          .append(".")
          .append(version.firmwareFix)
          .append(", hw v")
          .append(version.hardwareMajor)
          .append(".")
          .append(version.hardwareMinor);

      this.firmwareString = firmwareString.toString();
    }
    return this.firmwareString;
  }

  /**
   * Set the amount of time to wait for a periodic status frame before returning a timeout error.
   * This timeout will apply to all periodic status frames for the ServoHub servo controller.
   *
   * <p>To prevent invalid timeout errors, the minimum timeout for a given periodic status is 2.1
   * times its period. To use the minimum timeout for all status frames, set timeout_ms to 0.
   *
   * <p>The default timeout is 500ms.
   *
   * @param timeout_ms The timeout in milliseconds
   */
  public void setPeriodicFrameTimeout(int timeout_ms) {
    throwIfClosed();
    CANServoHubJNI.c_ServoHub_SetPeriodicFrameTimeout(servoHubHandle, timeout_ms);
  }

  /**
   * Sets the timeout duration for waiting for CAN responses from the device.
   *
   * @param timeout_ms The timeout in milliseconds.
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError setCANTimeout(int timeout_ms) {
    throwIfClosed();
    return REVLibError.fromInt(CANServoHubJNI.c_ServoHub_SetCANTimeout(servoHubHandle, timeout_ms));
  }

  /**
   * Set the maximum number of times to retry an RTR CAN frame. This applies to calls such as
   * GetFirmwareVersion where a request is made to the ServoHub and a response is expected. Anytime
   * sending the request or receiving the response fails, it will retry the request a number of
   * times, no more than the value set by this method. If an attempt succeeds, it will immediately
   * return. The minimum number of retries is 0, where only a single attempt will be made and will
   * return regardless of success or failure.
   *
   * <p>The default maximum is 5 retries.
   *
   * @param numRetries The maximum number of retries
   */
  public void setCANMaxRetries(int numRetries) {
    throwIfClosed();
    CANServoHubJNI.c_ServoHub_SetCANMaxRetries(servoHubHandle, numRetries);
  }

  /**
   * Set the control frame send period for the native CAN Send thread.
   *
   * @param periodMs The send period in milliseconds between 1ms and 100ms or set to 0 to disable
   *     periodic sends.
   */
  public void setControlFramePeriodMs(int periodMs) {
    throwIfClosed();
    CANServoHubJNI.c_ServoHub_SetControlFramePeriod(servoHubHandle, periodMs);
  }

  /**
   * Set the control frame send period for the native CAN Send thread.
   *
   * @return int The send period in milliseconds.
   */
  public int getControlFramePeriodMs() {
    throwIfClosed();
    return CANServoHubJNI.c_ServoHub_GetControlFramePeriod(servoHubHandle);
  }

  public record PeriodicStatus0(
      double voltage,
      double servoVoltage,
      double deviceCurrent,
      boolean primaryHeartbeatLock,
      boolean systemEnabled,
      int communicationMode, // 0: None, 1: CAN, 2: RS-48
      boolean programmingEnabled,
      boolean activelyProgramming,
      int revlibError,
      long timestamp)
      implements StatusFrame {}

  public Signal<PeriodicStatus0> getPeriodicStatus0() {
    throwIfClosed();
    var status = CANServoHubJNI.c_ServoHub_GetPeriodStatus0(servoHubHandle);
    return create(status);
  }

  public record PeriodicStatus1(
      boolean regulatorPowerGoodFault,
      boolean brownout,
      boolean canWarning,
      boolean canBusOff,
      boolean hardwareFault,
      boolean firmwareFault,
      boolean hasReset,
      boolean lowBatteryFault,
      boolean channel0Overcurrent,
      boolean channel1Overcurrent,
      boolean channel2Overcurrent,
      boolean channel3Overcurrent,
      boolean channel4Overcurrent,
      boolean channel5Overcurrent,
      boolean stickyRegulatorPowerGoodFault,
      boolean stickyBrownout,
      boolean stickyCanWarning,
      boolean stickyCanBusOff,
      boolean stickyHardwareFault,
      boolean stickyFirmwareFault,
      boolean stickyHasReset,
      boolean stickyLowBatteryFault,
      boolean stickyChannel0Overcurrent,
      boolean stickyChannel1Overcurrent,
      boolean stickyChannel2Overcurrent,
      boolean stickyChannel3Overcurrent,
      boolean stickyChannel4Overcurrent,
      boolean stickyChannel5Overcurrent,
      int revlibError,
      long timestamp)
      implements StatusFrame {}

  public Signal<PeriodicStatus1> getPeriodicStatus1() {
    throwIfClosed();
    var status = CANServoHubJNI.c_ServoHub_GetPeriodStatus1(servoHubHandle);
    return create(status);
  }

  public record PeriodicStatus2(
      short channel0PulseWidth,
      short channel1PulseWidth,
      short channel2PulseWidth,
      boolean channel0Enabled,
      boolean channel1Enabled,
      boolean channel2Enabled,
      boolean channel0OutOfRange,
      boolean channel1OutOfRange,
      boolean channel2OutOfRange,
      int revlibError,
      long timestamp)
      implements StatusFrame {}

  public Signal<PeriodicStatus2> getPeriodicStatus2() {
    throwIfClosed();
    var status = CANServoHubJNI.c_ServoHub_GetPeriodStatus2(servoHubHandle);
    return create(status);
  }

  public record PeriodicStatus3(
      short channel3PulseWidth,
      short channel4PulseWidth,
      short channel5PulseWidth,
      boolean channel3Enabled,
      boolean channel4Enabled,
      boolean channel5Enabled,
      boolean channel3OutOfRange,
      boolean channel4OutOfRange,
      boolean channel5OutOfRange,
      int revlibError,
      long timestamp)
      implements StatusFrame {}

  public Signal<PeriodicStatus3> getPeriodicStatus3() {
    throwIfClosed();
    var status = CANServoHubJNI.c_ServoHub_GetPeriodStatus3(servoHubHandle);
    return create(status);
  }

  public record PeriodicStatus4(
      double channel0Current,
      double channel1Current,
      double channel2Current,
      double channel3Current,
      double channel4Current,
      double channel5Current,
      int revlibError,
      long timestamp)
      implements StatusFrame {}

  public Signal<PeriodicStatus4> getPeriodicStatus4() {
    throwIfClosed();
    var status = CANServoHubJNI.c_ServoHub_GetPeriodStatus4(servoHubHandle);
    return create(status);
  }

  /** Create the sim gui Fault Manager for this Servo Hub */
  public void createSimFaultManager() {
    CANServoHubJNI.c_ServoHub_CreateSimFaultManager(servoHubHandle);
  }

  protected void throwIfClosed() {
    if (isClosed.get()) {
      throw new IllegalStateException("This ServoHub object has previously been closed.");
    }
  }
}
