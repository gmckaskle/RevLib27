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

import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.jni.CANServoHubJNI;
import com.revrobotics.jni.REVLibJNI;
import com.revrobotics.servohub.config.ServoHubConfig;
import com.revrobotics.servohub.config.ServoHubConfigAccessor;
import com.revrobotics.util.Signal;

public class ServoHub extends ServoHubLowLevel {
  private ServoChannel[] servoChannels = new ServoChannel[ServoChannel.kNumServoChannels];

  /**
   * Accessor for ServoHub parameter values. This object contains fields and methods to retrieve
   * parameters that have been applied to the device. To set parameters, see {@link ServoHubConfig}
   * and {@link ServoHub#configure(ServoHubConfig, com.revrobotics.ResetMode)}.
   *
   * <p>NOTE: This uses calls that are blocking to retrieve parameters and should be used
   * infrequently.
   */
  public final ServoHubConfigAccessor configAccessor;

  /**
   * Create a new object to control a ServoHub Servo Controller
   *
   * @param busId The CAN bus ID this device will be on.
   * @param deviceId The device ID.
   */
  public ServoHub(int busId, int deviceId) {
    super(busId, deviceId);
    configAccessor = new ServoHubConfigAccessor(servoHubHandle);

    for (int i = 0; i < servoChannels.length; ++i) {
      servoChannels[i] = new ServoChannel(ServoChannel.ChannelId.fromInt(i), this);
    }
  }

  /**
   * Set the configuration for the ServoHub.
   *
   * <p>If {@code resetMode} is {@link com.revrobotics.ResetMode#kResetSafeParameters}, this method
   * will reset safe writable parameters to their default values before setting the given
   * configuration.
   *
   * @param config The desired ServoHub configuration
   * @param resetMode Whether to reset safe parameters before setting the configuration
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError configure(ServoHubConfig config, ResetMode resetMode) {
    throwIfClosed();
    REVLibError status =
        REVLibError.fromInt(
            CANServoHubJNI.c_ServoHub_Configure(
                servoHubHandle, config.flatten(), resetMode == ResetMode.kResetSafeParameters));

    if (status != REVLibError.kOk) {
      throw new IllegalStateException(REVLibJNI.c_REVLib_ErrorFromCode(status.value));
    }

    return status;
  }

  /**
   * Set the configuration for the ServoHub without waiting for a response.
   *
   * <p>If {@code resetMode} is {@link ResetMode#kResetSafeParameters}, this method will reset safe
   * writable parameters to their default values before setting the given configuration.
   *
   * <p>NOTE: This method will immediately return {@link REVLibError#kOk} and the action will be
   * done in the background. Any errors that occur will be reported to the driver station.
   *
   * @param config The desired ServoHub configuration
   * @param resetMode Whether to reset safe parameters before setting the configuration
   * @return {@link REVLibError#kOk}
   * @see #configure(ServoHubConfig, ResetMode)
   */
  public REVLibError configureAsync(ServoHubConfig config, ResetMode resetMode) {
    throwIfClosed();
    return REVLibError.fromInt(
        CANServoHubJNI.c_ServoHub_ConfigureAsync(
            servoHubHandle, config.flatten(), resetMode == ResetMode.kResetSafeParameters));
  }

  /**
   * Get whether the ServoHub has one or more active faults.
   *
   * @return true if there is an active fault
   * @see #getFaults()
   */
  public Signal<Boolean> hasActiveFault() {
    throwIfClosed();
    return getFaults().map(status -> status.rawBits != 0);
  }

  /**
   * Get whether the ServoHub has one or more sticky faults.
   *
   * @return true if there is a sticky fault
   * @see #getStickyFaults()
   */
  public Signal<Boolean> hasStickyFault() {
    throwIfClosed();
    return getStickyFaults().map(status -> status.rawBits != 0);
  }

  /**
   * Get whether the ServoHub has one or more active warnings.
   *
   * @return true if there is an active warning
   * @see #getWarnings()
   */
  public Signal<Boolean> hasActiveWarning() {
    throwIfClosed();
    return getWarnings().map(status -> status.rawBits != 0);
  }

  /**
   * Get whether the ServoHub has one or more sticky warnings.
   *
   * @return true if there is a sticky warning
   * @see #getStickyWarnings()
   */
  public Signal<Boolean> hasStickyWarning() {
    throwIfClosed();
    return getStickyWarnings().map(status -> status.rawBits != 0);
  }

  public static class Faults {
    public final boolean regulatorPowerGood;
    public final boolean hardware;
    public final boolean firmware;
    public final boolean lowBattery;
    public final int rawBits;

    public Faults(int faults) {
      rawBits = faults;
      regulatorPowerGood = (faults & 0x1) != 0;
      hardware = (faults & 0x2) != 0;
      firmware = (faults & 0x4) != 0;
      lowBattery = (faults & 0x8) != 0;
    }

    public Faults(
        boolean regulatorPowerGood, boolean hardware, boolean firmware, boolean lowBattery) {
      this.regulatorPowerGood = regulatorPowerGood;
      this.hardware = hardware;
      this.firmware = firmware;
      this.lowBattery = lowBattery;

      rawBits =
          getBit(regulatorPowerGood, 0)
              | getBit(hardware, 1)
              | getBit(firmware, 2)
              | getBit(lowBattery, 3);
    }

    private int getBit(boolean isSet, int index) {
      return isSet ? (1 << index) : 0;
    }
  }

  /**
   * Get the active faults that are currently present on the ServoHub. Faults are fatal errors that
   * prevent the motor from running.
   *
   * @return A struct with each fault and their active value
   */
  public Signal<Faults> getFaults() {
    throwIfClosed();
    var status = getPeriodicStatus1();
    return status.map(
        statusFrame ->
            new Faults(
                statusFrame.regulatorPowerGoodFault(),
                statusFrame.hardwareFault(),
                statusFrame.firmwareFault(),
                statusFrame.lowBatteryFault()));
  }

  /**
   * Get the sticky faults that were present on the ServoHub at one point since the sticky faults
   * were last cleared. Faults are fatal errors that prevent the motor from running.
   *
   * <p>Sticky faults can be cleared with {@link ServoHub#clearFaults()}.
   *
   * @return A struct with each fault and their sticky value
   */
  public Signal<Faults> getStickyFaults() {
    throwIfClosed();
    var status = getPeriodicStatus1();
    return status.map(
        statusFrame ->
            new Faults(
                statusFrame.stickyRegulatorPowerGoodFault(),
                statusFrame.stickyHardwareFault(),
                statusFrame.stickyFirmwareFault(),
                statusFrame.stickyLowBatteryFault()));
  }

  public static class Warnings {
    public final boolean brownout;
    public final boolean canWarning;
    public final boolean canBusOff;
    public final boolean hasReset;
    public final boolean channel0Overcurrent;
    public final boolean channel1Overcurrent;
    public final boolean channel2Overcurrent;
    public final boolean channel3Overcurrent;
    public final boolean channel4Overcurrent;
    public final boolean channel5Overcurrent;
    public final int rawBits;

    public Warnings(int warnings) {
      rawBits = warnings;
      brownout = (warnings & 0x1) != 0;
      canWarning = (warnings & 0x2) != 0;
      canBusOff = (warnings & 0x4) != 0;
      hasReset = (warnings & 0x8) != 0;
      channel0Overcurrent = (warnings & 0x10) != 0;
      channel1Overcurrent = (warnings & 0x20) != 0;
      channel2Overcurrent = (warnings & 0x40) != 0;
      channel3Overcurrent = (warnings & 0x80) != 0;
      channel4Overcurrent = (warnings & 0x100) != 0;
      channel5Overcurrent = (warnings & 0x200) != 0;
    }

    public Warnings(
        boolean brownout,
        boolean canWarning,
        boolean canBusOff,
        boolean hasReset,
        boolean channel0Overcurrent,
        boolean channel1Overcurrent,
        boolean channel2Overcurrent,
        boolean channel3Overcurrent,
        boolean channel4Overcurrent,
        boolean channel5Overcurrent) {
      this.brownout = brownout;
      this.canWarning = canWarning;
      this.canBusOff = canBusOff;
      this.hasReset = hasReset;
      this.channel0Overcurrent = channel0Overcurrent;
      this.channel1Overcurrent = channel1Overcurrent;
      this.channel2Overcurrent = channel2Overcurrent;
      this.channel3Overcurrent = channel3Overcurrent;
      this.channel4Overcurrent = channel4Overcurrent;
      this.channel5Overcurrent = channel5Overcurrent;

      rawBits =
          getBit(brownout, 0)
              | getBit(canWarning, 1)
              | getBit(canBusOff, 2)
              | getBit(hasReset, 3)
              | getBit(channel0Overcurrent, 4)
              | getBit(channel1Overcurrent, 5)
              | getBit(channel2Overcurrent, 6)
              | getBit(channel3Overcurrent, 7)
              | getBit(channel4Overcurrent, 8)
              | getBit(channel5Overcurrent, 9);
    }

    private int getBit(boolean isSet, int index) {
      return isSet ? (1 << index) : 0;
    }
  }

  /**
   * Get the active warnings that are currently present on the ServoHub. Warnings are non-fatal
   * errors.
   *
   * @return A struct with each warning and their active value
   */
  public Signal<Warnings> getWarnings() {
    throwIfClosed();
    var status = getPeriodicStatus1();
    return status.map(
        statusFrame ->
            new Warnings(
                statusFrame.brownout(),
                statusFrame.canWarning(),
                statusFrame.canBusOff(),
                statusFrame.hasReset(),
                statusFrame.channel0Overcurrent(),
                statusFrame.channel1Overcurrent(),
                statusFrame.channel2Overcurrent(),
                statusFrame.channel3Overcurrent(),
                statusFrame.channel4Overcurrent(),
                statusFrame.channel5Overcurrent()));
  }

  /**
   * Get the sticky warnings that were present on the ServoHub at one point since the sticky
   * warnings were last cleared. Warnings are non-fatal errors.
   *
   * <p>Sticky warnings can be cleared with {@link ServoHub#clearFaults()}.
   *
   * @return A struct with each warning and their sticky value
   */
  public Signal<Warnings> getStickyWarnings() {
    throwIfClosed();
    var status = getPeriodicStatus1();
    return status.map(
        statusFrame ->
            new Warnings(
                statusFrame.brownout(),
                statusFrame.canWarning(),
                statusFrame.canBusOff(),
                statusFrame.hasReset(),
                statusFrame.channel0Overcurrent(),
                statusFrame.channel1Overcurrent(),
                statusFrame.channel2Overcurrent(),
                statusFrame.channel3Overcurrent(),
                statusFrame.channel4Overcurrent(),
                statusFrame.channel5Overcurrent()));
  }

  /**
   * Clears all sticky faults.
   *
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError clearFaults() {
    throwIfClosed();
    return REVLibError.fromInt(CANServoHubJNI.c_ServoHub_ClearFaults(servoHubHandle));
  }

  /**
   * @return The voltage fed into the servo controller.
   */
  public Signal<Double> getDeviceVoltage() {
    throwIfClosed();
    var status = getPeriodicStatus0();
    return status.map(PeriodicStatus0::voltage);
  }

  /**
   * @return The servo controller's output current in Amps.
   */
  public Signal<Double> getDeviceCurrent() {
    throwIfClosed();
    var status = getPeriodicStatus0();
    return status.map(PeriodicStatus0::deviceCurrent);
  }

  /**
   * @return The voltage fed to the actual servos.
   */
  public Signal<Double> getServoVoltage() {
    throwIfClosed();
    var status = getPeriodicStatus0();
    return status.map(PeriodicStatus0::servoVoltage);
  }

  /**
   * Returns an object to control a specific servo channel.
   *
   * @param channelId The specific servo channel to get
   * @return The specified ServoChannel
   */
  public ServoChannel getServoChannel(ServoChannel.ChannelId channelId) {
    throwIfClosed();
    return servoChannels[channelId.value];
  }

  public enum Bank {
    kBank0_2(0),
    kBank3_5(1);

    @SuppressWarnings("MemberName")
    public final int value;

    Bank(int value) {
      this.value = value;
    }
  }

  /**
   * Set the Pulse Period for servo channels 0-2 or servo channels 3-5.
   *
   * @param bank The bank of channels (0-2 or 3-5) to set
   * @param pulsePeriod_us The pulse period in microseconds
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError setBankPulsePeriod(Bank bank, int pulsePeriod_us) {
    throwIfClosed();
    return REVLibError.fromInt(
        CANServoHubJNI.c_ServoHub_SetBankPulsePeriod(servoHubHandle, bank.value, pulsePeriod_us));
  }
}
