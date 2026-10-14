/*
 * Copyright (c) 2025-2026 REV Robotics
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

package com.revrobotics.encoder;

import static com.revrobotics.util.Signal.create;

import com.revrobotics.*;
import com.revrobotics.encoder.config.DetachedEncoderAccessor;
import com.revrobotics.encoder.config.DetachedEncoderConfig;
import com.revrobotics.jni.DetachedEncoderJNI;
import com.revrobotics.util.Signal;
import java.util.concurrent.atomic.AtomicBoolean;

public abstract class DetachedEncoder extends NativeResourceCleaner
    implements REVDevice, RelativeEncoder {
  protected long handle;
  private final int busId;
  private final int deviceId;
  private final AtomicBoolean isClosed = new AtomicBoolean(false);
  private final Model model;

  /**
   * Accessor for Detached encoder parameter values. This object contains fields and methods to
   * retrieve parameters that have been applied to the device. To set parameters, see {@link
   * DetachedEncoderConfig} and {@link #configure(DetachedEncoderConfig,
   * com.revrobotics.ResetMode)}.
   *
   * <p>NOTE: This uses calls that are blocking to retrieve parameters and should be used
   * infrequently.
   */
  public final DetachedEncoderAccessor detachedEncoderAccessor;

  /**
   * Create a new object to control a Detached Encoder
   *
   * @param bus The CAN bus ID this device will be on.
   * @param id The device ID.
   * @param model The specific model of detached encoder
   */
  public DetachedEncoder(int bus, int id, Model model) {
    this.busId = bus;
    this.deviceId = id;
    this.model = model;

    if (DetachedEncoderJNI.registerId(busId, deviceId) == REVLibError.kDuplicateCANId.value) {
      throw new IllegalStateException(
          "A DetachedEncoder instance has already been created on Bus "
              + busId
              + " with this device ID: "
              + deviceId);
    }
    handle = DetachedEncoderJNI.create(bus, id, model.ordinal());
    registerCleaner(handle);

    detachedEncoderAccessor = new DetachedEncoderAccessor(handle);
  }

  /**
   * Get the configured CAN Bus ID of the Detached Encoder.
   *
   * @return int CAN bus ID
   */
  public int GetBusId() {
    throwIfClosed();
    return busId;
  }

  /**
   * Get the configured Device ID of the Detached encoder.
   *
   * @return int device ID
   */
  public int getDeviceId() {
    throwIfClosed();
    return deviceId;
  }

  /**
   * Get the Model of this Detached Encoder Device. Useful for determining if this is a MAXSpline,
   * or other device
   *
   * @return the model of this encoder
   */
  public Model getModel() {
    return model;
  }

  /**
   * Get the position of the encoder. This returns the native units of 'rotations' by default, and
   * can be changed by a scale factor using {@link
   * DetachedEncoderConfig#positionConversionFactor(float)}.
   *
   * @return Number of rotations of the encoder
   */
  @Override
  public Signal<Double> getPosition() {
    throwIfClosed();
    var status = getStatus3();
    return status.map(statusFrame -> (double) statusFrame.position);
  }

  /**
   * Get the absolute position of the encoder. This returns the native units of 'rotations' [0, 1)
   * by default, and can be changed by a scale factor using {@link
   * DetachedEncoderConfig#positionConversionFactor(float)}.
   *
   * @return Number of rotations of the encoder
   */
  public Signal<Double> getAngle() {
    throwIfClosed();
    var status = getStatus2();
    return status.map(statusFrame -> (double) statusFrame.angle);
  }

  /**
   * Get the absolute position of the encoder. This returns the native units of 'rotations' [0, 1)
   * without scaling from conversion factors.
   *
   * @return Number of rotations of the encoder
   */
  public Signal<Double> getRawAngle() {
    throwIfClosed();
    var status = getStatus2();
    return status.map(statusFrame -> (double) statusFrame.rawAngle);
  }

  /**
   * Get the velocity of the encoder. This returns the native units of 'RPM' by default, and can be
   * changed by a scale factor using {@link DetachedEncoderConfig#velocityConversionFactor(float)}.
   *
   * @return Number the RPM of the encoder
   */
  @Override
  public Signal<Double> getVelocity() {
    throwIfClosed();
    var status = getStatus4();
    return status.map(statusFrame -> (double) statusFrame.velocity);
  }

  /**
   * Set the position of the encoder. By default the units are 'rotations' and can be changed by a
   * scale factor using {@link DetachedEncoderConfig#positionConversionFactor(float)}.
   *
   * @param position Number of rotations of the motor
   * @return {@link REVLibError#kOk} if successful
   */
  @Override
  public REVLibError setPosition(double position) {
    throwIfClosed();
    return REVLibError.fromInt(DetachedEncoderJNI.setEncoderPosition(handle, position));
  }

  /**
   * Get the active faults that are currently present on the detached encoder. Faults are fatal
   * errors that prevent the encoder from functioning.
   *
   * @return A struct with each fault and their active value
   */
  public Signal<Faults> getFaults() {
    throwIfClosed();
    var status = getStatus1();
    return status.map(
        statusFrame ->
            new Faults(
                statusFrame.unexpectedFault,
                statusFrame.canTxFault,
                statusFrame.canRxFault,
                statusFrame.eepromFault));
  }

  /**
   * Get the sticky faults that were present on the detached encoder at one point since the sticky
   * faults were last cleared. Faults are fatal errors that prevent the encoder from functioning.
   *
   * <p>Sticky faults can be cleared with {@link DetachedEncoder#clearFaults()}.
   *
   * @return A struct with each fault and their sticky value
   */
  public Signal<Faults> getStickyFaults() {
    throwIfClosed();

    var status = getStatus1();
    return status.map(
        statusFrame ->
            new Faults(
                statusFrame.stickyUnexpectedFault,
                statusFrame.stickyCanTxFault,
                statusFrame.stickyCanRxFault,
                statusFrame.stickyEepromFault));
  }

  /** Clears all sticky faults. */
  public void clearFaults() {
    throwIfClosed();
    DetachedEncoderJNI.clearFaults(handle);
  }

  /**
   * Get the firmware version of the detached encoder.
   *
   * @return Firmware version object
   */
  public FirmwareVersion getFirmwareVersion() {
    throwIfClosed();
    return DetachedEncoderJNI.getFirmwareVersion(handle);
  }

  /**
   * Set the configuration for the Detached encoder.
   *
   * <p>If {@code resetMode} is {com.revrobotics.ResetMode#kResetSafeParameters}, this method will
   * reset safe writable parameters to their default values before setting the given configuration.
   * The following parameters will not be reset by this action: CAN ID and Absolute (Duty Cycle)
   * Zero Offset.
   *
   * @param config The desired Detached encoder configuration
   * @param resetMode Whether to reset safe parameters before setting the configuration
   */
  public void configure(DetachedEncoderConfig config, ResetMode resetMode) {
    var configString = config.flatten();

    DetachedEncoderJNI.configure(handle, configString, resetMode == ResetMode.kResetSafeParameters);
  }

  /**
   * @return periodic status 0, or null if it's not found
   */
  public Signal<PeriodicStatus0> getStatus0() {
    throwIfClosed();
    return create(DetachedEncoderJNI.getStatus0(handle));
  }

  /**
   * @return periodic status 1, or null if it's not found
   */
  public Signal<PeriodicStatus1> getStatus1() {
    throwIfClosed();
    return create(DetachedEncoderJNI.getStatus1(handle));
  }

  /**
   * @return periodic status 2, or null if it's not found
   */
  public Signal<PeriodicStatus2> getStatus2() {
    throwIfClosed();
    return create(DetachedEncoderJNI.getStatus2(handle));
  }

  /**
   * @return periodic status 3, or null if it's not found
   */
  public Signal<PeriodicStatus3> getStatus3() {
    throwIfClosed();
    return create(DetachedEncoderJNI.getStatus3(handle));
  }

  /**
   * @return periodic status 4, or null if it's not found
   */
  public Signal<PeriodicStatus4> getStatus4() {
    throwIfClosed();
    return create(DetachedEncoderJNI.getStatus4(handle));
  }

  /** Closes the Detached encoder controller */
  public void close() {
    if (isClosed.get()) {
      return;
    }
    isClosed.set(true);
    DetachedEncoderJNI.close(handle);
  }

  @Override
  protected OnClean getCleanAction() {
    return DetachedEncoderJNI::destroy;
  }

  private void throwIfClosed() {
    if (isClosed.get()) {
      throw new IllegalStateException("This DetachedEncoder object has previously been closed.");
    }
  }

  public record Faults(boolean unexpected, boolean canTx, boolean canRx, boolean eeprom) {}

  public record FirmwareVersion(
      int major, int minor, int fix, int prerelease, int hardwareMajor, int hardwareMinor) {}

  public enum Model {
    Unknown,
    MAXSplineEncoder,
  }

  public record PeriodicStatus0(int deviceModel, int revlibError, long timestamp)
      implements StatusFrame {}

  public record PeriodicStatus1(
      boolean unexpectedFault,
      boolean hasResetFault,
      boolean canTxFault,
      boolean canRxFault,
      boolean eepromFault,
      boolean stickyUnexpectedFault,
      boolean stickyHasResetFault,
      boolean stickyCanTxFault,
      boolean stickyCanRxFault,
      boolean stickyEepromFault,
      int revlibError,
      long timestamp)
      implements StatusFrame {}

  public record PeriodicStatus2(float rawAngle, float angle, int revlibError, long timestamp)
      implements StatusFrame {}

  public record PeriodicStatus3(float position, int revlibError, long timestamp)
      implements StatusFrame {}

  public record PeriodicStatus4(float velocity, int revlibError, long timestamp)
      implements StatusFrame {}
}
