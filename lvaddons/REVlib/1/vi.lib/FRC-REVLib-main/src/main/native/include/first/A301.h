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

#pragma once

#include <memory>
#include <string>

#include <wpi/hardware/motor/MotorController.hpp>

// TODO
#include "rev/REVLibError.h"
#include "rev/util/Signal.h"

namespace first::a301 {

namespace internal {
class A301;
}  // namespace internal

// The default A301 device ID is set to 3 out of the factory.
constexpr int defaultDeviceId{3};

class A301 : public wpi::MotorController {
public:
    /**
     * Create a new object to control a FIRST A301 motor
     *
     * NOTE: If busId indicates a Motioncore bus, the device ID will be
     * auto-detected, otherwise, the defaultDeviceId is used.
     *
     * @param busId The CAN bus ID this device will be on.
     * @param deviceId The device ID.
     */
    explicit A301(int busId);

    /**
     * Create a new object to control a FIRST A301 motor
     *
     * NOTE: If busId indicates a Motioncore bus, the device ID will be
     * auto-detected and deviceId will be ignored.
     *
     * @param busId The CAN bus ID this device will be on.
     * @param deviceId The device ID.
     */
    A301(int busId, int deviceId);

    /**
     * Closes the A301
     */
    ~A301() override;

    /**
     * Get the configured CAN Bus ID of the FIRST A301.
     *
     * @return int CAN bus ID
     *
     */
    int GetBusId() const;

    /**
     * Get the configured Device ID of the FIRST A301.
     *
     * @return int device ID
     *
     */
    int GetDeviceId() const;

    struct FirmwareVersion {
        uint8_t major;
        uint8_t minor;
        uint16_t patch;
        uint8_t prerelease;
    };

    /**
     * Get the firmware version of the FIRST A301.
     *
     * @return FirmwareVersion
     *
     */
    FirmwareVersion GetFirmwareVersion() const;

    /**
     * Get the firmware version of the FIRST A301 as a string.
     *
     * @return std::string Human readable firmware version string
     *
     */
    std::string GetFirmwareString() const;

    /**
     * Get whether the A301 has one or more active faults.
     *
     * @return Signal containing true if there is an active fault
     * @see GetFaults()
     */

    rev::util::Signal<bool> HasActiveFault() const;

    /**
     * Get whether the A301 has one or more sticky faults.
     *
     * @return Signal containing true if there is a sticky fault
     * @see GetStickyFaults()
     */
    rev::util::Signal<bool> HasStickyFault() const;

    /**
     * Get whether the A301 has one or more active warnings.
     *
     * @return Signal containing true if there is an active warning
     * @see GetWarnings()
     */
    rev::util::Signal<bool> HasActiveWarning() const;

    /**
     * Get whether the A301 has one or more sticky warnings.
     *
     * @return Signal containing true if there is a sticky warning
     * @see GetStickyWarnings()
     */
    rev::util::Signal<bool> HasStickyWarning() const;

    struct Faults {
        bool other{};
        bool sensor{};
        bool can{};
        bool temperature{};
        bool gateDriver{};
        bool escEeprom{};
        bool firmware{};
        bool motorStartup{};
        uint16_t rawBits{};

        Faults() = default;

        explicit Faults(uint16_t faults);
    };

    /**
     * Get the active faults that are currently present on the A301. Faults
     * are fatal errors that prevent the motor from running.
     *
     * @return Signal containing each fault and their active value
     */
    rev::util::Signal<Faults> GetFaults() const;

    /**
     * Get the sticky faults that were present on the A301 at one point
     * since the sticky faults were last cleared. Faults are fatal errors
     * that prevent the motor from running.
     *
     * <p>Sticky faults can be cleared with A301::ClearFaults().
     *
     * @return Signal containing each fault and their sticky value
     */
    rev::util::Signal<Faults> GetStickyFaults() const;

    struct Warnings {
        bool overvoltage{};
        bool motorLoopSpeed{};
        bool brownout{};
        bool overcurrent{};
        bool escEeprom{};
        bool extEeprom{};
        bool sensor{};
        bool stall{};
        bool hasReset{};
        bool other{};
        uint16_t rawBits{};

        Warnings() = default;

        explicit Warnings(uint16_t warnings);
    };

    /**
     * Get the active warnings that are currently present on the A301.
     * Warnings are non-fatal errors.
     *
     * @return Signal containing each warning and their active value
     */
    rev::util::Signal<Warnings> GetWarnings() const;

    /**
     * Get the sticky warnings that were present on the A301 at one point
     * since the sticky warnings were last cleared. Warnings are non-fatal
     * errors.
     *
     * <p>Sticky warnings can be cleared with A301::clearFaults().
     *
     * @return Signal containing each warning and their sticky value
     */
    rev::util::Signal<Warnings> GetStickyWarnings() const;

    /**
     * Returns the voltage fed into the A301.
     *
     * @return Signal containing the bus voltage
     */
    rev::util::Signal<double> GetBusVoltage() const;

    /**
     * Returns the A301's output duty cycle.
     *
     * @return Signal containing the applied output duty cycle
     */
    rev::util::Signal<double> GetAppliedOutput() const;

    /**
     * Returns A301's motor current in Amps.
     *
     * @return Signal containing the motor current in Amps
     */
    rev::util::Signal<double> GetMotorCurrent() const;

    /**
     * Returns the motor temperature in Celsius.
     *
     * @return Signal containing the motor temperature in Celsius
     */
    rev::util::Signal<double> GetMotorTemperature() const;

    enum class GearboxRPM {
        kRPM_Unknown,
        kRPM_215,
        kRPM_500,
    };

    /**
     * Returns the RPM of the A301's attached gearbox.
     *
     * @return Signal containing the gearbox RPM (@see GearboxRPM)
     */
    rev::util::Signal<GearboxRPM> GetGearboxRPM() const;

    /**
     * Get the position of the motor. This returns the native units
     * of 'rotations'.
     *
     * @return Signal containing the number of rotations of the motor
     */
    rev::util::Signal<double> GetRelativeEncoderPosition() const;

    /**
     * Get the velocity of the motor. This returns the native units
     * of 'RPM'.
     *
     * @return Signal containing the RPM of the motor
     */
    rev::util::Signal<double> GetEncoderVelocity() const;

    /**
     * Get the absolute position of the motor. This returns the native units
     * of 'rotations'.
     *
     * @return Signal containing the number of rotations of the motor
     */
    rev::util::Signal<double> GetAbsoluteEncoderPosition() const;

    /**
     * Set the position of the relative encoder.
     *
     * @param position Number of rotations of the motor
     * @return REVLibError::kOk if successful
     */
    rev::REVLibError SetRelativeEncoderPosition(double position);

    /**
     * Set the position of the absolute encoder.
     *
     * @param position Number of rotations of the motor
     * @return REVLibError::kOk if successful
     */
    rev::REVLibError SetAbsoluteEncoderPosition(double position);

    /**
     * Clears all sticky faults.
     */
    rev::REVLibError ClearFaults();

    /**
     * Sets the velocity of the A301.
     *
     * @param velocity The velocity (in RPM) to set
     * @return REVLibError::kOk if successful
     */
    rev::REVLibError SetVelocity(double velocity);

    /**
     * Sets the relative position of the A301 (with maximum speed).
     *
     * @param position The relative position (in Rotations) to set
     * @return REVLibError::kOk if successful
     */
    rev::REVLibError SetRelativePosition(double position);

    /**
     * Sets the relative position of the A301 with a specific speed.
     *
     * @param position The relative position (in Rotations) to set
     * @param speed The speed to approach the position at (in RPM)
     * @return REVLibError::kOk if successful
     */
    rev::REVLibError SetRelativePositionWithSpeed(double position,
                                                  double speed);

    /**
     * Sets the absolute position of the A301 (with maximum speed).
     *
     * @param absPosition The absolute position (-0.5 to 0.5) to set
     * @return REVLibError::kOk if successful
     */
    rev::REVLibError SetAbsolutePosition(double absPosition);

    /**
     * Sets the absolute position of the A301 with a specific speed.
     *
     * Setting speed <= 0 will apply maximum speed)
     *
     * @param absPosition The absolute position (-0.5 to 0.5) to set
     * @param speed The speed to approach the position at (in RPM)
     * @return REVLibError::kOk if successful
     */
    rev::REVLibError SetAbsolutePositionWithSpeed(double absPosition,
                                                  double speed);

    enum class IdleMode {
        kCoast,
        kBrake,
    };

    /**
     * Sets the Idle Mode for the motor
     *
     * @return REVLibError::kOk if successful
     */
    rev::REVLibError SetIdleMode(IdleMode idleMode);

    /**
     * Queries the Idle Mode of the motor
     *
     * @return the current idle mode
     */
    IdleMode GetIdleMode() const;

    /**
     * Enables continuous input for absolute position control.
     *
     * @return REVLibError::kOk if successful
     */
    rev::REVLibError EnableAbsolutePositionContinuousInput();

    /**
     * Disables continuous input for absolute position control.
     *
     * @return REVLibError::kOk if successful
     */
    rev::REVLibError DisableAbsolutePositionContinuousInput();

    /**
     * Queries the continuous input for absolute position control.
     *
     * @return true if continuous input is enabled.
     */
    bool IsAbsolutePositionContinuousInputEnabled() const;

    /**
     * Set the offset of the range about zero of the absolute encoder. Moves the
     * range between (-1.0, 0] and [0, 1), instead of the default range [-0.5,
     * 0.5), assuming the default units of rotations.
     *
     * @param offset Fraction of a rotation [-0.5, 0.5] to shift from 0
     * @return REVLibError::kOk if successful
     */
    rev::REVLibError SetAbsoluteEncoderRangeOffset(double offset);

    /**
     * Get the offset of the range about zero of the absolute encoder.
     * This returns the native units of 'rotations'.
     *
     * @return The range offset (in rotations) of the absolute encoder
     */
    double GetAbsoluteEncoderRangeOffset() const;

    /**
     * Sets the motor current of the A301.
     *
     * @param current The current (in Amps) to drive the motor at
     * @return REVLibError::kOk if successful
     */
    rev::REVLibError SetCurrent(double current);

    /**** MotorController Interface ****/

    /**
     * Sets the throttle of the motor controller.
     *
     * @param throttle The throttle where -1.0 indicates full reverse and 1.0
     *     indicates full forward.
     */
    void SetThrottle(double throttle) override;

    /**
     * Gets the throttle of the motor controller.
     *
     * @return The throttle where -1.0 represents full reverse and 1.0
     * represents full forward.
     */
    double GetThrottle() const override;

    /**
     * Sets the voltage output of the SpeedController. The behavior of
     * this call differs slightly from the WPILib documentation for this call
     * since the device internally sets the desired voltage (not a
     * compensation value). That means that this *can* be a 'set-and-forget'
     * call.
     *
     * @param output The voltage to output. (-12.0 V to 12.0 V)
     */
    void SetVoltage(wpi::units::volt_t output) override;

    /**
     * Common interface for setting the inversion state of the motor controller.
     *
     * @param isInverted The inversion state.
     */
    void SetInverted(bool isInverted) override;

    /**
     * Common interface for getting the inversion state of the motor controller.
     *
     * @return The inversion state.
     */
    bool GetInverted() const override;

    /**
     * Common interface for disabling a motor.
     */
    void Disable() override;

    /**** Signals Configuration Interface ****/

    /**
     * Set the period (ms) of the status frame that provides the signal returned
     * by GetFaults() and GetStickyFaults().
     *
     * The default period is 50ms.
     * The maximum period is 1000ms (1s).
     *
     * <p>If multiple periods are set for signals within the same status frame,
     * the minimum given value will be used.
     *
     * @param period_ms The period in milliseconds
     * @return The modified A301 object for method chaining
     */
    A301& FaultsPeriodMs(int period_ms);

    /**
     * Get the period (ms) of the status frame that provides the signal returned
     * by GetFaults() and GetStickyFaults().
     *
     * @return The period in milliseconds
     */
    int GetFaultsPeriodMs() const;

    /**
     * Set the period (ms) of the status frame that provides the signal returned
     * by GetWarnings() and GetStickyWarnings().
     *
     * The default period is 50ms.
     * The maximum period is 1000ms (1s).
     *
     * <p>If multiple periods are set for signals within the same status frame,
     * the minimum given value will be used.
     *
     * @param period_ms The period in milliseconds
     * @return The modified A301 object for method chaining
     */
    A301& WarningsPeriodMs(int period_ms);

    /**
     * Get the period (ms) of the status frame that provides the signal returned
     * by GetWarnings() and GetStickyWarnings().
     *
     * @return The period in milliseconds
     */
    int GetWarningsPeriodMs() const;

    /**
     * Set the period (ms) of the status frame that provides the signal returned
     * by GetBusVoltage().
     *
     * The default period is 10ms.
     * The maximum period is 1000ms (1s).
     *
     * <p>If multiple periods are set for signals within the same status frame,
     * the minimum given value will be used.
     *
     * @param period_ms The period in milliseconds
     * @return The modified A301 object for method chaining
     */
    A301& BusVoltagePeriodMs(int period_ms);

    /**
     * Get the period (ms) of the status frame that provides the signal returned
     * by GetBusVoltage().
     *
     * @return The period in milliseconds
     */
    int GetBusVoltagePeriodMs() const;

    /**
     * Set the period (ms) of the status frame that provides the signal returned
     * by GetAppliedOutput().
     *
     * The default period is 10ms.
     * The maximum period is 1000ms (1s).
     *
     * <p>If multiple periods are set for signals within the same status frame,
     * the minimum given value will be used.
     *
     * @param period_ms The period in milliseconds
     * @return The modified A301 object for method chaining
     */
    A301& AppliedOutputPeriodMs(int period_ms);

    /**
     * Get the period (ms) of the status frame that provides the signal returned
     * by GetAppliedOutput().
     *
     * @return The period in milliseconds
     */
    int GetAppliedOutputPeriodMs() const;

    /**
     * Set the period (ms) of the status frame that provides the signal returned
     * by GetMotorCurrent().
     *
     * The default period is 10ms.
     * The maximum period is 1000ms (1s).
     *
     * <p>If multiple periods are set for signals within the same status frame,
     * the minimum given value will be used.
     *
     * @param period_ms The period in milliseconds
     * @return The modified A301 object for method chaining
     */
    A301& MotorCurrentPeriodMs(int period_ms);

    /**
     * Get the period (ms) of the status frame that provides the signal returned
     * by GetMotorCurrent().
     *
     * @return The period in milliseconds
     */
    int GetMotorCurrentPeriodMs() const;

    /**
     * Set the period (ms) of the status frame that provides the signal returned
     * by GetMotorTemperature().
     *
     * The default period is 10ms.
     * The maximum period is 1000ms (1s).
     *
     * <p>If multiple periods are set for signals within the same status frame,
     * the minimum given value will be used.
     *
     * @param period_ms The period in milliseconds
     * @return The modified A301 object for method chaining
     */
    A301& MotorTemperaturePeriodMs(int period_ms);

    /**
     * Get the period (ms) of the status frame that provides the signal returned
     * by GetMotorTemperature().
     *
     * @return The period in milliseconds
     */
    int GetMotorTemperaturePeriodMs() const;

    /**
     * Set the period (ms) of the status frame that provides the signal returned
     * by GetRelativeEncoderPosition().
     *
     * The default period is 20ms.
     * The maximum period is 1000ms (1s).
     *
     * <p>If multiple periods are set for signals within the same status frame,
     * the minimum given value will be used.
     *
     * @param period_ms The period in milliseconds
     * @return The modified A301 object for method chaining
     */
    A301& RelativeEncoderPositionPeriodMs(int period_ms);

    /**
     * Get the period (ms) of the status frame that provides the signal returned
     * by GetRelativeEncoderPosition().
     *
     * @return The period in milliseconds
     */
    int GetRelativeEncoderPositionPeriodMs() const;

    /**
     * Set the period (ms) of the status frame that provides the signal returned
     * by GetEncoderVelocity().
     *
     * The default period is 20ms.
     * The maximum period is 1000ms (1s).
     *
     * <p>If multiple periods are set for signals within the same status frame,
     * the minimum given value will be used.
     *
     * @param period_ms The period in milliseconds
     * @return The modified A301 object for method chaining
     */
    A301& EncoderVelocityPeriodMs(int period_ms);

    /**
     * Get the period (ms) of the status frame that provides the signal returned
     * by GetEncoderVelocity().
     *
     * @return The period in milliseconds
     */
    int GetEncoderVelocityPeriodMs() const;

    /**
     * Set the period (ms) of the status frame that provides the signal returned
     * by GetAbsoluteEncoderPosition().
     *
     * The default period is 20ms.
     * The maximum period is 1000ms (1s).
     *
     * <p>If multiple periods are set for signals within the same status frame,
     * the minimum given value will be used.
     *
     * @param period_ms The period in milliseconds
     * @return The modified A301 object for method chaining
     */
    A301& AbsoluteEncoderPositionPeriodMs(int period_ms);

    /**
     * Get the period (ms) of the status frame that provides the signal returned
     * by GetAbsoluteEncoderPosition().
     *
     * @return The period in milliseconds
     */
    int GetAbsoluteEncoderPositionPeriodMs() const;

private:
    std::unique_ptr<internal::A301> p;
};

}  // namespace first::a301
