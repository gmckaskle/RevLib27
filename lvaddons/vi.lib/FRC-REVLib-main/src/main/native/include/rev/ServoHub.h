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

#pragma once

#include <stdint.h>

#include <vector>

#include "rev/ConfigureTypes.h"
#include "rev/REVLibError.h"
#include "rev/ServoChannel.h"
#include "rev/ServoHubLowLevel.h"
#include "rev/config/ServoHubConfig.h"
#include "rev/config/ServoHubConfigAccessor.h"
#include "rev/util/Signal.h"

namespace rev::servohub {

class ServoHub : public ServoHubLowLevel {
    friend class ServoHubSim;

public:
    /**
     * Accessor for ServoHub parameter values. This object contains fields and
     * methods to retrieve parameters that have been applied to the device. To
     * set parameters, see ServoHubConfig and ServoHub::Configure().
     *
     * NOTE: This uses calls that are blocking to retrieve parameters and should
     * be used infrequently.
     */
    ServoHubConfigAccessor configAccessor;

public:
    /**
     * Create a new object to control a ServoHub Servo Controller
     *
     * @param busID The CAN bus ID this device will be on.
     * @param deviceID The device ID.
     */
    ServoHub(int busID, int deviceID);

    /**
     * Closes the ServoHub
     */
    ~ServoHub() override;

    enum class ResetMode { kNoResetSafeParameters, kResetSafeParameters };

    /**
     * Set the configuration for the ServoHub.
     *
     * <p>If @c resetMode is ResetMode::kResetSafeParameters, this
     * method will reset safe writable parameters to their default values before
     * setting the given configuration.
     *
     * @param config The desired ServoHub configuration
     * @param resetMode Whether to reset safe parameters before setting the
     * configuration
     * @return REVLibError::kOk if successful
     */
    REVLibError Configure(
        ServoHubConfig& config,
        rev::ResetMode resetMode = rev::ResetMode::kNoResetSafeParameters);

    /**
     * Set the configuration for the ServoHub without waiting for a response.
     *
     * <p>If @c resetMode is ResetMode::kResetSafeParameters, this
     * method will reset safe writable parameters to their default values before
     * setting the given configuration.
     *
     * <p>NOTE: This method will immediately return REVLibError::kOk and
     * the action will be done in the background. Any errors that occur will be
     * reported to the driver station.
     *
     * @param config The desired ServoHub configuration
     * @param resetMode Whether to reset safe parameters before setting the
     * configuration
     * @return REVLibError::kOk
     * @see Configure()
     */
    REVLibError ConfigureAsync(
        ServoHubConfig& config,
        rev::ResetMode resetMode = rev::ResetMode::kNoResetSafeParameters);

    /**
     * Get whether the ServoHub has one or more active faults.
     *
     * @return Signal containing true if there is an active fault
     * @see GetFaults()
     */
    rev::util::Signal<bool> HasActiveFault() const;

    /**
     * Get whether the ServoHub has one or more sticky faults.
     *
     * @return Signal containing true if there is a sticky fault
     * @see GetStickyFaults()
     */
    rev::util::Signal<bool> HasStickyFault() const;

    /**
     * Get whether the ServoHub has one or more active warnings.
     *
     * @return Signal containing true if there is an active warning
     * @see GetWarnings()
     */
    rev::util::Signal<bool> HasActiveWarning() const;

    /**
     * Get whether the ServoHub has one or more sticky warnings.
     *
     * @return Signal containing true if there is a sticky warning
     * @see GetStickyWarnings()
     */
    rev::util::Signal<bool> HasStickyWarning() const;

    struct Faults {
        bool regulatorPowerGood{};
        bool hardware{};
        bool firmware{};
        bool lowBattery{};
        uint16_t rawBits{};

        Faults() = default;
        explicit Faults(uint16_t faults);
    };

    /**
     * Get the active faults that are currently present on the ServoHub. Faults
     * are fatal errors that prevent the motor from running.
     *
     * @return Signal containing a struct with each fault and their active value
     */
    rev::util::Signal<Faults> GetFaults() const;

    /**
     * Get the sticky faults that were present on the ServoHub at one point
     * since the sticky faults were last cleared. Faults are fatal errors
     * that prevent the motor from running.
     *
     * <p>Sticky faults can be cleared with ServoHub::ClearFaults().
     *
     * @return Signal containing a struct with each fault and their sticky value
     */
    rev::util::Signal<Faults> GetStickyFaults() const;

    struct Warnings {
        bool brownout{};
        bool canWarning{};
        bool canBusOff{};
        bool hasReset{};
        bool channel0Overcurrent{};
        bool channel1Overcurrent{};
        bool channel2Overcurrent{};
        bool channel3Overcurrent{};
        bool channel4Overcurrent{};
        bool channel5Overcurrent{};
        uint16_t rawBits{};

        Warnings() = default;
        explicit Warnings(uint16_t warnings);
    };

    /**
     * Get the active warnings that are currently present on the ServoHub.
     * Warnings are non-fatal errors.
     *
     * @return Signal containing a struct with each warning and their active
     * value
     */
    rev::util::Signal<Warnings> GetWarnings() const;

    /**
     * Get the sticky warnings that were present on the ServoHub at one point
     * since the sticky warnings were last cleared. Warnings are non-fatal
     * errors.
     *
     * <p>Sticky warnings can be cleared with ServoHub::clearFaults().
     *
     * @return Signal containing a struct with each warning and their sticky
     * value
     */
    rev::util::Signal<Warnings> GetStickyWarnings() const;

    /**
     * Clears all non-sticky faults.
     *
     * Sticky faults must be cleared by resetting the motor controller.
     */
    REVLibError ClearFaults();

    /**
     * @return Signal containing the voltage fed into the servo controller.
     */
    rev::util::Signal<double> GetDeviceVoltage() const;

    /**
     * @return Signal containing the servo controller's output current in Amps.
     */
    rev::util::Signal<double> GetDeviceCurrent() const;

    /**
     * @return Signal containing the voltage fed to the actual servos.
     */
    rev::util::Signal<double> GetServoVoltage() const;

    /**
     * Returns an object to control a specific servo channel.
     *
     * @param channelId The specific servo channel to get
     * @return The specified ServoChannel
     */
    ServoChannel& GetServoChannel(ServoChannel::ChannelId channelId);

    enum class Bank { kBank0_2 = 0, kBank3_5 = 1 };

    /**
     * Set the Pulse Period for servo channels 0-2 or servo channels 3-5.
     *
     * @param bank The bank of channels (0-2 or 3-5) to set
     * @param pulsePeriod_us The pulse period in microseconds
     * @return REVLibError::kOk if successful
     */
    REVLibError SetBankPulsePeriod(Bank bank, int pulsePeriod_us);

private:
    std::vector<ServoChannel> m_servoChannels;
};

}  // namespace rev::servohub
