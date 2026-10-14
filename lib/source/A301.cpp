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

#include "first/A301.h"

#include <stdexcept>
#include <string>

#include "first/CANA301Driver.h"

using namespace first::a301;

using namespace rev::util;

namespace first::a301 {
namespace internal {

// Low-level A301 functions
class A301 {
public:
    A301(int busId, int deviceId) : m_busId{busId}, m_deviceId{deviceId} {
        int actualDeviceId{};
        if (c_A301_RegisterId(busId, deviceId, &actualDeviceId) ==
            c_REVLibError_DuplicateCANId) {
            const bool isDetected{deviceId != actualDeviceId};

            throw std::runtime_error(
                std::format("A FIRST A301 instance has already been created "
                            "with this {}device ID: {} on Bus: {}",
                            isDetected ? "auto-detected " : "",
                            isDetected ? actualDeviceId : deviceId, busId));
        }
        m_deviceId = actualDeviceId;

        c_REVLib_ErrorCode status;
        m_a301Handle = c_A301_Create(busId, deviceId, &status);

        if (status == c_REVLibError_CantFindFirmware) {
            // Don't throw exception when no firmware is found. It's possible
            // the device is disconnected and we don't want to stop the program
            // if that is the case.
        } else if (status == c_REVLibError_FirmwareTooOld) {
            throw std::runtime_error(std::format(
                "The firmware version of Bus {} A301 #{} is too old and must "
                "be updated to 27.0.0-prerelease-11 or later.",
                busId, deviceId));
        } else if (status == c_REVLibError_FirmwareTooNew) {
            throw std::runtime_error(std::format(
                "The firmware version of Bus {} A301 #{} is too new for this "
                "version of REVLib",
                busId, deviceId));
        }
    }

    ~A301() {
        c_A301_Close(m_a301Handle);
        c_A301_Destroy(m_a301Handle);
    }

    enum class ControlType {
        kDutyCycle = 0,
        kVelocity = 1,
        kVoltage = 2,
        kRelativePosition = 3,
        kAbsolutePosition = 4,
        kCurrent = 5,
    };

    static constexpr double kDefaultPositionSpeed{0.0};  // maximum speed
    /**
     * Set the controller setpoint based on the selected control mode.
     *
     * @param setpoint The setpoint to set depending on the control mode.
     * For:
     *   - Duty Cycle (-1.0 to 1.0)
     *   - Velocity Control: Velocity (RPM)
     *   - Voltage Control: Voltage (volts) (-12.0 to 12.0)
     *   - Relative Position Control: Position (Rotations)
     *   - Absolute Position Control: Position (-0.5 to 0.5)
     *   - Current Control: Current (Amps).
     *
     * @param ctrl Is the control type
     * @param positionSpeed The speed to approach the position at
     * (position control only)
     * @return REVLibError::kOk if successful
     */
    rev::REVLibError SetSetpoint(double setpoint, ControlType ctrl,
                                 double positionSpeed = kDefaultPositionSpeed) {
        const auto status = c_A301_SetpointCommand(
            m_a301Handle, setpoint, static_cast<c_A301_ControlType>(ctrl),
            positionSpeed);

        return static_cast<rev::REVLibError>(status);
    }

    struct PeriodicStatus0 {
        double appliedOutput;
        double voltage;
        double motorCurrent;
        uint8_t motorTemperature;
        bool primaryHeartbeatLock;
        uint8_t gearboxRPM;
        uint64_t timestamp;
    };

    Signal<PeriodicStatus0> GetPeriodicStatus0() const {
        c_A301_PeriodicStatus0 cStatus0;
        auto revLibError = static_cast<rev::REVLibError>(
            c_A301_GetPeriodicStatus0(m_a301Handle, &cStatus0));

        const PeriodicStatus0 status0{
            .appliedOutput = cStatus0.appliedOutput,
            .voltage = cStatus0.voltage,
            .motorCurrent = cStatus0.current,
            .motorTemperature = cStatus0.motorTemperature,
            .primaryHeartbeatLock = cStatus0.primaryHeartbeatLock != 0,
            .gearboxRPM = cStatus0.gearboxRPM,
            .timestamp = cStatus0.timestamp,
        };

        return Signal(status0, revLibError, status0.timestamp);
    }

    struct PeriodicStatus1 {
        bool otherFault;
        bool motorTypeFault;
        bool sensorFault;
        bool canFault;
        bool temperatureFault;
        bool drvFault;
        bool escEepromFault;
        bool firmwareFault;
        bool motorStartupFault;

        bool overvoltageWarning;
        bool motorLoopSpeedWarning;
        bool brownoutWarning;
        bool overcurrentWarning;
        bool escEepromWarning;
        bool extEepromWarning;
        bool sensorWarning;
        bool stallWarning;
        bool hasResetWarning;
        bool otherWarning;

        bool otherStickyFault;
        bool motorTypeStickyFault;
        bool sensorStickyFault;
        bool canStickyFault;
        bool temperatureStickyFault;
        bool drvStickyFault;
        bool escEepromStickyFault;
        bool firmwareStickyFault;
        bool motorStartupStickyFault;

        bool overvoltageStickyWarning;
        bool motorLoopSpeedStickyWarning;
        bool brownoutStickyWarning;
        bool overcurrentStickyWarning;
        bool escEepromStickyWarning;
        bool extEepromStickyWarning;
        bool sensorStickyWarning;
        bool stallStickyWarning;
        bool hasResetStickyWarning;
        bool otherStickyWarning;
        uint64_t timestamp;
    };

    Signal<PeriodicStatus1> GetPeriodicStatus1() const {
        c_A301_PeriodicStatus1 cStatus1;
        auto revLibError = static_cast<rev::REVLibError>(
            c_A301_GetPeriodicStatus1(m_a301Handle, &cStatus1));

        const PeriodicStatus1 status1 = {
            .otherFault = cStatus1.otherFault != 0,
            // TODO: Remove? Doesn't make sense for A301
            .motorTypeFault = cStatus1.motorTypeFault != 0,
            .sensorFault = cStatus1.sensorFault != 0,
            .canFault = cStatus1.canFault != 0,
            .temperatureFault = cStatus1.temperatureFault != 0,
            .drvFault = cStatus1.drvFault != 0,
            .escEepromFault = cStatus1.escEepromFault != 0,
            .firmwareFault = cStatus1.firmwareFault != 0,
            .motorStartupFault = cStatus1.motorStartupFault != 0,

            .overvoltageWarning = cStatus1.overvoltageWarning != 0,
            .motorLoopSpeedWarning = cStatus1.motorLoopSpeedWarning != 0,
            .brownoutWarning = cStatus1.brownoutWarning != 0,
            .overcurrentWarning = cStatus1.overcurrentWarning != 0,
            .escEepromWarning = cStatus1.escEepromWarning != 0,
            .extEepromWarning = cStatus1.extEepromWarning != 0,
            .sensorWarning = cStatus1.sensorWarning != 0,
            .stallWarning = cStatus1.stallWarning != 0,
            .hasResetWarning = cStatus1.hasResetWarning != 0,
            .otherWarning = cStatus1.otherWarning != 0,

            .otherStickyFault = cStatus1.otherStickyFault != 0,
            .motorTypeStickyFault = cStatus1.motorTypeStickyFault != 0,
            .sensorStickyFault = cStatus1.sensorStickyFault != 0,
            .canStickyFault = cStatus1.canStickyFault != 0,
            .temperatureStickyFault = cStatus1.temperatureStickyFault != 0,
            .drvStickyFault = cStatus1.drvStickyFault != 0,
            .escEepromStickyFault = cStatus1.escEepromStickyFault != 0,
            .firmwareStickyFault = cStatus1.firmwareStickyFault != 0,
            .motorStartupStickyFault = cStatus1.motorStartupStickyFault != 0,

            .overvoltageStickyWarning = cStatus1.overvoltageStickyWarning != 0,
            .motorLoopSpeedStickyWarning =
                cStatus1.motorLoopSpeedStickyWarning != 0,
            .brownoutStickyWarning = cStatus1.brownoutStickyWarning != 0,
            .overcurrentStickyWarning = cStatus1.overcurrentStickyWarning != 0,
            .escEepromStickyWarning = cStatus1.escEepromStickyWarning != 0,
            .extEepromStickyWarning = cStatus1.extEepromStickyWarning != 0,
            .sensorStickyWarning = cStatus1.sensorStickyWarning != 0,
            .stallStickyWarning = cStatus1.stallStickyWarning != 0,
            .hasResetStickyWarning = cStatus1.hasResetStickyWarning != 0,
            .otherStickyWarning = cStatus1.otherStickyWarning != 0,

            // TODO: uncomment when/if followers are added
            //.isFollower = cStatus1.isFollower != 0,
            .timestamp = cStatus1.timestamp,
        };

        return Signal(status1, revLibError, status1.timestamp);
    }

    struct PeriodicStatus2 {
        double encoderVelocity;
        double relativeEncoderPosition;
        uint64_t timestamp;
    };

    Signal<PeriodicStatus2> GetPeriodicStatus2() const {
        c_A301_PeriodicStatus2 cStatus2;
        auto revLibError = static_cast<rev::REVLibError>(
            c_A301_GetPeriodicStatus2(m_a301Handle, &cStatus2));

        const PeriodicStatus2 status2 = {
            .encoderVelocity = cStatus2.encoderVelocity,
            .relativeEncoderPosition = cStatus2.relativeEncoderPosition,
            .timestamp = cStatus2.timestamp,
        };

        return Signal(status2, revLibError, status2.timestamp);
    }

    struct PeriodicStatus3 {
        double absoluteEncoderPosition;
        uint64_t timestamp;
    };

    Signal<PeriodicStatus3> GetPeriodicStatus3() const {
        c_A301_PeriodicStatus3 cStatus3;
        auto revLibError = static_cast<rev::REVLibError>(
            c_A301_GetPeriodicStatus3(m_a301Handle, &cStatus3));

        const PeriodicStatus3 status3 = {
            .absoluteEncoderPosition = cStatus3.absoluteEncoderPosition,
            .timestamp = cStatus3.timestamp,
        };

        return Signal(status3, revLibError, status3.timestamp);
    }

    c_A301_handle m_a301Handle{};

    int m_busId;
    int m_deviceId;

    // Only used for MotorController get/setDutyCycle() API
    double m_setpoint = 0.0;
};

}  // namespace internal

}  // namespace first::a301

namespace {

constexpr uint16_t BitMask(int offset) {
    return static_cast<uint16_t>(0x1u << static_cast<uint16_t>(offset));
}

}  // namespace

A301::Faults::Faults(uint16_t faults) {
    rawBits = faults;
    other = (faults & BitMask(c_A301_Fault_kOther)) != 0;
    // TODO: Remove? Doesn't make sense for A301
    // motorType = (faults & BitMask(c_A301_Fault_kMotorType)) != 0;
    sensor = (faults & BitMask(c_A301_Fault_kSensor)) != 0;
    can = (faults & BitMask(c_A301_Fault_kCan)) != 0;
    temperature = (faults & BitMask(c_A301_Fault_kTemperature)) != 0;
    gateDriver = (faults & BitMask(c_A301_Fault_kDrv)) != 0;
    escEeprom = (faults & BitMask(c_A301_Fault_kEscEeprom)) != 0;
    firmware = (faults & BitMask(c_A301_Fault_kFirmware)) != 0;
    motorStartup = (faults & BitMask(c_A301_Fault_kMotorStartup)) != 0;
}

A301::Warnings::Warnings(uint16_t warnings) {
    rawBits = warnings;
    overvoltage = (warnings & BitMask(c_A301_Warning_kOvervoltage)) != 0;
    motorLoopSpeed = (warnings & BitMask(c_A301_Warning_kMotorLoopSpeed)) != 0;
    brownout = (warnings & BitMask(c_A301_Warning_kBrownout)) != 0;
    overcurrent = (warnings & BitMask(c_A301_Warning_kOvercurrent)) != 0;
    escEeprom = (warnings & BitMask(c_A301_Warning_kEscEeprom)) != 0;
    extEeprom = (warnings & BitMask(c_A301_Warning_kExtEeprom)) != 0;
    sensor = (warnings & BitMask(c_A301_Warning_kSensor)) != 0;
    stall = (warnings & BitMask(c_A301_Warning_kStall)) != 0;
    hasReset = (warnings & BitMask(c_A301_Warning_kHasReset)) != 0;
    other = (warnings & BitMask(c_A301_Warning_kOther)) != 0;
}

A301::A301(wpi::CANPort canPort, int deviceId)
    : p(std::make_unique<internal::A301>(static_cast<int>(canPort), deviceId)) {
}

A301::A301(wpi::CANPort canPort) : A301(canPort, defaultDeviceId) {}

A301::~A301() = default;

wpi::CANPort A301::GetCanPort() const {
    return static_cast<wpi::CANPort>(p->m_busId);
}

int A301::GetDeviceId() const { return p->m_deviceId; }

A301::FirmwareVersion A301::GetFirmwareVersion() const {
    c_A301_FirmwareVersion fwVersion;

    c_A301_GetFirmwareVersion(p->m_a301Handle, &fwVersion);

    return {.major = fwVersion.major,
            .minor = fwVersion.minor,
            .patch = fwVersion.patch,
            .prerelease = fwVersion.prerelease};
}

std::string A301::GetFirmwareString() const {
    c_A301_FirmwareVersion fwVersion;

    c_A301_GetFirmwareVersion(p->m_a301Handle, &fwVersion);

    if (fwVersion.prerelease != 0) {
        return std::format("v{}.{}.{} {} Debug Build", fwVersion.major,
                           fwVersion.minor, fwVersion.patch,
                           fwVersion.prerelease);
    } else {
        return std::format("v{}.{}.{}", fwVersion.major, fwVersion.minor,
                           fwVersion.patch);
    }
}

Signal<bool> A301::HasActiveFault() const {
    return GetFaults().Map(
        [](const auto& faults) { return faults.rawBits != 0; });
}

Signal<bool> A301::HasStickyFault() const {
    return GetStickyFaults().Map(
        [](const auto& faults) { return faults.rawBits != 0; });
}

Signal<bool> A301::HasActiveWarning() const {
    return GetWarnings().Map(
        [](const auto& warnings) { return warnings.rawBits != 0; });
}

Signal<bool> A301::HasStickyWarning() const {
    return GetStickyWarnings().Map(
        [](const auto& warnings) { return warnings.rawBits != 0; });
}

Signal<A301::Faults> A301::GetFaults() const {
    return p->GetPeriodicStatus1().Map([](const auto& s1) {
        const uint16_t faults =
            s1.otherFault << static_cast<uint8_t>(c_A301_Fault_kOther) |
            s1.motorTypeFault << static_cast<uint8_t>(c_A301_Fault_kMotorType) |
            s1.sensorFault << static_cast<uint8_t>(c_A301_Fault_kSensor) |
            s1.canFault << static_cast<uint8_t>(c_A301_Fault_kCan) |
            s1.temperatureFault
                << static_cast<uint8_t>(c_A301_Fault_kTemperature) |
            s1.drvFault << static_cast<uint8_t>(c_A301_Fault_kDrv) |
            s1.escEepromFault << static_cast<uint8_t>(c_A301_Fault_kEscEeprom) |
            s1.firmwareFault << static_cast<uint8_t>(c_A301_Fault_kFirmware);
        // TODO: uncomment when A301 adds
        //| s1.motorStartupFault <<
        // static_cast<uint8_t>(c_A301_Fault_kMotorStartup);

        return Faults(faults);
    });
}

Signal<A301::Faults> A301::GetStickyFaults() const {
    return p->GetPeriodicStatus1().Map([](const auto& s1) {
        const uint16_t faults =
            s1.otherStickyFault << static_cast<uint8_t>(c_A301_Fault_kOther) |
            s1.motorTypeStickyFault
                << static_cast<uint8_t>(c_A301_Fault_kMotorType) |
            s1.sensorStickyFault << static_cast<uint8_t>(c_A301_Fault_kSensor) |
            s1.canStickyFault << static_cast<uint8_t>(c_A301_Fault_kCan) |
            s1.temperatureStickyFault
                << static_cast<uint8_t>(c_A301_Fault_kTemperature) |
            s1.drvStickyFault << static_cast<uint8_t>(c_A301_Fault_kDrv) |
            s1.escEepromStickyFault
                << static_cast<uint8_t>(c_A301_Fault_kEscEeprom) |
            s1.firmwareStickyFault
                << static_cast<uint8_t>(c_A301_Fault_kFirmware);
        // TODO: uncomment when A301 adds
        //| s1.motorStartupStickyFault <<
        // static_cast<uint8_t>(c_A301_Fault_kMotorStartup);

        return Faults(faults);
    });
}

Signal<A301::Warnings> A301::GetWarnings() const {
    return p->GetPeriodicStatus1().Map([](const auto& s1) {
        const uint16_t warnings =
            s1.brownoutWarning
                << static_cast<uint8_t>(c_A301_Warning_kBrownout) |
            s1.overcurrentWarning
                << static_cast<uint8_t>(c_A301_Warning_kOvercurrent) |
            s1.escEepromWarning
                << static_cast<uint8_t>(c_A301_Warning_kEscEeprom) |
            s1.extEepromWarning
                << static_cast<uint8_t>(c_A301_Warning_kExtEeprom) |
            s1.sensorWarning << static_cast<uint8_t>(c_A301_Warning_kSensor) |
            s1.stallWarning << static_cast<uint8_t>(c_A301_Warning_kStall) |
            s1.hasResetWarning
                << static_cast<uint8_t>(c_A301_Warning_kHasReset) |
            s1.otherWarning << static_cast<uint8_t>(c_A301_Warning_kOther);
        // TODO: uncomment when A301 adds
        //| s1.overVoltage << static_cast<uint8_t>(c_A301_Warning_kOverVoltage)
        //| s1.motorLoopSpeed <<
        // static_cast<uint8_t>(c_A301_Warning_kMotorLooppeed);

        return Warnings(warnings);
    });
}

Signal<A301::Warnings> A301::GetStickyWarnings() const {
    return p->GetPeriodicStatus1().Map([](const auto& s1) {
        const uint16_t warnings =
            s1.brownoutStickyWarning
                << static_cast<uint8_t>(c_A301_Warning_kBrownout) |
            s1.overcurrentStickyWarning
                << static_cast<uint8_t>(c_A301_Warning_kOvercurrent) |
            s1.escEepromStickyWarning
                << static_cast<uint8_t>(c_A301_Warning_kEscEeprom) |
            s1.extEepromStickyWarning
                << static_cast<uint8_t>(c_A301_Warning_kExtEeprom) |
            s1.sensorStickyWarning
                << static_cast<uint8_t>(c_A301_Warning_kSensor) |
            s1.stallStickyWarning
                << static_cast<uint8_t>(c_A301_Warning_kStall) |
            s1.hasResetStickyWarning
                << static_cast<uint8_t>(c_A301_Warning_kHasReset) |
            s1.otherStickyWarning
                << static_cast<uint8_t>(c_A301_Warning_kOther);
        // TODO: uncomment when A301 adds
        //| s1.overVoltage << static_cast<uint8_t>(c_A301_Warning_kOverVoltage)
        //| s1.motorLoopSpeed <<
        // static_cast<uint8_t>(c_A301_Warning_kMotorLooppeed);

        return Warnings(warnings);
    });
}

Signal<double> A301::GetBusVoltage() const {
    return p->GetPeriodicStatus0().Map(
        [](const auto& s0) { return static_cast<double>(s0.voltage); });
}

Signal<double> A301::GetAppliedOutput() const {
    return p->GetPeriodicStatus0().Map(
        [](const auto& s0) { return static_cast<double>(s0.appliedOutput); });
}

Signal<double> A301::GetMotorCurrent() const {
    return p->GetPeriodicStatus0().Map(
        [](const auto& s0) { return static_cast<double>(s0.motorCurrent); });
}

Signal<double> A301::GetMotorTemperature() const {
    return p->GetPeriodicStatus0().Map([](const auto& s0) {
        return static_cast<double>(s0.motorTemperature);
    });
}

Signal<A301::GearboxRPM> A301::GetGearboxRPM() const {
    return p->GetPeriodicStatus0().Map([](const auto& s0) {
        return [cRpm = s0.gearboxRPM]() {
            switch (cRpm) {
                case c_A301_GearboxRPM_Unknown:
                    [[fallthrough]];
                default:
                    return A301::GearboxRPM::kRPM_Unknown;
                case c_A301_GearboxRPM_215:
                    return A301::GearboxRPM::kRPM_215;
                case c_A301_GearboxRPM_500:
                    return A301::GearboxRPM::kRPM_500;
            }
        }();
    });
}

Signal<double> A301::GetRelativeEncoderPosition() const {
    return p->GetPeriodicStatus2().Map([](const auto& s2) {
        return static_cast<double>(s2.relativeEncoderPosition);
    });
}

Signal<double> A301::GetEncoderVelocity() const {
    return p->GetPeriodicStatus2().Map(
        [](const auto& s2) { return static_cast<double>(s2.encoderVelocity); });
}

Signal<double> A301::GetAbsoluteEncoderPosition() const {
    return p->GetPeriodicStatus3().Map([](const auto& s3) {
        return static_cast<double>(s3.absoluteEncoderPosition);
    });
}

rev::REVLibError A301::ClearFaults() {
    auto status = c_A301_ClearFaults(p->m_a301Handle);
    return static_cast<rev::REVLibError>(status);
}

rev::REVLibError A301::SetVelocity(double velocity) {
    return p->SetSetpoint(velocity, internal::A301::ControlType::kVelocity);
}

rev::REVLibError A301::SetRelativePosition(double position) {
    return p->SetSetpoint(position,
                          internal::A301::ControlType::kRelativePosition);
}

rev::REVLibError A301::SetRelativePositionWithSpeed(double position,
                                                    double speed) {
    return p->SetSetpoint(
        position, internal::A301::ControlType::kRelativePosition, speed);
}

rev::REVLibError A301::SetAbsolutePosition(double absPosition) {
    return p->SetSetpoint(absPosition,
                          internal::A301::ControlType::kAbsolutePosition);
}

rev::REVLibError A301::SetAbsolutePositionWithSpeed(double absPosition,
                                                    double speed) {
    return p->SetSetpoint(
        absPosition, internal::A301::ControlType::kAbsolutePosition, speed);
}

rev::REVLibError A301::SetIdleMode(IdleMode idleMode) {
    c_REVLib_ErrorCode status;
    status =
        c_A301_SetIdleMode(p->m_a301Handle, static_cast<uint8_t>(idleMode));
    return static_cast<rev::REVLibError>(status);
}

A301::IdleMode A301::GetIdleMode() const {
    uint8_t idleMode{};
    c_A301_GetIdleMode(p->m_a301Handle, &idleMode);
    return static_cast<IdleMode>(idleMode);
}

rev::REVLibError A301::EnableAbsolutePositionContinuousInput() {
    c_REVLib_ErrorCode status;
    status = c_A301_SetAbsolutePositionContinuousInput(p->m_a301Handle, true);
    return static_cast<rev::REVLibError>(status);
}

rev::REVLibError A301::DisableAbsolutePositionContinuousInput() {
    c_REVLib_ErrorCode status;
    status = c_A301_SetAbsolutePositionContinuousInput(p->m_a301Handle, false);
    return static_cast<rev::REVLibError>(status);
}

bool A301::IsAbsolutePositionContinuousInputEnabled() const {
    uint8_t enabled{};

    c_A301_GetAbsolutePositionContinuousInput(p->m_a301Handle, &enabled);

    return enabled != 0;
}

rev::REVLibError A301::SetCurrent(double current) {
    return p->SetSetpoint(current, internal::A301::ControlType::kCurrent);
}

rev::REVLibError A301::SetRelativeEncoderPosition(double position) {
    c_REVLib_ErrorCode status;
    status = c_A301_SetRelativeEncoderPosition(p->m_a301Handle, position);
    return static_cast<rev::REVLibError>(status);
}

rev::REVLibError A301::SetAbsoluteEncoderPosition(double position) {
    c_REVLib_ErrorCode status;
    status = c_A301_SetAbsoluteEncoderPosition(p->m_a301Handle, position);
    return static_cast<rev::REVLibError>(status);
}

rev::REVLibError A301::SetAbsoluteEncoderRangeOffset(double offset) {
    c_REVLib_ErrorCode status;
    status = c_A301_SetAbsoluteEncoderRangeOffset(p->m_a301Handle, offset);
    return static_cast<rev::REVLibError>(status);
}

double A301::GetAbsoluteEncoderRangeOffset() const {
    float offset;
    c_A301_GetAbsoluteEncoderRangeOffset(p->m_a301Handle, &offset);
    return offset;
}

void A301::SetThrottle(double throttle) {
    // Only for 'get' API
    p->m_setpoint = throttle;
    p->SetSetpoint(throttle, internal::A301::ControlType::kDutyCycle);
}

double A301::GetThrottle() const { return p->m_setpoint; }

void A301::SetVoltage(wpi::units::volt_t output) {
    double dOutput = wpi::units::unit_cast<double>(output);
    p->m_setpoint = dOutput / 12.0;
    p->SetSetpoint(dOutput, internal::A301::ControlType::kVoltage);
}

void A301::SetInverted(bool isInverted) {
    c_A301_SetInverted(p->m_a301Handle, isInverted);
}

bool A301::GetInverted() const {
    uint8_t inverted;
    c_A301_GetInverted(p->m_a301Handle, &inverted);
    return inverted ? true : false;
}

void A301::Disable() { SetThrottle(0.0); }

first::a301::A301& A301::FaultsPeriodMs(int period_ms) {
    c_A301_SetStatusFramePeriod(p->m_a301Handle, c_A301_kStatus1, period_ms);
    return *this;
}

int A301::GetFaultsPeriodMs() const {
    uint32_t period_ms;
    c_A301_GetStatusFramePeriod(p->m_a301Handle, c_A301_kStatus1, &period_ms);
    return period_ms;
}

first::a301::A301& A301::WarningsPeriodMs(int period_ms) {
    c_A301_SetStatusFramePeriod(p->m_a301Handle, c_A301_kStatus1, period_ms);
    return *this;
}

int A301::GetWarningsPeriodMs() const {
    uint32_t period_ms;
    c_A301_GetStatusFramePeriod(p->m_a301Handle, c_A301_kStatus1, &period_ms);
    return period_ms;
}

first::a301::A301& A301::BusVoltagePeriodMs(int period_ms) {
    c_A301_SetStatusFramePeriod(p->m_a301Handle, c_A301_kStatus0, period_ms);
    return *this;
}

int A301::GetBusVoltagePeriodMs() const {
    uint32_t period_ms;
    c_A301_GetStatusFramePeriod(p->m_a301Handle, c_A301_kStatus0, &period_ms);
    return period_ms;
}

first::a301::A301& A301::AppliedOutputPeriodMs(int period_ms) {
    c_A301_SetStatusFramePeriod(p->m_a301Handle, c_A301_kStatus0, period_ms);
    return *this;
}

int A301::GetAppliedOutputPeriodMs() const {
    uint32_t period_ms;
    c_A301_GetStatusFramePeriod(p->m_a301Handle, c_A301_kStatus0, &period_ms);
    return period_ms;
}

first::a301::A301& A301::MotorCurrentPeriodMs(int period_ms) {
    c_A301_SetStatusFramePeriod(p->m_a301Handle, c_A301_kStatus0, period_ms);
    return *this;
}

int A301::GetMotorCurrentPeriodMs() const {
    uint32_t period_ms;
    c_A301_GetStatusFramePeriod(p->m_a301Handle, c_A301_kStatus0, &period_ms);
    return period_ms;
}

first::a301::A301& A301::MotorTemperaturePeriodMs(int period_ms) {
    c_A301_SetStatusFramePeriod(p->m_a301Handle, c_A301_kStatus0, period_ms);
    return *this;
}

int A301::GetMotorTemperaturePeriodMs() const {
    uint32_t period_ms;
    c_A301_GetStatusFramePeriod(p->m_a301Handle, c_A301_kStatus0, &period_ms);
    return period_ms;
}

first::a301::A301& A301::RelativeEncoderPositionPeriodMs(int period_ms) {
    c_A301_SetStatusFramePeriod(p->m_a301Handle, c_A301_kStatus2, period_ms);
    return *this;
}

int A301::GetRelativeEncoderPositionPeriodMs() const {
    uint32_t period_ms;
    c_A301_GetStatusFramePeriod(p->m_a301Handle, c_A301_kStatus2, &period_ms);
    return period_ms;
}

first::a301::A301& A301::EncoderVelocityPeriodMs(int period_ms) {
    c_A301_SetStatusFramePeriod(p->m_a301Handle, c_A301_kStatus2, period_ms);
    return *this;
}

int A301::GetEncoderVelocityPeriodMs() const {
    uint32_t period_ms;
    c_A301_GetStatusFramePeriod(p->m_a301Handle, c_A301_kStatus2, &period_ms);
    return period_ms;
}

first::a301::A301& A301::AbsoluteEncoderPositionPeriodMs(int period_ms) {
    c_A301_SetStatusFramePeriod(p->m_a301Handle, c_A301_kStatus3, period_ms);
    return *this;
}

int A301::GetAbsoluteEncoderPositionPeriodMs() const {
    uint32_t period_ms;
    c_A301_GetStatusFramePeriod(p->m_a301Handle, c_A301_kStatus3, &period_ms);
    return period_ms;
}
