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

#include "rev/SparkSim.h"

#include <rev/driver/REVLibDriver.h>
#include <rev/sim/NoiseGenerator.h>

#include <cmath>
#include <cstring>
#include <memory>
#include <string>

#include "rev/CANSparkDriver.h"
#include "rev/config/SparkParameters.h"
#include "rev/sim/SparkAbsoluteEncoderSim.h"
#include "rev/sim/SparkAnalogSensorSim.h"
#include "rev/sim/SparkExternalEncoderSim.h"
#include "rev/sim/SparkFlexSim.h"
#include "rev/sim/SparkLimitSwitchSim.h"
#include "rev/sim/SparkMaxAlternateEncoderSim.h"
#include "rev/sim/SparkMaxSim.h"
#include "rev/sim/SparkRelativeEncoderSim.h"
#include "rev/sim/SparkSimFaultManager.h"

using namespace rev::spark;

SparkSim::SparkSim(SparkBase* spark, wpi::math::DCMotor* motor)
    : m_spark(spark), m_dcMotor(motor), m_velocityAverage(2, 0.016) {
    std::string deviceType = "UNKNOWN";
    if (spark->GetSparkModel() == SparkLowLevel::SparkModel::kSparkFlex) {
        deviceType = "SPARK Flex";
    } else if (spark->GetSparkModel() == SparkLowLevel::SparkModel::kSparkMax) {
        deviceType = "SPARK MAX";
    }

    m_deviceName = std::format("{} [{},{}]", deviceType,
                               static_cast<int>(spark->GetCanPort()),
                               spark->GetDeviceId());
    m_unknownDeviceTypeMessage = std::format(
        "[REVLib Simulation error] {}: Unknown device type", m_deviceName);

    wpi::sim::SimDeviceSim sparkSim(m_deviceName.c_str());
    m_appliedOutput = sparkSim.GetDouble("Applied Output");
    m_position = sparkSim.GetDouble("Position");
    m_velocity = sparkSim.GetDouble("Velocity");
    m_busVoltage = sparkSim.GetDouble("Bus Voltage");
    m_motorCurrent = sparkSim.GetDouble("Motor Current");
    m_setpoint = sparkSim.GetDouble("Setpoint");
    m_arbFF = sparkSim.GetDouble("Arbitrary Feedforward");
    m_closedLoopSlot = sparkSim.GetInt("Closed Loop Slot");
    m_arbFFUnits = sparkSim.GetInt("ArbFF Units");
    m_controlMode = sparkSim.GetInt("Control Mode");
}

double SparkSim::GetAppliedOutput() const { return m_appliedOutput.Get(); }

void SparkSim::SetAppliedOutput(double appliedOutput) {
    m_appliedOutput.Set(appliedOutput);
}

double SparkSim::GetSetpoint() const { return m_setpoint.Get(); }

ClosedLoopSlot SparkSim::GetClosedLoopSlot() const {
    switch (m_closedLoopSlot.Get()) {
        case 0:
            return ClosedLoopSlot::kSlot0;
        case 1:
            return ClosedLoopSlot::kSlot1;
        case 2:
            return ClosedLoopSlot::kSlot2;
        case 3:
            return ClosedLoopSlot::kSlot3;
        default:
            return ClosedLoopSlot::kSlot0;
    }
}

namespace {

enum SIM_Spark_ControlModes {
    kControlMode_DutyCycle = 0,
    kControlMode_Velocity = 1,
    kControlMode_Voltage = 2,
    kControlMode_Position = 3,
    kControlMode_Current = 4,
    kControlMode_MAXMotionPosition = 5,
    kControlMode_MAXMotionVelocity = 6
};

}  // namespace

void SparkSim::iterate(double velocity, double vbus, double dt) {
    if (vbus == 0) {
        auto details = std::format(
            "[REVLib Simulation error] {}: vbus provided to .iterate() cannot "
            "be zero",
            m_deviceName);
        getREVLibDriver()->sendError(0, details.c_str(), false);
        return;
    }

    double internalVelocity = NoiseGenerator::hallSensorVelocity(velocity);
    m_velocityAverage.put(internalVelocity, dt);
    internalVelocity = m_velocityAverage.get();

    m_velocity.Set(internalVelocity);

    double velocityRPM = velocity;
    m_position.Set(m_position.Get() + ((velocityRPM / 60) * dt));
    m_busVoltage.Set(vbus);

    float appliedOutput = 0.0;
    switch (m_controlMode.Get()) {
        case kControlMode_DutyCycle:
            appliedOutput = static_cast<float>(m_setpoint.Get());
            break;
        case kControlMode_Velocity:
            c_SIM_Spark_GetSimPIDOutput(
                static_cast<c_Spark_handle>(m_spark->m_sparkMaxHandle),
                &appliedOutput, static_cast<float>(m_setpoint.Get()),
                static_cast<float>(internalVelocity), static_cast<float>(dt));
            break;
        case kControlMode_Voltage:
            appliedOutput = m_setpoint.Get() / vbus;
            break;
        case kControlMode_Position:
            c_SIM_Spark_GetSimPIDOutput(
                static_cast<c_Spark_handle>(m_spark->m_sparkMaxHandle),
                &appliedOutput, static_cast<float>(m_setpoint.Get()),
                static_cast<float>(m_position.Get()), static_cast<float>(dt));
            break;
        case kControlMode_Current:
            c_SIM_Spark_GetSimPIDOutput(
                static_cast<c_Spark_handle>(m_spark->m_sparkMaxHandle),
                &appliedOutput, static_cast<float>(m_setpoint.Get()),
                static_cast<float>(m_motorCurrent.Get()),
                static_cast<float>(dt));
            break;
        case kControlMode_MAXMotionPosition:
            c_SIM_Spark_GetSimMAXMotionPositionControlOutput(
                static_cast<c_Spark_handle>(m_spark->m_sparkMaxHandle),
                &appliedOutput, static_cast<float>(dt));
            break;
        case kControlMode_MAXMotionVelocity:
            c_SIM_Spark_GetSimMAXMotionVelocityControlOutput(
                static_cast<c_Spark_handle>(m_spark->m_sparkMaxHandle),
                &appliedOutput, static_cast<float>(dt));
            break;
        default: {
            auto details = std::format(
                "[REVLib Simulation error] {}: Control mode out of bounds",
                m_deviceName);
            getREVLibDriver()->sendError(0, details.c_str(), false);
            break;
        }
    }

    if (m_arbFFUnits.Get() == 0) {
        appliedOutput += m_arbFF.Get() / vbus;
    } else {
        appliedOutput += m_arbFF.Get();
    }

    uint32_t voltageCompMode = 0;
    c_Spark_GetParameterUint32(
        static_cast<c_Spark_handle>(m_spark->m_sparkMaxHandle),
        c_Spark_kVoltageCompensationMode, &voltageCompMode);
    if (voltageCompMode == 2) {
        if (m_spark->GetSparkModel() == SparkLowLevel::SparkModel::kSparkFlex) {
            appliedOutput = (appliedOutput *
                             static_cast<SparkFlex*>(m_spark)
                                 ->configAccessor.GetVoltageCompensation()) /
                            vbus;
        } else if (m_spark->GetSparkModel() ==
                   SparkLowLevel::SparkModel::kSparkMax) {
            appliedOutput = (appliedOutput *
                             static_cast<SparkMax*>(m_spark)
                                 ->configAccessor.GetVoltageCompensation()) /
                            vbus;
        }
    }

    double maxOutput = runLimitLogic(true) ? 0 : 1;
    double minOutput = runLimitLogic(false) ? 0 : -1;
    appliedOutput = fminf(fmaxf(appliedOutput, minOutput), maxOutput);

    c_SIM_Spark_GetSimCurrentLimitOutput(
        static_cast<c_Spark_handle>(m_spark->m_sparkMaxHandle), &appliedOutput,
        appliedOutput, static_cast<float>(m_motorCurrent.Get()));

    m_motorCurrent.Set(wpi::units::unit_cast<double>(m_dcMotor->Current(
        wpi::units::radians_per_second_t(
            wpi::units::make_unit<wpi::units::revolutions_per_minute_t>(
                velocityRPM)),
        wpi::units::make_unit<wpi::units::volt_t>(appliedOutput * vbus))));

    SparkBase::Faults motorFaults = m_spark->GetFaults().Get();
    SparkBase::Faults motorStickyFaults = m_spark->GetStickyFaults().Get();
    if (motorFaults.can || motorStickyFaults.can || motorFaults.escEeprom ||
        motorStickyFaults.escEeprom || motorFaults.motorType ||
        motorStickyFaults.motorType || motorFaults.firmware ||
        motorStickyFaults.firmware || motorFaults.gateDriver ||
        motorStickyFaults.gateDriver || motorFaults.sensor ||
        motorStickyFaults.sensor || motorFaults.temperature ||
        motorStickyFaults.temperature || motorFaults.other ||
        motorStickyFaults.other) {
        appliedOutput = 0;
        auto details =
            std::format("[REVLib Simulation] {}: Device stopped due to fault",
                        m_deviceName);
        getREVLibDriver()->sendWarning(0, details.c_str(), false);
    }

    bool doEnable = false;
    if (m_enable == nullptr) {
        doEnable = getREVLibDriver()->areOutputsEnabled();
    } else {
        doEnable = *m_enable;
    }

    if (doEnable) {
        m_appliedOutput.Set(appliedOutput);
    } else {
        m_appliedOutput.Set(0.0);
        m_motorCurrent.Set(0.0);
    }

    FeedbackSensor selectedFeedbackSensor;
    if (m_spark->GetSparkModel() == SparkLowLevel::SparkModel::kSparkFlex) {
        selectedFeedbackSensor =
            static_cast<SparkFlex*>(m_spark)
                ->configAccessor.closedLoop.GetFeedbackSensor();
    } else if (m_spark->GetSparkModel() ==
               SparkLowLevel::SparkModel::kSparkMax) {
        selectedFeedbackSensor =
            static_cast<SparkMax*>(m_spark)
                ->configAccessor.closedLoop.GetFeedbackSensor();
    } else {
        selectedFeedbackSensor = FeedbackSensor::kNoSensor;
    }

    switch (selectedFeedbackSensor) {
        case FeedbackSensor::kPrimaryEncoder: {
            SparkRelativeEncoderSim relativeEncoderSim =
                GetRelativeEncoderSim();
            relativeEncoderSim.SetPosition(m_position.Get());
            relativeEncoderSim.SetVelocity(m_velocity.Get());
            break;
        }
        case FeedbackSensor::kAnalogSensor: {
            SparkAnalogSensorSim analogSensorSim = GetAnalogSensorSim();
            analogSensorSim.SetPosition(m_position.Get());
            analogSensorSim.SetVelocity(m_velocity.Get());
            break;
        }
        case FeedbackSensor::kAlternateOrExternalEncoder: {
            if (m_spark->GetSparkModel() ==
                SparkLowLevel::SparkModel::kSparkFlex) {
                SparkExternalEncoderSim externalEncoderSim =
                    SparkFlexSim(static_cast<SparkFlex*>(m_spark), m_dcMotor)
                        .GetExternalEncoderSim();
                externalEncoderSim.SetPosition(m_position.Get());
                externalEncoderSim.SetVelocity(m_velocity.Get());
            } else if (m_spark->GetSparkModel() ==
                       SparkLowLevel::SparkModel::kSparkMax) {
                SparkMaxAlternateEncoderSim alternateEncoderSim =
                    SparkMaxSim(static_cast<SparkMax*>(m_spark), m_dcMotor)
                        .GetAlternateEncoderSim();
                alternateEncoderSim.SetPosition(m_position.Get());
                alternateEncoderSim.SetVelocity(m_velocity.Get());
            }
            break;
        }
        case FeedbackSensor::kAbsoluteEncoder: {
            SparkAbsoluteEncoderSim absoluteEncoderSim =
                GetAbsoluteEncoderSim();
            absoluteEncoderSim.SetPosition(m_position.Get());
            absoluteEncoderSim.SetVelocity(m_velocity.Get());
            break;
        }
        default:
            break;
    }
}

double SparkSim::GetVelocity() const { return m_velocity.Get(); }

void SparkSim::SetVelocity(double velocity) { m_velocity.Set(velocity); }

double SparkSim::GetPosition() const { return m_position.Get(); }

void SparkSim::SetPosition(double position) { m_position.Set(position); }

double SparkSim::GetBusVoltage() const { return m_busVoltage.Get(); }

void SparkSim::SetBusVoltage(double voltage) { m_busVoltage.Set(voltage); }

double SparkSim::GetMotorCurrent() const { return m_motorCurrent.Get(); }

void SparkSim::SetMotorCurrent(double current) { m_motorCurrent.Set(current); }

void SparkSim::enable() { m_enable = std::make_unique<bool>(true); }

void SparkSim::disable() { m_enable = std::make_unique<bool>(false); }

void SparkSim::useDriverStationEnable() { m_enable.reset(); }

SparkRelativeEncoderSim SparkSim::GetRelativeEncoderSim() {
    c_SIM_Spark_CreateSimRelativeEncoder(
        static_cast<c_Spark_handle>(m_spark->m_sparkMaxHandle));

    if (m_spark->GetSparkModel() == SparkLowLevel::SparkModel::kSparkMax) {
        return SparkRelativeEncoderSim(static_cast<SparkMax*>(m_spark));
    } else if (m_spark->GetSparkModel() ==
               SparkLowLevel::SparkModel::kSparkFlex) {
        return SparkRelativeEncoderSim(static_cast<SparkFlex*>(m_spark));
    } else {
        getREVLibDriver()->sendError(0, m_unknownDeviceTypeMessage.c_str(),
                                     false);
        return SparkRelativeEncoderSim(static_cast<SparkMax*>(m_spark));
    }
}

SparkAbsoluteEncoderSim SparkSim::GetAbsoluteEncoderSim() {
    c_SIM_Spark_CreateSimAbsoluteEncoder(
        static_cast<c_Spark_handle>(m_spark->m_sparkMaxHandle));

    if (m_spark->GetSparkModel() == SparkLowLevel::SparkModel::kSparkMax) {
        return SparkAbsoluteEncoderSim(static_cast<SparkMax*>(m_spark));
    } else if (m_spark->GetSparkModel() ==
               SparkLowLevel::SparkModel::kSparkFlex) {
        return SparkAbsoluteEncoderSim(static_cast<SparkFlex*>(m_spark));
    } else {
        getREVLibDriver()->sendError(0, m_unknownDeviceTypeMessage.c_str(),
                                     false);
        return SparkAbsoluteEncoderSim(static_cast<SparkMax*>(m_spark));
    }
}

SparkAnalogSensorSim SparkSim::GetAnalogSensorSim() {
    c_SIM_Spark_CreateSimAnalogSensor(
        static_cast<c_Spark_handle>(m_spark->m_sparkMaxHandle));

    if (m_spark->GetSparkModel() == SparkLowLevel::SparkModel::kSparkMax) {
        return SparkAnalogSensorSim(static_cast<SparkMax*>(m_spark));
    } else if (m_spark->GetSparkModel() ==
               SparkLowLevel::SparkModel::kSparkFlex) {
        return SparkAnalogSensorSim(static_cast<SparkFlex*>(m_spark));
    } else {
        getREVLibDriver()->sendError(0, m_unknownDeviceTypeMessage.c_str(),
                                     false);
        return SparkAnalogSensorSim(static_cast<SparkMax*>(m_spark));
    }
}

SparkLimitSwitchSim SparkSim::GetForwardLimitSwitchSim() {
    c_SIM_Spark_CreateSimForwardLimitSwitch(
        static_cast<c_Spark_handle>(m_spark->m_sparkMaxHandle));
    if (m_spark->GetSparkModel() == SparkLowLevel::SparkModel::kSparkMax) {
        return SparkLimitSwitchSim(static_cast<SparkMax*>(m_spark), true);
    } else if (m_spark->GetSparkModel() ==
               SparkLowLevel::SparkModel::kSparkFlex) {
        return SparkLimitSwitchSim(static_cast<SparkFlex*>(m_spark), true);
    } else {
        getREVLibDriver()->sendError(0, m_unknownDeviceTypeMessage.c_str(),
                                     false);
        return SparkLimitSwitchSim(static_cast<SparkMax*>(m_spark), true);
    }
}

SparkLimitSwitchSim SparkSim::GetReverseLimitSwitchSim() {
    c_SIM_Spark_CreateSimReverseLimitSwitch(
        static_cast<c_Spark_handle>(m_spark->m_sparkMaxHandle));
    if (m_spark->GetSparkModel() == SparkLowLevel::SparkModel::kSparkMax) {
        return SparkLimitSwitchSim(static_cast<SparkMax*>(m_spark), false);
    } else if (m_spark->GetSparkModel() ==
               SparkLowLevel::SparkModel::kSparkFlex) {
        return SparkLimitSwitchSim(static_cast<SparkFlex*>(m_spark), false);
    } else {
        getREVLibDriver()->sendError(0, m_unknownDeviceTypeMessage.c_str(),
                                     false);
        return SparkLimitSwitchSim(static_cast<SparkMax*>(m_spark), false);
    }
}

SparkSimFaultManager SparkSim::GetFaultManager() {
    c_SIM_Spark_CreateSimFaultManager(
        static_cast<c_Spark_handle>(m_spark->m_sparkMaxHandle));
    if (m_spark->GetSparkModel() == SparkLowLevel::SparkModel::kSparkMax) {
        return SparkSimFaultManager(static_cast<SparkMax*>(m_spark));
    } else if (m_spark->GetSparkModel() ==
               SparkLowLevel::SparkModel::kSparkFlex) {
        return SparkSimFaultManager(static_cast<SparkFlex*>(m_spark));
    } else {
        getREVLibDriver()->sendError(0, m_unknownDeviceTypeMessage.c_str(),
                                     false);
        return SparkSimFaultManager(static_cast<SparkMax*>(m_spark));
    }
}

bool SparkSim::runLimitLogic(bool forward) {
    if (forward) {
        if (m_spark->GetSparkModel() == SparkLowLevel::SparkModel::kSparkFlex) {
            if (static_cast<SparkFlex*>(m_spark)
                    ->configAccessor.softLimit.GetForwardSoftLimitEnabled() &&
                static_cast<SparkFlex*>(m_spark)
                        ->configAccessor.softLimit.GetForwardSoftLimit() <
                    m_position.Get()) {
                return true;
            }
        } else if (m_spark->GetSparkModel() ==
                   SparkLowLevel::SparkModel::kSparkMax) {
            if (static_cast<SparkMax*>(m_spark)
                    ->configAccessor.softLimit.GetForwardSoftLimitEnabled() &&
                static_cast<SparkMax*>(m_spark)
                        ->configAccessor.softLimit.GetForwardSoftLimit() <
                    m_position.Get()) {
                return true;
            }
        }

        return (GetForwardLimitSwitchSim().GetEnabled() &&
                GetForwardLimitSwitchSim().GetPressed());
    } else {
        if (m_spark->GetSparkModel() == SparkLowLevel::SparkModel::kSparkFlex) {
            if (static_cast<SparkFlex*>(m_spark)
                    ->configAccessor.softLimit.GetReverseSoftLimitEnabled() &&
                static_cast<SparkFlex*>(m_spark)
                        ->configAccessor.softLimit.GetReverseSoftLimit() >
                    m_position.Get()) {
                return true;
            }
        } else if (m_spark->GetSparkModel() ==
                   SparkLowLevel::SparkModel::kSparkMax) {
            if (static_cast<SparkMax*>(m_spark)
                    ->configAccessor.softLimit.GetReverseSoftLimitEnabled() &&
                static_cast<SparkMax*>(m_spark)
                        ->configAccessor.softLimit.GetReverseSoftLimit() >
                    m_position.Get()) {
                return true;
            }
        }

        return (GetReverseLimitSwitchSim().GetEnabled() &&
                GetReverseLimitSwitchSim().GetPressed());
    }
}
