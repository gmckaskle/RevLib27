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

#include "rev/REVLibDaemon.h"

#include <rev/StatusLoggerDriver.h>
#include <rev/driver/REVLibDriver.h>

#include <array>
#include <atomic>
#include <mutex>
#include <thread>
#include <vector>

// Replacement for wpi functions
#include "SafeThread.h"

#include "rev/CANServoHubFrames.h"
#include "rev/CANSparkFrames.h"
#include "rev/REVLibErrors.h"
#include "rev/REVUtils.h"

namespace {
// Note: This is dependent on __attribute__((PACKED)) and processor is
// little-endian

#ifdef _WIN32
#define PACKED
#pragma pack(push, 1)
#else
#define PACKED __attribute__((__packed__))
#endif

typedef struct PACKED {
    float setpoint;
    int16_t auxSetpoint;
    uint32_t pidSlot : 2;
    uint8_t arbFFUnits : 1;
    uint32_t rsvd0 : 5;
    uint32_t rsvd1 : 8;
} frc_dataframe_setpoint_out_t;

typedef struct PACKED {
    uint16_t updateRate;
    uint8_t statusSelect0;
    uint8_t statusSelect1;
    uint8_t statusSelect2;
    uint8_t statusSelect3;
} frc_dataframe_statusConfig_out_t;

typedef struct PACKED {
    uint32_t parameter;
    uint8_t parameterType;
} frc_dataframe_setParam_out_t;

typedef struct PACKED {
    uint32_t parameter0;
    uint8_t parameterType;
    uint8_t parameterResponse;
} frc_dataframe_getParam_in_t;

typedef struct PACKED {
    uint8_t magicNum0;
    uint8_t magicNum1;
} frc_dataframe_burnFlash_out_t;

typedef struct PACKED {
    uint32_t followerID;
    uint32_t followerCfg;
} frc_dataframe_follower_out_t;

typedef struct PACKED {
    uint8_t firmwareMajor;
    uint8_t firmwareMinor;
    uint16_t firmwareBuild;
    uint8_t debugBuild;
    uint8_t hardwareRevision;
} frc_dataframe_firmware_in_t;

typedef struct PACKED {
    int16_t appliedOutput;
    uint16_t faults;
    uint16_t stickyFaults;
    uint8_t sensorInv : 1;
    uint8_t setpointInv : 1;
    uint8_t lock : 2;
    uint8_t mtrType : 1;
    uint8_t isFollower : 1;
    uint8_t roboRIO : 1;
    uint8_t rsvd0 : 1;
} frc_dataframe_status0_in_t;

typedef struct PACKED {
    int32_t sensorVel;
    uint8_t mtrTemp;
    uint16_t mtrVoltage : 12;
    uint16_t mtrCurrent : 12;
} frc_dataframe_status1_in_t;

typedef struct PACKED {
    int32_t sensorPos;
    int16_t partialPosition;
    int16_t unused;
} frc_dataframe_status2_in_t;

typedef struct PACKED {
    uint32_t analogVoltage : 10;
    int32_t analogVel : 22;
    int32_t analogPos;
} frc_dataframe_status3_in_t;

typedef struct PACKED {
    float altEncoderVelocity;
    float altEncoderPosition;
} frc_dataframe_status4_in_t;

typedef struct PACKED {
    float dutyCyclePosition;
    uint16_t dutyCycleAbsoluteValue;
    uint8_t rsvd0;
    uint8_t dutyCycleStatus;
} frc_dataframe_status5_in_t;

typedef struct PACKED {
    float dutyCycleVelocity;
    uint16_t dutyCycleFrequency;
} frc_dataframe_status6_in_t;

typedef struct PACKED {
    int32_t iAccum;
} frc_dataframe_status7_in_t;

typedef struct PACKED {
    uint16_t userParam0ID;
    uint16_t userParam1ID;
    uint16_t userParam2ID;
    uint16_t userParam3ID;
} frc_dataframe_userStatus_in_t;

typedef struct PACKED {
    uint16_t DRVStat0;
    uint16_t DRVStat1;
    uint16_t faults;
    uint16_t stickyFaults;
} frc_dataframe_drvStatus_in_t;

typedef union {
    frc_dataframe_setpoint_out_t setpointOut;
    frc_dataframe_statusConfig_out_t statusConfigOut;
    frc_dataframe_setParam_out_t setParamOut;
    frc_dataframe_burnFlash_out_t burnFlashOut;
    frc_dataframe_follower_out_t followerOut;
    frc_dataframe_firmware_in_t firmwareIn;
    frc_dataframe_status0_in_t status0In;
    frc_dataframe_status1_in_t status1In;
    frc_dataframe_status2_in_t status2In;
    frc_dataframe_status3_in_t status3In;
    frc_dataframe_status4_in_t status4In;
    frc_dataframe_status5_in_t status5In;
    frc_dataframe_status6_in_t status6In;
    frc_dataframe_status7_in_t status7In;
    frc_dataframe_userStatus_in_t userStatusIn;
    frc_dataframe_drvStatus_in_t drvStatusIn;
    frc_dataframe_getParam_in_t getParamIn;
    uint8_t data[8];
} frc_dataframe_t;

}  // namespace

namespace {
constexpr uint64_t kErrorReportInterval_us{2000000};
constexpr uint64_t kThreadRate_us{25000};

class REVLibDaemon : public rev::SafeThread {
public:
    REVLibDaemon() = default;

    void RegisterAsyncCall(std::function<void()> call) {
        std::lock_guard<std::mutex> lock(asyncCallsMutex);
        registeredAsyncCalls.push_back(call);
    }

private:
    uint64_t startTime{};
    std::vector<std::function<void()>> registeredAsyncCalls{};
    std::mutex asyncCallsMutex;

    void Main() override {
        uint64_t currTime{};
        uint64_t lastErrorTime{};

        uint32_t timer = getREVLibDriver()->allocatePreciseTimer();
        startTime = getREVLibDriver()->getPreciseTime();

        while (m_active) {
            // Flush error codes on occasion
            currTime = getREVLibDriver()->getPreciseTime();
            if (c_REVLib_ErrorSize() != 0 &&
                ((currTime - lastErrorTime) > kErrorReportInterval_us)) {
                c_REVLib_FlushErrors();
                lastErrorTime = currTime;
                currTime = getREVLibDriver()->getPreciseTime();
            }

            // Process async CAN calls
            std::vector<std::function<void()>> asyncCalls;
            {
                std::lock_guard<std::mutex> lock(asyncCallsMutex);
                asyncCalls = registeredAsyncCalls;
                registeredAsyncCalls.clear();
            }
            for (const auto& call : asyncCalls) {
                call();
            }

            StatusLoggerDriver_read();

            getREVLibDriver()->preciseDelayMicroseconds(
                timer, static_cast<uint64_t>(
                           (kThreadRate_us - (currTime - startTime))));
            startTime = getREVLibDriver()->getPreciseTime();
        }

        getREVLibDriver()->closePreciseTimer(timer);
    }
};

}  // namespace

static rev::SafeThreadOwner<REVLibDaemon>& REVLibThread() {
    static rev::SafeThreadOwner<REVLibDaemon> inst;
    return inst;
}

// TODO(jan): Refactor this to be a part of CANSparkDriver

extern "C" {

namespace {
std::mutex devicesMutex;
std::atomic_uint initialized{0};

// TODO: (dave) support all busses
constexpr int TODO_BUS_ID{0};

void c_REVLib_InitDaemon(void) {
    // Hacky way to make sure all periodic messages are killed
    // before enabling the daemon.
    static constexpr std::array<uint32_t, 7> messageIDs = {
        SPARK_DUTY_CYCLE_SETPOINT_FRAME_ID,
        SPARK_VELOCITY_SETPOINT_FRAME_ID,
        SPARK_POSITION_SETPOINT_FRAME_ID,
        SPARK_VOLTAGE_SETPOINT_FRAME_ID,
        SPARK_CURRENT_SETPOINT_FRAME_ID,
        SERVO_HUB_SET_SERVO_0_TO_2_PULSE_WIDTHS_FRAME_ID,
        SERVO_HUB_SET_SERVO_3_TO_5_PULSE_WIDTHS_FRAME_ID,
    };

    for (auto deviceID = 0; deviceID < 63; deviceID++) {
        int32_t status{};
        for (auto messageID : messageIDs) {
            getREVLibDriver()->writeCanFrame(TODO_BUS_ID, messageID | deviceID,
                                             nullptr, 0, SEND_PERIOD_STOP,
                                             &status);
        }
        // After disabled repeating message, before daemon is enabled,
        // set 0 DC
        frc_dataframe_t frame;
        frame.setpointOut.auxSetpoint = 0;
        frame.setpointOut.setpoint = 0;
        frame.setpointOut.pidSlot = 0;
        getREVLibDriver()->writeCanFrame(
            TODO_BUS_ID, SPARK_DUTY_CYCLE_SETPOINT_FRAME_ID | deviceID,
            frame.data, 8, SEND_PERIOD_NO_REPEAT, &status);
    }

    if (StatusLoggerDriver_getAutoLogging()) {
        StatusLoggerDriver_start();
    }
}

}  // namespace

uint32_t c_REVLib_RunDaemon(void) {
    std::lock_guard<std::mutex> lock(devicesMutex);
    if (!initialized) {
        // TODO: replace with std::call_once???
        c_REVLib_InitDaemon();
        REVLibThread().Start();
    }
    ++initialized;
    return initialized;
}

void c_REVLib_StopDaemon(void) {
    std::lock_guard<std::mutex> lock(devicesMutex);
    if (initialized) {
        initialized--;

        if (initialized == 0) {
            REVLibThread().Join();
        }
    }
}

void c_REVLib_RegisterAsyncCall(std::function<void()> call) {
    REVLibThread().GetThreadSharedPtr()->RegisterAsyncCall(call);
}
}  // extern "C"
