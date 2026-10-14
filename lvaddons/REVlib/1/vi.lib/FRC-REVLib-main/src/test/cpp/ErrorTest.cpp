/*
 * Copyright (c) 2018-2024 REV Robotics
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

// TODO(jan): Fix tests
#if 0
#include <chrono>
#include <iostream>
#include <thread>

#include "gtest/gtest.h"
#include "rev/CANSparkDriver.h"
#include "rev/REVLibError.h"
#include "rev/SparkMax.h"

/**
 * Takes a CAN ID and an error, and then generates tge error for the device with
 * that CAN ID. Confirms that generated error is accessible with GetLastError().
 */
void SpawnError(int id, rev::REVLibError error) {
    rev::SparkMax m_spark{id, rev::SparkMax::MotorType::kBrushless};

    c_Spark_GenerateError(id, static_cast<c_REVLib_ErrorCode>(error));
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    EXPECT_EQ(m_spark.GetLastError(), error);
}

/**
 * Checks that SetLastError and GetLastError are thread safe.
 *
 * Test generates a specific error, then spawns multiple threads that all
 * generate their own unique errors. Then check the original thread to make sure
 * that the other threads' errors don't overwrite the original.
 */
TEST(ErrorTest, SpawnErrorThreads) {
    rev::SparkMax m_spark{1, rev::SparkMax::MotorType::kBrushless};
    c_Spark_GenerateError(m_spark.GetDeviceId(),
                          c_REVLibError_SetpointOutOfRange);

    std::thread thread_a(SpawnError, 2, rev::REVLibError::kTimeout);
    std::thread thread_b(SpawnError, 3, rev::REVLibError::kError);
    std::thread thread_c(SpawnError, 4, rev::REVLibError::kHALError);
    std::thread thread_d(SpawnError, 5, rev::REVLibError::kFirmwareTooNew);
    std::thread thread_e(SpawnError, 6, rev::REVLibError::kNotImplemented);

    thread_a.join();
    thread_b.join();
    thread_c.join();
    thread_d.join();
    thread_e.join();

    EXPECT_EQ(m_spark.GetLastError(), rev::REVLibError::kSetpointOutOfRange);
}

TEST(ErrorTest, MultipleDevices) {
    rev::SparkMax m_rightFront{1, rev::SparkMax::MotorType::kBrushless};
    rev::SparkMax m_rightBack{2, rev::SparkMax::MotorType::kBrushless};
    rev::SparkMax m_leftFront{3, rev::SparkMax::MotorType::kBrushless};
    rev::SparkMax m_leftBack{4, rev::SparkMax::MotorType::kBrushless};

    c_Spark_GenerateError(m_rightFront.GetDeviceId(), c_REVLibError_CANTimeout);
    c_Spark_GenerateError(m_rightBack.GetDeviceId(),
                          c_REVLibError_FollowConfigMismatch);
    c_Spark_GenerateError(m_leftFront.GetDeviceId(),
                          c_REVLibError_FirmwareTooOld);
    c_Spark_GenerateError(m_leftBack.GetDeviceId(),
                          c_REVLibError_CantFindFirmware);

    std::thread thread_a(SpawnError, 5, rev::REVLibError::kError);
    std::thread thread_b(SpawnError, 6, rev::REVLibError::kHALError);
    std::thread thread_c(SpawnError, 7, rev::REVLibError::kFirmwareTooNew);

    thread_a.join();
    thread_b.join();
    thread_c.join();

    EXPECT_EQ(m_rightFront.GetLastError(), rev::REVLibError::kTimeout);
    EXPECT_EQ(m_rightBack.GetLastError(),
              rev::REVLibError::kFollowConfigMismatch);
    EXPECT_EQ(m_leftFront.GetLastError(), rev::REVLibError::kFirmwareTooOld);
    EXPECT_EQ(m_leftBack.GetLastError(), rev::REVLibError::kCantFindFirmware);
}
#endif
