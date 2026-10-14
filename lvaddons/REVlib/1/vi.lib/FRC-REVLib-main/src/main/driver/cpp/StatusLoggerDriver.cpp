// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

// Note: steps to add new devices are ordered by number in comments in the code
// (3 steps)

// SSH Command to retrieve REVLOG and WPILOG: scp
// admin@roborio-TEAM-frc.local:/path/to/your/logfile.revlog
// C:\path\on\your\computer

#include "rev/StatusLoggerDriver.h"

#include <rev/CANDetachedEncoderFrames.h>
#include <rev/CANServoHubFrames.h>
#include <rev/CANSparkFrames.h>
#include <rev/driver/REVLibDriver.h>

#include <algorithm>
#include <bit>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <string>
#include <vector>

extern "C" {

// 1. Add new variables for deviceType of new device from the FRC CAN Device
// specs
constexpr int32_t manufacturer = 5;  // HAL_CANManufacturer HAL_CAN_Man_kREV
constexpr int32_t sparkDeviceType =
    2;  // HAL_CANDeviceType HAL_CAN_Dev_kMotorController
constexpr int32_t encoderDeviceType =
    7;  // HAL_CANDeviceType HAL_CAN_Dev_kEncoder
constexpr int32_t servoHubDeviceType =
    12;  // HAL_CANDeviceType HAL_CAN_Dev_kServoController

constexpr int firmwareApiClass = 9;
constexpr int firmwareApiIndex = 8;
constexpr int firmwareApi =
    (firmwareApiClass & 0x3f) << 4 | (firmwareApiIndex & 0xf);
constexpr int periodicApiClass = 46;

constexpr int firmwareMessageIdMask = 0x1fffffc0;
constexpr int periodicMessageIdMask = 0x1ffffc00;

constexpr int generateFirmwareMessageId(int32_t deviceType) {
    return ((deviceType & 0x1f) << 24) | ((manufacturer & 0xff) << 16) |
           ((firmwareApi & 0x3ff) << 6);
}
constexpr int generatePeriodicMessageId(int32_t deviceType) {
    return ((deviceType & 0x1f) << 24) | ((manufacturer & 0xff) << 16) |
           ((periodicApiClass & 0x3f) << 10);
}

constexpr int sparkFirmwareMessageId =
    generateFirmwareMessageId(sparkDeviceType);
constexpr int sparkPeriodicMessageId =
    generatePeriodicMessageId(sparkDeviceType);

constexpr int encoderFirmwareMessageId =
    generateFirmwareMessageId(encoderDeviceType);
constexpr int encoderPeriodicMessageId =
    generatePeriodicMessageId(encoderDeviceType);

constexpr int servoHubFirmwareMessageId =
    generateFirmwareMessageId(servoHubDeviceType);
constexpr int servoHubPeriodicMessageId =
    generatePeriodicMessageId(servoHubDeviceType);

// 2. Add new variables for firmwareMessageId and periodicMessageId for new
// device given the specs. Manufacturer and api should be the same, but device
// type will change.

constexpr int persistentMessageSize = 10;
constexpr int periodicMessageSize = 16;
constexpr int maxPersistentMessages = 200;
constexpr int maxPeriodicMessages = 500;
constexpr int persistentSize =
    4 + (persistentMessageSize * maxPersistentMessages);
constexpr int periodicSize = 4 + (periodicMessageSize * maxPeriodicMessages);

constexpr uint32_t kMaxCanMessages =
    std::max(maxPersistentMessages, maxPeriodicMessages);

constexpr uint8_t one = 0x01;
constexpr uint8_t two = 0x02;

// 50 MB
constexpr uintmax_t kFreeSpaceThreshold = 50000000;
// 5 MB
constexpr uintmax_t kMinFreeSpaceThreshold = 5000000;

struct DeviceStreamInfo {
    int deviceType;
    int firmwareMessageId;
    int periodicMessageId;
    uint32_t firmwareFrameId;

    uint32_t firmwareStreamHandle;
    uint32_t periodicStreamHandle;
    uint64_t devicesFound = 0;
    uint64_t devicesFirmwareReceived = 0;
};

std::vector<DeviceStreamInfo> deviceStreams;

bool running = false;
bool autoLogging = true;
bool stopped = false;
char* persistentBuffer;
char* periodicBuffer;
int32_t halStatus;
uint32_t timeOffsetMillis;
uint8_t bitfield;
uint32_t persistentMessageCount = 0;

uint32_t readCount = 0;
uint64_t devicesCANReady = 0;

std::string path;
std::string temp_filename;
std::ofstream outFile;
bool outFileHasBeenRenamed;
bool lowSpaceWarningShown = false;
std::error_code ec;

// TODO: (dave) support logging all busses
constexpr int TODO_BUS_ID{0};

void StatusLoggerDriver_start(void) {
    if (running) return;

#ifdef __FRC_ROBORIO__
    auto s = std::filesystem::status("/u", ec);
    if (!ec && std::filesystem::is_directory(s) &&
        (s.permissions() & std::filesystem::perms::others_write) !=
            std::filesystem::perms::none) {
        path = "/u/logs/";
    } else {
        getREVLibDriver()->sendWarning(
            0,
            "StatusLogger: It is not recommended to log to RoboRIO "
            "internal storage. Plug in a flash drive.",
            false);
        path = "/home/lvuser/logs/";
    }
#else
    path = std::filesystem::current_path().string() + "/logs";
#endif

    std::filesystem::create_directory(path, ec);

    std::random_device dev;
    std::mt19937 rng(dev());
    std::uniform_int_distribution<int> dist(0, 15);
    const char* v = "0123456789abcdef";
    temp_filename = "REV_TBD_";
    for (int i = 0; i < 16; i++) {
        temp_filename += v[dist(rng)];
    }
    temp_filename += ".revlog";

    outFile.open(path + temp_filename, std::ios::binary);
    if (!outFile.is_open()) {
        getREVLibDriver()->sendError(
            0,
            ("StatusLogger: Failed to open log file at " + path + temp_filename)
                .c_str(),
            false);
        running = false;
        return;
    }
    getREVLibDriver()->sendConsoleLine(
        ("StatusLogger: Logging REVLOG to \'" + path + temp_filename + "\'")
            .c_str());
    outFileHasBeenRenamed = false;

    running = true;

    struct DeviceStreamInfo devices[] = {
        {sparkDeviceType, sparkFirmwareMessageId, sparkPeriodicMessageId,
         SPARK_GET_FIRMWARE_VERSION_FRAME_ID, 0, 0},
        {encoderDeviceType, encoderFirmwareMessageId, encoderPeriodicMessageId,
         ENCODER_GET_VERSIONING_FRAME_ID, 0, 0},
        {servoHubDeviceType, servoHubFirmwareMessageId,
         servoHubPeriodicMessageId, SERVO_HUB_GET_VERSION_FRAME_ID, 0, 0},
        // 3. Create DeviceStreamInfo given the new device's deviceType,
        // firmwareMessageId, periodicMessageId, and firmware frame id
    };

    for (size_t i = 0; i < sizeof(devices) / sizeof(devices[0]); i++) {
        deviceStreams.push_back(devices[i]);
    }

    persistentBuffer = reinterpret_cast<char*>(std::malloc(persistentSize));
    periodicBuffer = reinterpret_cast<char*>(std::malloc(periodicSize));
    uint32_t fpgaMillis = (getREVLibDriver()->getPreciseTime() / 1000ull);
    timeOffsetMillis = fpgaMillis - getREVLibDriver()->getCANPacketBaseTime();

    for (auto& deviceInfo : deviceStreams) {
        deviceInfo.firmwareStreamHandle = getREVLibDriver()->createCanStream(
            TODO_BUS_ID, deviceInfo.firmwareMessageId, firmwareMessageIdMask,
            maxPersistentMessages, &halStatus);

        deviceInfo.periodicStreamHandle = getREVLibDriver()->createCanStream(
            TODO_BUS_ID, deviceInfo.periodicMessageId, periodicMessageIdMask,
            maxPeriodicMessages, &halStatus);
    }
}

void StatusLoggerDriver_manualStart(void) {
    stopped = false;
    StatusLoggerDriver_start();  // Will not run if start has already been
                                 // called before
}

void StatusLoggerDriver_disableAutoLogging(void) {
    autoLogging = false;
    StatusLoggerDriver_stop();
}

void StatusLoggerDriver_stop(void) { stopped = true; }

bool StatusLoggerDriver_getAutoLogging(void) { return autoLogging; }

/**
 * Write a persistent message to the buffer, replacing the value if it exists.
 */
void writeMessagePersistent(CanMessage message) {
    uint32_t index = 0;
    while (index < persistentMessageCount) {
        char* messageBuffer =
            persistentBuffer + 4 + (index * persistentMessageSize);
        uint32_t existingMessageId;
        std::memcpy(&existingMessageId, messageBuffer, 4);
        if (message.messageID == existingMessageId) {
            std::memcpy(messageBuffer + 4, &message.data, 6);
            return;
        }
        index++;
    }

    if (persistentMessageCount < maxPersistentMessages) {
        persistentMessageCount++;
        char* messageBuffer =
            persistentBuffer + 4 + (index * persistentMessageSize);
        std::memcpy(messageBuffer, &message.messageID, 4);
        std::memcpy(messageBuffer + 4, &message.data, 6);
    }
}

/**
 * Write a periodic message to the buffer at the specified index.
 */
void writeMessagePeriodic(CanMessage message, uint32_t index) {
    char* messageBuffer = periodicBuffer + 4 + (index * periodicMessageSize);
    uint32_t timestamp = message.timestamp + timeOffsetMillis;
    std::memcpy(messageBuffer + 0, &timestamp, 4);
    std::memcpy(messageBuffer + 4, &message.messageID, 4);
    std::memcpy(messageBuffer + 8, &message.data, 8);
}

void StatusLoggerDriver_read(void) {
    if (!outFile.good()) {
        getREVLibDriver()->sendError(
            0,
            "StatusLogger: A file write operation failed. Stopping logger. "
            "Please reboot your roboRIO",
            false);
        running = false;
        outFile.close();
        return;
    }
    auto s = std::filesystem::status("/u", ec);
    if (!ec && std::filesystem::is_directory(s) &&
        (s.permissions() & std::filesystem::perms::others_write) !=
            std::filesystem::perms::none &&
        path == "/home/lvuser/logs/") {
        getREVLibDriver()->sendWarning(
            0,
            "StatusLogger: A flash drive was detected. Restart your robot code "
            "to write to the drive.",
            false);
    }
    if (!running || stopped) return;
    if (std::endian::native != std::endian::little) {
        // Little endian expected by AdvantageScope
        return;
    }

    if (!outFileHasBeenRenamed) {
        std::time_t now = std::time(nullptr);
        struct tm* time_info = std::gmtime(&now);
        if (time_info->tm_year > 120) {
            char buffer[80];

            std::strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%S", time_info);
            std::string filename = "REV_" + std::string(buffer) + ".revlog";

            if (outFile.is_open()) {
                outFile.close();
            }

            if (std::rename((path + temp_filename).c_str(),
                            (path + filename).c_str()) == 0) {
                outFile.open(path + filename, std::ios::binary);
                if (!outFile.is_open()) {
                    getREVLibDriver()->sendError(
                        0,
                        ("StatusLogger: Failed to open log file at " + path +
                         temp_filename)
                            .c_str(),
                        false);
                    running = false;
                    return;
                }

                getREVLibDriver()->sendConsoleLine(
                    ("StatusLogger: Renamed REVLOG from \'" + temp_filename +
                     "\' to \'" + filename + "\' at \'" + path + filename +
                     "\'")
                        .c_str());
                outFileHasBeenRenamed = true;
            } else {
                outFile.open(path + temp_filename, std::ios::binary);
                if (!outFile.is_open()) {
                    getREVLibDriver()->sendError(
                        0,
                        ("StatusLogger: Failed to open log file at " + path +
                         temp_filename)
                            .c_str(),
                        false);
                    running = false;
                    return;
                }
            }
        }
    }

    std::error_code ec;
    uintmax_t freeSpace;
    auto freeSpaceInfo = std::filesystem::space(path, ec);
    if (!ec) {
        freeSpace = freeSpaceInfo.available;
    } else {
        freeSpace = UINTMAX_MAX;
    }

    if (freeSpace <= kMinFreeSpaceThreshold) {
        StatusLoggerDriver_stop();
        getREVLibDriver()->sendError(
            0,
            ("StatusLogger: Log storage device has less than " +
             std::to_string(kMinFreeSpaceThreshold / 1000000) +
             " MB of free space remaining. Logging has been stopped "
             "automatically due to low disk space.")
                .c_str(),
            false);
    } else if (freeSpace < kFreeSpaceThreshold) {
        std::vector<std::filesystem::directory_entry> entries;
        for (auto&& entry : std::filesystem::directory_iterator{path, ec}) {
            auto stem = entry.path().stem().string();
            if (stem.rfind("REV_", 0) == 0 &&
                !(stem.rfind("REV_TBD_", 0) == 0) &&
                entry.path().extension() == ".revlog") {
                entries.emplace_back(entry);
            }
        }
        std::sort(entries.begin(), entries.end(),
                  [](const auto& a, const auto& b) {
                      return a.last_write_time() < b.last_write_time();
                  });
        int count = entries.size();

        if (count != 1) {  // Don't delete the current file
            for (auto&& entry : entries) {
                auto size = entry.file_size();
                if (std::filesystem::remove(entry.path(), ec)) {
                    getREVLibDriver()->sendError(
                        0,
                        ("StatusLogger: Log storage device has less than " +
                         std::to_string(kFreeSpaceThreshold / 1000000) +
                         " MB of free space remaining. Deleted \'" +
                         entry.path().string() + "\'")
                            .c_str(),
                        false);
                    freeSpace += size;
                    if (freeSpace >= kFreeSpaceThreshold) {
                        break;
                    }
                } else {
                    getREVLibDriver()->sendError(
                        0,
                        ("StatusLogger: could not delete \'" +
                         entry.path().string() + "\'")
                            .c_str(),
                        false);
                }
            }
        }

    } else if (freeSpace < 2 * kFreeSpaceThreshold && !lowSpaceWarningShown) {
        lowSpaceWarningShown = true;
        getREVLibDriver()->sendWarning(
            0,
            ("StatusLogger: Log storage device has " +
             std::to_string(freeSpace / 1000000) +
             " MB of free space remaining. REVLOGs will get deleted below " +
             std::to_string(kFreeSpaceThreshold / 1000000) +
             " MB of free space. Consider deleting logs off the "
             "storage device.")
                .c_str(),
            false);
    } else if (freeSpace >= 2 * kFreeSpaceThreshold && lowSpaceWarningShown) {
        lowSpaceWarningShown = false;
    }

    // Request unknown firmware and models every (~400ms)
    readCount += 1;
    if (readCount >= 20) {
        readCount = 0;
        for (auto& deviceInfo : deviceStreams) {
            uint64_t unknownFirmwareDevices =
                deviceInfo.devicesFound & ~deviceInfo.devicesFirmwareReceived;
            for (uint8_t i = 0; i < 64; i++) {
                bool unknownFirmware = (unknownFirmwareDevices >> i) & 1;
                if (unknownFirmware) {
                    getREVLibDriver()->writeCanRtrFrame(
                        TODO_BUS_ID, deviceInfo.firmwareFrameId | i, 8,
                        SEND_PERIOD_NO_REPEAT, &halStatus);
                }
            }
        }
    }

    CanMessage messages[kMaxCanMessages];
    uint32_t messageCount = 0;

    for (auto& deviceInfo : deviceStreams) {
        // Read firmware messages
        getREVLibDriver()->receiveCanMessages(deviceInfo.firmwareStreamHandle,
                                              messages, maxPersistentMessages,
                                              &messageCount, &halStatus);
        for (uint32_t i = 0; i < messageCount; i++) {
            writeMessagePersistent(messages[i]);
            uint8_t deviceId = messages[i].messageID & 0x3f;
            deviceInfo.devicesFound |= static_cast<uint64_t>(1) << deviceId;
            deviceInfo.devicesFirmwareReceived |= static_cast<uint64_t>(1)
                                                  << deviceId;
        }

        // Read periodic messages
        getREVLibDriver()->receiveCanMessages(deviceInfo.periodicStreamHandle,
                                              messages, maxPeriodicMessages,
                                              &messageCount, &halStatus);

        for (uint32_t i = 0; i < messageCount; i++) {
            writeMessagePeriodic(messages[i], i);
            uint8_t deviceId = messages[i].messageID & 0x3f;
            deviceInfo.devicesFound |= static_cast<uint64_t>(1) << deviceId;
        }

        // Write sizes
        uint32_t persistentSize =
            persistentMessageCount * persistentMessageSize;
        uint32_t periodicSize = messageCount * periodicMessageSize;
        std::memcpy(persistentBuffer, &persistentSize, 4);
        std::memcpy(periodicBuffer, &periodicSize, 4);

        auto requiredBytes = [](uint64_t value) -> uint8_t {
            if (value <= 0xFF) return 1;
            if (value <= 0xFFFF) return 2;
            if (value <= 0xFFFFFF) return 3;
            return 4;
        };

        std::vector<char> record;

        if (persistentSize > 0) {
            bitfield = 0;
            bitfield |= (((requiredBytes(persistentSize) - 1) & 0x3) << 2);

            record.insert(record.end(),
                          reinterpret_cast<const char*>(&bitfield),
                          reinterpret_cast<const char*>(&bitfield) +
                              requiredBytes(bitfield));
            record.insert(
                record.end(), reinterpret_cast<const char*>(&one),
                reinterpret_cast<const char*>(&one) + requiredBytes(one));
            record.insert(record.end(),
                          reinterpret_cast<const char*>(&persistentSize),
                          reinterpret_cast<const char*>(&persistentSize) +
                              requiredBytes(persistentSize));
            record.insert(record.end(),
                          reinterpret_cast<const char*>(persistentBuffer + 4),
                          reinterpret_cast<const char*>(persistentBuffer + 4) +
                              persistentSize);
        }

        if (periodicSize > 0) {
            bitfield = 0;
            bitfield |= (((requiredBytes(periodicSize) - 1) & 0x3) << 2);

            record.insert(record.end(),
                          reinterpret_cast<const char*>(&bitfield),
                          reinterpret_cast<const char*>(&bitfield) +
                              requiredBytes(bitfield));
            record.insert(
                record.end(), reinterpret_cast<const char*>(&two),
                reinterpret_cast<const char*>(&two) + requiredBytes(two));
            record.insert(record.end(),
                          reinterpret_cast<const char*>(&periodicSize),
                          reinterpret_cast<const char*>(&periodicSize) +
                              requiredBytes(periodicSize));
            record.insert(record.end(),
                          reinterpret_cast<const char*>(periodicBuffer + 4),
                          reinterpret_cast<const char*>(periodicBuffer + 4) +
                              periodicSize);
        }
        outFile.write(record.data(), record.size());
        if (outFile.is_open()) {
            outFile.flush();
        }
    }

    if (outFile.is_open()) {
        outFile.flush();
    }
}
}  // extern "C"
