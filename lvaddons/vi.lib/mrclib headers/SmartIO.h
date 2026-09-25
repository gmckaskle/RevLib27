// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "mrclib/MrcLibTypes.h"

#define MRC_SMARTIO_RATE_WINDOW_DEFAULT_MS 50
#define MRC_SMARTIO_RATE_WINDOW_MIN_MS 5
#define MRC_SMARTIO_RATE_WINDOW_MAX_MS 255

MRC_ENUM(MRC_SmartIOMode){
    MRC_SmartIOMode_DigitalInput = 0,
    MRC_SmartIOMode_DigitalOutput = 1,
    MRC_SmartIOMode_AnalogInput = 2,
    MRC_SmartIOMode_PwmInput = 3,
    MRC_SmartIOMode_PwmOutput = 4,
    MRC_SmartIOMode_SingleCounterRising = 5,
    MRC_SmartIOMode_SingleCounterFalling = 6,
    MRC_SmartIOMode_Quadrature = 7,
    MRC_SmartIOMode_AddressableLED = 13,
};

MRC_ENUM(MRC_PwmOutputPeriod){
    MRC_PwmOutputPeriod_20ms = 0,
    MRC_PwmOutputPeriod_10ms,
    MRC_PwmOutputPeriod_5ms,
    MRC_PwmOutputPeriod_2ms,
};

extern "C" {
/**
 * Gets the number of SmartIO devices available on the system.
 *
 * @param count pointer to store the number of devices
 * @return status of the operation
 */
MRC_Status MRC_CALLCONV MRC_SmartIO_GetDeviceCount(int32_t* count);

/**
 * Closes the SmartIO device at the specified index.
 *
 * @param deviceIndex index of device to close (0-based)
 * @return status of the operation
 */
MRC_Status MRC_CALLCONV MRC_SmartIO_Close(int32_t deviceIndex);

/**
 * Initializes the mode of the SmartIO device at the specified index.
 *
 * @param deviceIndex index of device to configure (0-based)
 * @param mode mode to set
 * @return status of the operation
 */
MRC_Status MRC_CALLCONV MRC_SmartIO_InitializeMode(int32_t deviceIndex,
                                                   MRC_SmartIOMode mode);

/**
 * Switches the direction of a SmartIO device configured as digital input or
 * output.
 *
 * @param deviceIndex index of device to configure (0-based)
 * @param isInput true to switch to input, false to switch to output
 * @return status of the operation
 */
MRC_Status MRC_CALLCONV MRC_SmartIO_SwitchDirection(int32_t deviceIndex,
                                                    MRC_Bool isInput);

/**
 * Switches the counting edge of a SmartIO device configured as a single
 * counter.
 *
 * @param deviceIndex index of device to configure (0-based)
 * @param risingEdge true to count rising edges, false to count falling edges
 * @return status of the operation
 */
MRC_Status MRC_CALLCONV MRC_SmartIO_SwitchCounterEdge(int32_t deviceIndex,
                                                      MRC_Bool risingEdge);

/**
 * Sets the rate calculation window of a SmartIO device configured as a single
 * counter or quadrature encoder.
 *
 * @param deviceIndex index of device to configure (0-based)
 * @param windowMilliseconds rate calculation window in milliseconds
 * @return status of the operation
 */
MRC_Status MRC_CALLCONV MRC_SmartIO_SetRateWindow(int32_t deviceIndex,
                                                  int32_t windowMilliseconds);

/**
 * Sets the digital output value of a SmartIO device configured as digital
 * output.
 *
 * @param deviceIndex index of device to configure (0-based)
 * @param value value to set (true for high, false for low)
 * @return status of the operation
 */
MRC_Status MRC_CALLCONV MRC_SmartIO_SetDigitalOutput(int32_t deviceIndex,
                                                     MRC_Bool value);

/**
 * Gets the digital input value of a SmartIO device configured as digital input.
 *
 * @param deviceIndex index of device to read (0-based)
 * @param value pointer to store the read value (true for high, false for low)
 * @return status of the operation
 */
MRC_Status MRC_CALLCONV MRC_SmartIO_GetDigitalInput(int32_t deviceIndex,
                                                    MRC_Bool* value);

/**
 * Gets the PWM input value in microseconds of a SmartIO device configured as
 * PWM input.
 *
 * @param deviceIndex index of device to read (0-based)
 * @param microseconds pointer to store the read value in microseconds
 * @return status of the operation
 */
MRC_Status MRC_CALLCONV
MRC_SmartIO_GetPwmInputMicroseconds(int32_t deviceIndex, int32_t* microseconds);

/**
 * Gets the PWM input period in microseconds of a SmartIO device configured as
 * PWM input.
 *
 * @param deviceIndex index of device to read (0-based)
 * @param microseconds pointer to store the read period in microseconds
 * @return status of the operation
 */
MRC_Status MRC_CALLCONV MRC_SmartIO_GetPwmInputPeriodMicroseconds(
    int32_t deviceIndex, int32_t* microseconds);

/**
 * Sets the PWM output period of a SmartIO device configured as PWM output.
 *
 * @param deviceIndex index of device to configure (0-based)
 * @param period period to set
 * @return status of the operation
 */
MRC_Status MRC_CALLCONV
MRC_SmartIO_SetPwmOutputPeriod(int32_t deviceIndex, MRC_PwmOutputPeriod period);

/**
 * Sets the PWM output value in microseconds of a SmartIO device configured as
 * PWM output.
 *
 * @param deviceIndex index of device to configure (0-based)
 * @param microseconds value to set in microseconds
 * @return status of the operation
 */
MRC_Status MRC_CALLCONV
MRC_SmartIO_SetPwmOutputMicroseconds(int32_t deviceIndex, int32_t microseconds);

/**
 * Gets the analog input value of a SmartIO device configured as analog input.
 *
 * @param deviceIndex index of device to read (0-based)
 * @param value pointer to store the read value
 * @return status of the operation
 */
MRC_Status MRC_CALLCONV MRC_SmartIO_GetAnalogInput(int32_t deviceIndex,
                                                   int32_t* value);

/**
 * Gets the counter value of a SmartIO device configured as single counter.
 *
 * @param deviceIndex index of device to read (0-based)
 * @param count pointer to store the read count value
 * @return status of the operation
 */
MRC_Status MRC_CALLCONV MRC_SmartIO_GetCounter(int32_t deviceIndex,
                                               int32_t* count);

/**
 * Gets the rate of a SmartIO device configured as single counter.
 *
 * @param deviceIndex index of device to read (0-based)
 * @param rate pointer to store the counter rate
 * @return status of the operation
 */
MRC_Status MRC_CALLCONV MRC_SmartIO_GetCounterRate(int32_t deviceIndex,
                                                   int32_t* rate);

/**
 * Gets the count of a SmartIO device configured as a quadrature encoder.
 *
 * @param deviceIndex index of device to read (0-based)
 * @param count pointer to store the quadrature count value
 * @return status of the operation
 */
MRC_Status MRC_CALLCONV MRC_SmartIO_GetQuadrature(int32_t deviceIndex,
                                                  int32_t* count);

/**
 * Gets the rate of a SmartIO device configured as a quadrature encoder.
 *
 * @param deviceIndex index of device to read (0-based)
 * @param rate pointer to store the quadrature rate
 * @return status of the operation
 */
MRC_Status MRC_CALLCONV MRC_SmartIO_GetQuadratureRate(int32_t deviceIndex,
                                                      int32_t* rate);

/*
 * Sets the LED start index of a SmartIO device configured as addressable LED.
 *
 * @param deviceIndex index of device to configure (0-based)
 * @param startIndex starting index of LEDs to control with this device
 * @return status of the operation
 */
MRC_Status MRC_CALLCONV MRC_SmartIO_SetLedStartIndex(int32_t deviceIndex,
                                                     int32_t startIndex);

/**
 * Sets the LED length of a SmartIO device configured as addressable LED.
 *
 * @param deviceIndex index of device to configure (0-based)
 * @param length number of LEDs to control with this device
 * @return status of the operation
 */
MRC_Status MRC_CALLCONV MRC_SmartIO_SetLedLength(int32_t deviceIndex,
                                                 int32_t length);

/**
 * Sets the LED buffer for the entire SmartIO chain.
 *
 * @param ledBuffer pointer to the LED buffer
 * @param bufferLength length of the LED buffer
 * @return status of the operation
 */
MRC_Status MRC_CALLCONV MRC_SmartIO_SetLedBuffer(uint8_t* ledBuffer,
                                                 int32_t bufferLength);
}  // extern "C"
