/*!
 *  @file DHT.h
 *
 *  This is a library for DHT series of low cost temperature/humidity sensors.
 *
 *  You must have Adafruit Unified Sensor Library library installed to use this
 * class.
 *
 *  Adafruit invests time and resources providing this open source code,
 *  please support Adafruit andopen-source hardware by purchasing products
 *  from Adafruit!
 *
 *  Written by Adafruit Industries.
 *
 *  MIT license, all text above must be included in any redistribution
 */

#ifndef DHT_H
#define DHT_H

#include "Arduino.h"

#include <optional>
#include <array>

#include <espchrono.h>

#include <hal/gpio_types.h>

/* Uncomment to enable printing out nice debug messages. */
//#define DHT_DEBUG

#define DEBUG_PRINTER                                                          \
  Serial /**< Define where debug output will be printed.                       \
          */

/* Setup debug printing macros. */
#ifdef DHT_DEBUG
#define DEBUG_PRINT(...)                                                       \
  { DEBUG_PRINTER.print(__VA_ARGS__); }
#define DEBUG_PRINTLN(...)                                                     \
  { DEBUG_PRINTER.println(__VA_ARGS__); }
#else
#define DEBUG_PRINT(...)                                                       \
  {} /**< Debug Print Placeholder if Debug is disabled */
#define DEBUG_PRINTLN(...)                                                     \
  {} /**< Debug Print Line Placeholder if Debug is disabled */
#endif

/* Define types of sensors. */
constexpr const uint8_t DHT11 = 11;  /**< DHT TYPE 11 */
constexpr const uint8_t DHT12 = 12;  /**< DHY TYPE 12 */
constexpr const uint8_t DHT22 = 22;  /**< DHT TYPE 22 */
constexpr const uint8_t DHT21 = 21;  /**< DHT TYPE 21 */
constexpr const uint8_t AM2301 = 21; /**< AM2301 */

#if defined(TARGET_NAME) && (TARGET_NAME == ARDUINO_NANO33BLE)
#ifndef microsecondsToClockCycles
/*!
 * As of 7 Sep 2020 the Arduino Nano 33 BLE boards do not have
 * microsecondsToClockCycles defined.
 */
#define microsecondsToClockCycles(a) ((a) * (SystemCoreClock / 1000000L))
#endif
#endif

/*!
 *  @brief  Class that stores state and functions for DHT
 */
class DHT {
public:
  DHT(gpio_num_t pin, uint8_t type, uint8_t count = 6);

  bool begin(uint8_t usec = 55);

  std::optional<float> readTemperature(bool S = false, bool force = false);
  float readTemperature(const std::array<uint8_t, 5> &data, bool S = false) const;

  static inline float convertCtoF(float c) { return c * 1.8 + 32; }
  static inline float convertFtoC(float f) { return (f - 32) * 0.55555; }

  std::optional<float> computeHeatIndex(bool isFahrenheit = true);
  static float computeHeatIndex(float temperature, float percentHumidity,
                                bool isFahrenheit = true);

  std::optional<float> readHumidity(bool force = false);
  float readHumidity(const std::array<uint8_t, 5> &data) const;

  const std::optional<std::array<uint8_t, 5>> &read(bool force = false);

private:
  const gpio_num_t _pin;
  const uint8_t _type;
#ifdef __AVR
  // Use direct GPIO access on an 8-bit AVR so keep track of the port and
  // bitmask for the digital pin connected to the DHT.  Other platforms will use
  // digitalRead.
  const uint8_t _bit;
  const uint8_t _port;
#endif
  const uint32_t _maxcycles;

  std::optional<espchrono::millis_clock::time_point> _lastreadtime;
  std::optional<std::array<uint8_t, 5>> _lastresult;

  uint8_t pullTime; // Time (in usec) to pull up data line before reading

  uint32_t expectPulse(bool level);
};

/*!
 *  @brief  Class that defines Interrupt Lock Avaiability
 */
class InterruptLock {
public:
  InterruptLock() {
#if !defined(ARDUINO_ARCH_NRF52)
    noInterrupts();
#endif
  }
  ~InterruptLock() {
#if !defined(ARDUINO_ARCH_NRF52)
    interrupts();
#endif
  }
};

#endif
