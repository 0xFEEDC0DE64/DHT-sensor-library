/*!
 *  @file DHT.cpp
 *
 *  @mainpage DHT series of low cost temperature/humidity sensors.
 *
 *  @section intro_sec Introduction
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
 *  @section author Author
 *
 *  Written by Adafruit Industries.
 *
 *  @section license License
 *
 *  MIT license, all text above must be included in any redistribution
 */

#include "DHT.h"

#include <chrono>

using namespace std::chrono_literals;

constexpr const auto MIN_INTERVAL = 2s; /**< min interval value */
#define TIMEOUT                                                                \
  UINT32_MAX /**< Used programmatically for timeout.                           \
                   Not a timeout duration. Type: uint32_t. */

/*!
 *  @brief  Instantiates a new DHT class
 *  @param  pin
 *          pin number that sensor is connected
 *  @param  type
 *          type of sensor
 *  @param  count
 *          number of sensors
 */
DHT::DHT(uint8_t pin, uint8_t type, uint8_t count) :
  _pin{pin},
  _type{type},
#ifdef __AVR
  _bit{digitalPinToBitMask(pin)},
  _port{digitalPinToPort(pin)},
#endif
  // 1 millisecond timeout for
  // reading pulses from DHT sensor.
  _maxcycles(microsecondsToClockCycles(1000))
{
  (void)count; // Workaround to avoid compiler warning.
  // Note that count is now ignored as the DHT reading algorithm adjusts itself
  // based on the speed of the processor.
}

/*!
 *  @brief  Setup sensor pins and set pull timings
 *  @param  usec
 *          Optionally pass pull-up time (in microseconds) before DHT reading
 *starts. Default is 55 (see function declaration in DHT.h).
 */
bool DHT::begin(uint8_t usec) {
  // set up the pins!
  pinMode(_pin, INPUT_PULLUP);

  DEBUG_PRINT("DHT max clock cycles: ");
  DEBUG_PRINTLN(_maxcycles, DEC);
  pullTime = usec;

  return true;
}

/*!
 *  @brief  Read temperature
 *  @param  S
 *          Scale. Boolean value:
 *					- true = Fahrenheit
 *					- false = Celcius
 *  @param  force
 *          true if in force mode
 *	@return Temperature value in selected scale
 */
std::optional<float> DHT::readTemperature(bool S, bool force) {
  const auto data = read(force);
  if (!data)
    return std::nullopt;

  return readTemperature(*data, S);
}

float DHT::readTemperature(const std::array<uint8_t, 5> &data, bool S) const {
  switch (_type) {
  case DHT11: {
    float f = data[2];
    if (data[3] & 0x80) {
      f = -1 - f;
    }
    f += (data[3] & 0x0f) * 0.1;
    if (S) {
      f = convertCtoF(f);
    }
    return f;
  }
  case DHT12: {
    float f = data[2];
    f += (data[3] & 0x0f) * 0.1;
    if (data[2] & 0x80) {
      f *= -1;
    }
    if (S) {
      f = convertCtoF(f);
    }
    return f;
  }
  case DHT22:
  case DHT21: {
    float f = ((word)(data[2] & 0x7F)) << 8 | data[3];
    f *= 0.1;
    if (data[2] & 0x80) {
      f *= -1;
    }
    if (S) {
      f = convertCtoF(f);
    }
    return f;
  }
  }

  __builtin_unreachable();
}

/*!
 *  @brief  Read Humidity
 *  @param  force
 *					force read mode
 *	@return float value - humidity in percent
 */
std::optional<float> DHT::readHumidity(bool force) {
  const auto data = read(force);
  if (!data)
    return std::nullopt;

  return readHumidity(*data);
}

float DHT::readHumidity(const std::array<uint8_t, 5> &data) const {
  switch (_type) {
  case DHT11:
  case DHT12: {
    float f = data[0] + data[1] * 0.1;
    return f;
  }
  case DHT22:
  case DHT21: {
    float f = ((word)data[0]) << 8 | data[1];
    f *= 0.1;
    return f;
  }
  }

  __builtin_unreachable();
}

/*!
 *  @brief  Compute Heat Index
 *          Simplified version that reads temp and humidity from sensor
 *  @param  isFahrenheit
 * 					true if fahrenheit, false if celcius
 *(default true)
 *	@return float heat index
 */
std::optional<float> DHT::computeHeatIndex(bool isFahrenheit) {
  const auto temperature = readTemperature(isFahrenheit);
  if (!temperature)
    return std::nullopt;

  const auto humidity = readHumidity();
  if (!humidity)
    return std::nullopt;

  float hi = computeHeatIndex(*temperature, *humidity, isFahrenheit);
  return hi;
}

/*!
 *  @brief  Compute Heat Index
 *  				Using both Rothfusz and Steadman's equations
 *					(http://www.wpc.ncep.noaa.gov/html/heatindex_equation.shtml)
 *  @param  temperature
 *          temperature in selected scale
 *  @param  percentHumidity
 *          humidity in percent
 *  @param  isFahrenheit
 * 					true if fahrenheit, false if celcius
 *	@return float heat index
 */
/*static*/ float DHT::computeHeatIndex(float temperature, float percentHumidity,
                                       bool isFahrenheit) {
  float hi;

  if (!isFahrenheit)
    temperature = convertCtoF(temperature);

  hi = 0.5 * (temperature + 61.0 + ((temperature - 68.0) * 1.2) +
              (percentHumidity * 0.094));

  if (hi > 79) {
    hi = -42.379 + 2.04901523 * temperature + 10.14333127 * percentHumidity +
         -0.22475541 * temperature * percentHumidity +
         -0.00683783 * pow(temperature, 2) +
         -0.05481717 * pow(percentHumidity, 2) +
         0.00122874 * pow(temperature, 2) * percentHumidity +
         0.00085282 * temperature * pow(percentHumidity, 2) +
         -0.00000199 * pow(temperature, 2) * pow(percentHumidity, 2);

    if ((percentHumidity < 13) && (temperature >= 80.0) &&
        (temperature <= 112.0))
      hi -= ((13.0 - percentHumidity) * 0.25) *
            sqrt((17.0 - abs(temperature - 95.0)) * 0.05882);

    else if ((percentHumidity > 85.0) && (temperature >= 80.0) &&
             (temperature <= 87.0))
      hi += ((percentHumidity - 85.0) * 0.1) * ((87.0 - temperature) * 0.2);
  }

  return isFahrenheit ? hi : convertFtoC(hi);
}

/*!
 *  @brief  Read value from sensor or return last one from less than two
 *seconds.
 *  @param  force
 *          true if using force mode
 *	@return float value
 */
const std::optional<std::array<uint8_t, 5>> &DHT::read(bool force) {
  // Check if sensor was read less than two seconds ago and return early
  // to use last reading.
  espchrono::millis_clock::time_point currenttime = espchrono::millis_clock::now();
  if (!force && _lastreadtime && ((currenttime - *_lastreadtime) < MIN_INTERVAL)) {
    return _lastresult; // return last correct measurement
  }
  _lastreadtime = currenttime;

  // Reset 40 bits of received data to zero.
  _lastresult = std::array<uint8_t, 5>{0, 0, 0, 0, 0};

#if defined(ESP8266)
  yield(); // Handle WiFi / reset software watchdog
#endif

  // Send start signal.  See DHT datasheet for full signal diagram:
  //   http://www.adafruit.com/datasheets/Digital%20humidity%20and%20temperature%20sensor%20AM2302.pdf

  // Go into high impedence state to let pull-up raise data line level and
  // start the reading process.
  pinMode(_pin, INPUT_PULLUP);
  delay(1);

  // First set data line low for a period according to sensor type
  pinMode(_pin, OUTPUT);
  digitalWrite(_pin, LOW);
  switch (_type) {
  case DHT22:
  case DHT21:
    delayMicroseconds(1100); // data sheet says "at least 1ms"
    break;
  case DHT11:
  default:
    delay(20); // data sheet says at least 18ms, 20ms just to be safe
    break;
  }

  uint32_t cycles[80];
  {
    // End the start signal by setting data line high for 40 microseconds.
    pinMode(_pin, INPUT_PULLUP);

    // Delay a moment to let sensor pull data line low.
    delayMicroseconds(pullTime);

    // Now start reading the data line to get the value from the DHT sensor.

    // Turn off interrupts temporarily because the next sections
    // are timing critical and we don't want any interruptions.
    InterruptLock lock;

    // First expect a low signal for ~80 microseconds followed by a high signal
    // for ~80 microseconds again.
    if (expectPulse(LOW) == TIMEOUT) {
      DEBUG_PRINTLN(F("DHT timeout waiting for start signal low pulse."));
      _lastresult = std::nullopt;
      return _lastresult;
    }
    if (expectPulse(HIGH) == TIMEOUT) {
      DEBUG_PRINTLN(F("DHT timeout waiting for start signal high pulse."));
      _lastresult = std::nullopt;
      return _lastresult;
    }

    // Now read the 40 bits sent by the sensor.  Each bit is sent as a 50
    // microsecond low pulse followed by a variable length high pulse.  If the
    // high pulse is ~28 microseconds then it's a 0 and if it's ~70 microseconds
    // then it's a 1.  We measure the cycle count of the initial 50us low pulse
    // and use that to compare to the cycle count of the high pulse to determine
    // if the bit is a 0 (high state cycle count < low state cycle count), or a
    // 1 (high state cycle count > low state cycle count). Note that for speed
    // all the pulses are read into a array and then examined in a later step.
    for (int i = 0; i < 80; i += 2) {
      cycles[i] = expectPulse(LOW);
      cycles[i + 1] = expectPulse(HIGH);
    }
  } // Timing critical code is now complete.

  // Inspect pulses and determine which ones are 0 (high state cycle count < low
  // state cycle count), or 1 (high state cycle count > low state cycle count).
  for (int i = 0; i < 40; ++i) {
    uint32_t lowCycles = cycles[2 * i];
    uint32_t highCycles = cycles[2 * i + 1];
    if ((lowCycles == TIMEOUT) || (highCycles == TIMEOUT)) {
      DEBUG_PRINTLN(F("DHT timeout waiting for pulse."));
      _lastresult = std::nullopt;
      return _lastresult;
    }
    (*_lastresult)[i / 8] <<= 1;
    // Now compare the low and high cycle times to see if the bit is a 0 or 1.
    if (highCycles > lowCycles) {
      // High cycles are greater than 50us low cycle count, must be a 1.
      (*_lastresult)[i / 8] |= 1;
    }
    // Else high cycles are less than (or equal to, a weird case) the 50us low
    // cycle count so this must be a zero.  Nothing needs to be changed in the
    // stored data.
  }

  DEBUG_PRINTLN(F("Received from DHT:"));
  DEBUG_PRINT((*_lastresult)[0], HEX);
  DEBUG_PRINT(F(", "));
  DEBUG_PRINT((*_lastresult)[1], HEX);
  DEBUG_PRINT(F(", "));
  DEBUG_PRINT((*_lastresult)[2], HEX);
  DEBUG_PRINT(F(", "));
  DEBUG_PRINT((*_lastresult)[3], HEX);
  DEBUG_PRINT(F(", "));
  DEBUG_PRINT((*_lastresult)[4], HEX);
  DEBUG_PRINT(F(" =? "));
  DEBUG_PRINTLN(((*_lastresult)[0] + (*_lastresult)[1] + (*_lastresult)[2] + (*_lastresult)[3]) & 0xFF, HEX);

  // Check we read 40 bits and that the checksum matches.
  if ((*_lastresult)[4] != (((*_lastresult)[0] + (*_lastresult)[1] + (*_lastresult)[2] + (*_lastresult)[3]) & 0xFF)) {
    DEBUG_PRINTLN(F("DHT checksum failure!"));
    _lastresult = std::nullopt;
    return _lastresult;
  }

  return _lastresult;
}

// Expect the signal line to be at the specified level for a period of time and
// return a count of loop cycles spent at that level (this cycle count can be
// used to compare the relative time of two pulses).  If more than a millisecond
// ellapses without the level changing then the call fails with a 0 response.
// This is adapted from Arduino's pulseInLong function (which is only available
// in the very latest IDE versions):
//   https://github.com/arduino/Arduino/blob/master/hardware/arduino/avr/cores/arduino/wiring_pulse.c
uint32_t DHT::expectPulse(bool level) {
#if (F_CPU > 16000000L)
  uint32_t count = 0;
#else
  uint16_t count = 0; // To work fast enough on slower AVR boards
#endif
// On AVR platforms use direct GPIO port access as it's much faster and better
// for catching pulses that are 10's of microseconds in length:
#ifdef __AVR
  uint8_t portState = level ? _bit : 0;
  while ((*portInputRegister(_port) & _bit) == portState) {
    if (count++ >= _maxcycles) {
      return TIMEOUT; // Exceeded timeout, fail.
    }
  }
// Otherwise fall back to using digitalRead (this seems to be necessary on
// ESP8266 right now, perhaps bugs in direct port access functions?).
#else
  while (digitalRead(_pin) == level) {
    if (count++ >= _maxcycles) {
      return TIMEOUT; // Exceeded timeout, fail.
    }
  }
#endif

  return count;
}
