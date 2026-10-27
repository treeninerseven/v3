// SPDX-License-Identifier: MIT
// Sample onboard environment sensors -> mioty -> RTC timed STM32 STOP -> repeat.
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <ADXL362.h>
#include <SHT31.h>
#include <SparkFun_MS5637_Arduino_Library.h>
#include <miotyAtClient.h>
#include <math.h>
#include "LowPowerPayload.h"
#include "TimedStop.h"

#if !defined(STM32L072xx)
#error "This firmware targets the M3B STM32L072."
#endif
#if defined(USBCON)
#error "Build low-power without USB support (select menu 8 in build-flash)."
#endif

constexpr uint32_t SLEEP_SECONDS = 5U * 60U;
constexpr uint32_t RF_TIMEOUT_MS = 120000;
// Same AT-TU mode as menus 4..7. Set false for a module configured for AT-U.
constexpr bool TRANSPARENT_UPLINK = true;
// Set true only after confirming a working 32.768 kHz crystal on the current board.
constexpr bool USE_LSE = false;
constexpr uint8_t LIGHT_ADDRESS = 0x5A;
static_assert(SLEEP_SECONDS >= 1 && SLEEP_SECONDS <= 65536, "RTC interval: 1..65536 seconds");

Uart SerialMioty(PC_11, PC_10);
TwoWire SensorWire(PB9, PB8);
SHT31 humidity(0x44, &SensorWire);
MS5637 pressure;
ADXL362 accelerometer;
uint32_t sequence = 0, sendStarted = 0;
uint16_t replyBytes = 0;
bool rfHalted = false;

bool writeRegister(uint8_t address, uint8_t reg, uint8_t value) {
  SensorWire.beginTransmission(address);
  SensorWire.write(reg);
  SensorWire.write(value);
  return SensorWire.endTransmission() == 0;
}

bool readRegisters(uint8_t address, uint8_t reg, uint8_t *out, size_t size) {
  SensorWire.beginTransmission(address);
  SensorWire.write(reg);
  if (SensorWire.endTransmission() != 0) return false;
  if (SensorWire.requestFrom(address, size) != size) return false;
  for (size_t i = 0; i < size; ++i) out[i] = SensorWire.read();
  return true;
}

// Si1141 forced ALS only: no autonomous measurement or proximity LED drive.
// Checked transactions prevent a missing light sensor becoming a valid 0xffff.
bool lightCommand(uint8_t command) {
  if (!writeRegister(LIGHT_ADDRESS, 0x18, 0)) return false;  // NOP: clear response
  uint8_t response = 0xff;
  uint32_t start = millis();
  do {
    delay(1);
    if (!readRegisters(LIGHT_ADDRESS, 0x20, &response, 1)) return false;
    if (millis() - start >= 50) return false;
  } while (response != 0);
  if (!writeRegister(LIGHT_ADDRESS, 0x18, command)) return false;
  start = millis();
  do {
    delay(1);
    if (!readRegisters(LIGHT_ADDRESS, 0x20, &response, 1)) return false;
    if (response & 0x80) return false;
    if (response & 0x0f) return true;
  } while (millis() - start < 50);
  return false;
}

bool readLight(uint16_t &value) {
  uint8_t id = 0;
  if (!readRegisters(LIGHT_ADDRESS, 0, &id, 1) || id != 0x41) return false;
  if (!writeRegister(LIGHT_ADDRESS, 0x18, 0x01)) return false;  // reset -> standby
  delay(10);
  if (!writeRegister(LIGHT_ADDRESS, 0x07, 0x17) ||  // HW_KEY
      !writeRegister(LIGHT_ADDRESS, 0x08, 0) ||
      !writeRegister(LIGHT_ADDRESS, 0x09, 0) ||  // MEAS_RATE = 0
      !writeRegister(LIGHT_ADDRESS, 0x17, 0x10) ||  // CHLIST: visible ALS only
      !lightCommand(0xA1) || !lightCommand(0x06)) return false;
  delay(50);  // finish the forced optical conversion before reading its result
  uint8_t raw[2];
  if (!readRegisters(LIGHT_ADDRESS, 0x22, raw, sizeof(raw))) return false;
  value = uint16_t(raw[0]) | uint16_t(raw[1]) << 8;
  return true;
}

void displayOff() {
  // SH1107/SSD1306 optional OLED from existing firmware: display OFF, no framebuffer.
  SensorWire.beginTransmission(0x3C);
  SensorWire.write(0);
  SensorWire.write(0xAE);
  SensorWire.endTransmission();
}

void stopSensorBus() {
  SensorWire.end();
  pinMode(PB8, INPUT_ANALOG);
  pinMode(PB9, INPUT_ANALOG);
}

LowPowerPayload::Sample collectSample() {
  LowPowerPayload::Sample sample;
  if (TimedStop::usesLsi) sample.flags |= LowPowerPayload::RTC_USES_LSI;
  SensorWire.begin();
  SensorWire.setClock(100000);
  displayOff();
  // Reset each cycle also recovers sensors after a transient communication failure.
  const bool shtReady = humidity.begin();
  const bool heaterOff = shtReady && humidity.heatOff();
  if (heaterOff && humidity.read(false)) {  // slow single-shot, CRC checked
    const float temp = humidity.getTemperature(), rh = humidity.getHumidity();
    if (isfinite(temp) && temp >= -45 && temp <= 130) {
      sample.temperatureCentiC = int16_t(lroundf(temp * 100));
      sample.flags |= LowPowerPayload::TEMPERATURE_VALID;
    }
    if (isfinite(rh) && rh >= 0 && rh <= 100) {
      sample.humidityCentiPercent = uint16_t(lroundf(rh * 100));
      sample.flags |= LowPowerPayload::HUMIDITY_VALID;
    }
  }
  if (pressure.begin(SensorWire)) {
    pressure.setResolution(ms5637_resolution_osr_2048);
    float temp = 0, hpa = 0;
    if (pressure.read_temperature_and_pressure(&temp, &hpa) == ms5637_status_ok &&
        isfinite(hpa) && hpa >= 10 && hpa <= 2000) {
      sample.pressurePa = uint32_t(lroundf(hpa * 100));
      sample.flags |= LowPowerPayload::PRESSURE_VALID;
    }
  }
  if (readLight(sample.lightRaw)) sample.flags |= LowPowerPayload::LIGHT_VALID;
  // All three sensors use one-shot conversions and return to standby themselves.
  stopSensorBus();
  return sample;
}

extern "C" void miotyAtClientWrite(uint8_t *data, uint16_t length) {
  SerialMioty.write(data, length);
  SerialMioty.flush();
}
extern "C" bool miotyAtClientRead(uint8_t *data, uint8_t *length) {
  const uint8_t capacity = *length;
  *length = 0;
  if (capacity < 2 || replyBytes >= 1024) return false;
  while (*length + 1 < capacity && millis() - sendStarted < RF_TIMEOUT_MS) {
    if (SerialMioty.available()) {
      data[(*length)++] = SerialMioty.read();
      if (data[*length - 1] == '\r' || data[*length - 1] == '\n') break;
    } else {
      // Shallow CPU sleep while the UART is receiving; SysTick/USART remain active.
      __WFI();
    }
  }
  if (*length && data[*length - 1] == '\r') data[(*length)++] = '\n';
  replyBytes += *length;
  return *length != 0;
}

void transmit(const LowPowerPayload::Sample &sample) {
  uint8_t payload[LowPowerPayload::SIZE];
  LowPowerPayload::encode(payload, ++sequence, sample);
  SerialMioty.begin(9600);
  while (SerialMioty.available()) SerialMioty.read();
  replyBytes = 0;
  sendStarted = millis();
  uint32_t counter = 0;
  auto send = TRANSPARENT_UPLINK ? miotyAtClient_sendMessageUniTransparent : miotyAtClient_sendMessageUni;
  const auto result = send(payload, sizeof(payload), &counter);
  // Timeout/overflow makes later replies ambiguous: stop uplinks until RESET,
  // but keep the MCU sleeping. A clean modem rejection can retry next cycle.
  if (result == MIOTYATCLIENT_RETURN_CODE_ATReadFailed ||
      result == MIOTYATCLIENT_RETURN_CODE_BufferSizeInsufficient ||
      replyBytes >= 1024 || millis() - sendStarted >= RF_TIMEOUT_MS) rfHalted = true;
  SerialMioty.end();
  pinMode(PC11, INPUT_ANALOG);
  digitalWrite(PC10, HIGH);  // keep the modem RX line idle, never drive a break
  pinMode(PC10, OUTPUT);
}

void clockFault() {
  // A failed RTC must not turn into continuous sampling/transmission.
  // Static red indication, no automatic reset/retransmission storm.
  digitalWrite(PC8, LOW);
  pinMode(PC8, OUTPUT);
  HAL_SuspendTick();
  for (;;) __WFI();
}

void setup() {
  // Analog/high impedance disconnects all LED drivers regardless of polarity.
  pinMode(PC6, INPUT_ANALOG);
  pinMode(PC7, INPUT_ANALOG);
  pinMode(PC8, INPUT_ANALOG);
  pinMode(PC13, INPUT_ANALOG);
  __HAL_RCC_HSI48_DISABLE();
  if (!TimedStop::begin(USE_LSE)) clockFault();
  // Existing board ADXL362 wiring. Never enable continuous acceleration sampling.
  accelerometer.begin(PA8);
  accelerometer.SPIwriteOneRegister(0x2D, 0);  // POWER_CTL: standby
  SPI.end();
  pinMode(PA5, INPUT_ANALOG);
  pinMode(PA6, INPUT_ANALOG);
  pinMode(PA7, INPUT_ANALOG);
  // Leave CS high; no USB, GPS, external IMU or OLED initialization.
  delay(1000);  // modem power-up, only once
}

void loop() {
  // First sample immediately after boot. After each result, sleep a full interval.
  if (!rfHalted) transmit(collectSample());
  if (!TimedStop::sleep(SLEEP_SECONDS)) clockFault();
}
