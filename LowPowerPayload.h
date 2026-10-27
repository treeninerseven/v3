// SPDX-License-Identifier: MIT
#pragma once
#include <stdint.h>

namespace LowPowerPayload {
constexpr uint8_t SIZE = 16;
constexpr uint8_t TEMPERATURE_VALID = 1, HUMIDITY_VALID = 2;
constexpr uint8_t PRESSURE_VALID = 4, LIGHT_VALID = 8, RTC_USES_LSI = 16;

struct Sample {
  uint8_t flags = 0;
  int16_t temperatureCentiC = 0;
  uint16_t humidityCentiPercent = 0;
  uint32_t pressurePa = 0;
  uint16_t lightRaw = 0;
};

constexpr void store(uint8_t *out, uint32_t value, uint8_t bytes) {
  for (uint8_t i = 0; i < bytes; ++i) out[i] = value >> ((bytes - 1 - i) * 8);
}

constexpr void encode(uint8_t *out, uint32_t sequence, const Sample &sample) {
  out[0] = 5;
  out[1] = sample.flags;
  store(out + 2, sequence, 4);
  store(out + 6, uint16_t(sample.temperatureCentiC), 2);
  store(out + 8, sample.humidityCentiPercent, 2);
  store(out + 10, sample.pressurePa, 4);
  store(out + 14, sample.lightRaw, 2);
}
}  // namespace LowPowerPayload
