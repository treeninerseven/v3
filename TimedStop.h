// SPDX-License-Identifier: MIT
#pragma once
#include <Arduino.h>

// STM32L072 only. Uses the HAL bundled with STM32 core 3.0.0; no RTC library.
// LSI is the default so no assumption about the current board's crystal is made.
// The sketch owns RTC and RTC_IRQHandler; do not combine with STM32RTC.
namespace TimedStop {
inline RTC_HandleTypeDef rtc = {};
inline volatile bool expired = false;
inline bool usesLsi = true;

inline bool begin(bool useLse) {
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWR_EnableBkUpAccess();
  // Own the RTC timebase, including after RESET of a previously flashed sketch.
  // This firmware does not keep a wall clock or data in RTC backup registers.
  __HAL_RCC_BACKUPRESET_FORCE();
  __HAL_RCC_BACKUPRESET_RELEASE();
  RCC_OscInitTypeDef osc = {};
  osc.OscillatorType = useLse ? RCC_OSCILLATORTYPE_LSE : RCC_OSCILLATORTYPE_LSI;
  osc.LSEState = RCC_LSE_ON;
  osc.LSIState = RCC_LSI_ON;
  osc.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&osc) != HAL_OK) return false;
  RCC_PeriphCLKInitTypeDef clocks = {};
  clocks.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  clocks.RTCClockSelection = useLse ? RCC_RTCCLKSOURCE_LSE : RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&clocks) != HAL_OK) return false;
  __HAL_RCC_RTC_ENABLE();
  rtc.Instance = RTC;
  rtc.Init.HourFormat = RTC_HOURFORMAT_24;
  rtc.Init.AsynchPrediv = 127;
  rtc.Init.SynchPrediv = (useLse ? LSE_VALUE : LSI_VALUE) / 128U - 1U;
  rtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  rtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  rtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&rtc) != HAL_OK || HAL_RTCEx_DeactivateWakeUpTimer(&rtc) != HAL_OK)
    return false;
  usesLsi = !useLse;
  HAL_NVIC_SetPriority(RTC_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(RTC_IRQn);
  HAL_DBGMCU_DisableDBGSleepMode();
  HAL_DBGMCU_DisableDBGStopMode();
  HAL_DBGMCU_DisableDBGStandbyMode();
  return true;
}

inline bool sleep(uint32_t seconds) {
  if (!seconds || seconds > 65536U) return false;
  expired = false;
  __HAL_RTC_WAKEUPTIMER_EXTI_CLEAR_FLAG();
  HAL_NVIC_ClearPendingIRQ(RTC_IRQn);
  if (HAL_RTCEx_SetWakeUpTimer_IT(&rtc, seconds - 1U,
                                RTC_WAKEUPCLOCK_CK_SPRE_16BITS) != HAL_OK) return false;
  __HAL_RCC_WAKEUPSTOP_CLK_CONFIG(RCC_STOP_WAKEUPCLOCK_HSI);
  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
  HAL_SuspendTick();
  SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk;
  while (!expired) {
    // Masking dispatch across the check/WFI pair prevents a lost RTC wakeup.
    // An enabled pending interrupt still wakes Cortex-M0+ with PRIMASK set.
    __disable_irq();
    if (!expired) HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
    __enable_irq();
  }
  HAL_ResumeTick();
  SystemClock_Config();
  // The generic board clock enables HSI48 even when USB is not compiled in.
  __HAL_RCC_HSI48_DISABLE();
  return HAL_RTCEx_DeactivateWakeUpTimer(&rtc) == HAL_OK;
}
}  // namespace TimedStop

extern "C" void RTC_IRQHandler(void) {
  HAL_RTCEx_WakeUpTimerIRQHandler(&TimedStop::rtc);
}
extern "C" void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *) {
  TimedStop::expired = true;
}
