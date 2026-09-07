/**
 * @file board.h
 * @brief NUCLEO-G474RE pin map.
 *
 *  name          | pin  | function
 *  --------------|------|-------------------------------------------------
 *  MIC           | PA0  | MAX4466 OUT, ADC1_IN1, Arduino A0
 *  LED           | PA5  | LD2, capture heartbeat, Arduino D13
 *  SCOPE         | PB5  | sample clock, Arduino D4
 *  VCP TX        | PA2  | LPUART1_TX to the ST-LINK virtual com port
 *  VCP RX        | PA3  | LPUART1_RX from the ST-LINK virtual com port
 *
 *  LD2 on PA5 and the virtual com port on PA2 / PA3 are from the STM32G4xx
 *  Nucleo BSP. The Arduino header positions follow the NUCLEO-64 mapping, in
 *  which D13 is PA5 and D0 / D1 are PA3 / PA2 - which is what the BSP says,
 *  so the rest of that mapping should hold. Check UM2505 before clipping a
 *  probe to D4 if it matters.
 */

#pragma once

#include "stm32g4xx_hal.h"

/** Microphone input: MAX4466 OUT into Arduino A0. */
#define MIC_PIN (GPIO_PIN_0)
#define MIC_PORT (GPIOA)
#define MIC_ADC_CHANNEL (ADC_CHANNEL_1)
#define MIC_ADC_SAMPLETIME (ADC_SAMPLETIME_247CYCLES_5)

/** User LED LD2, used as a capture heartbeat. */
#define LED_PIN (GPIO_PIN_5)
#define LED_PORT (GPIOA)

/** Sample clock output, for a scope. Toggled in the TIM6 update interrupt. */
#define SCOPE_PIN (GPIO_PIN_5)
#define SCOPE_PORT (GPIOB)
#define SCOPE_TOGGLE() { SCOPE_PORT->ODR ^= SCOPE_PIN; }

/** Virtual com port, wired to LPUART1 through the ST-LINK on this board. */
#define VCP_UART (LPUART1)
#define VCP_BAUDRATE (921600)
#define VCP_TX_PIN (GPIO_PIN_2)
#define VCP_TX_PORT (GPIOA)
#define VCP_RX_PIN (GPIO_PIN_3)
#define VCP_RX_PORT (GPIOA)
#define VCP_AF (GPIO_AF12_LPUART1)
