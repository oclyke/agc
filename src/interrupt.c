/**
 * @file interrupt.c
 * @brief Interrupt handlers for the AGC firmware.
 *
 *  The CMSIS startup file supplies a weak alias to Default_Handler for
 *  every vector, so only the handlers listed here need to exist.
 */

#include "board.h"

#include "stm32g4xx_hal.h"

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
extern UART_HandleTypeDef hlpuart1;
extern DMA_HandleTypeDef hdma_lpuart1_tx;

void NMI_Handler(void) {
  while (1) {
    /** A non-maskable interrupt has ocurred and has no handlers. */
  }
}

void HardFault_Handler(void) {
  while (1) {
    /** A hard fault has occurred and has no handlers. */
  }
}

void MemManage_Handler(void) {
  while (1) {
    /** A memory management fault has occurred and has no handlers. */
  }
}

void BusFault_Handler(void) {
  while (1) {
    /** A bus fault has occurred and has no handlers. */
  }
}

void UsageFault_Handler(void) {
  while (1) {
    /** A usage fault has occurred and has no handlers. */
  }
}

void SVC_Handler(void) {
  /** Nothing to do for a system service call (via SWI instruction). */
}

void DebugMon_Handler(void) {
  /** Nothing to do for a debug monitor interrupt. */
}

void PendSV_Handler(void) {
  /** Nothing to do for a pendable request for system service. */
}

void SysTick_Handler(void) {
  HAL_IncTick();
}

void ADC1_2_IRQHandler(void) {
  HAL_ADC_IRQHandler(&hadc1);
}

void DMA1_Channel1_IRQHandler(void) {
  HAL_DMA_IRQHandler(&hdma_adc1);
}

void DMA1_Channel2_IRQHandler(void) {
  HAL_DMA_IRQHandler(&hdma_lpuart1_tx);
}

void LPUART1_IRQHandler(void) {
  HAL_UART_IRQHandler(&hlpuart1);
}

/**
 * @brief Sample clock, on a pin.
 *
 * Deliberately not routed through HAL_TIM_IRQHandler. The whole point of this
 * pin is an edge that lands at the sample instant, and the HAL path is a few
 * hundred cycles of flag testing before it would get there. Writing the
 * status register outright clears UIF and leaves the other flags set, which is
 * what the reference manual asks for and avoids a read-modify-write.
 *
 * The edge marks the trigger, not the conversion: the ADC starts a few ADC
 * clocks later and takes about 6 us to finish.
 */
void TIM6_DAC_IRQHandler(void) {
  TIM6->SR = ~TIM_SR_UIF;
  SCOPE_TOGGLE();
}
