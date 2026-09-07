/**
 * @file main.c
 * @brief AGC firmware: 8 kHz capture of the MAX4466 microphone, streamed out.
 *
 *  The signal path, end to end:
 *
 *    TIM6 TRGO                one trigger per sample period, no CPU involved
 *        |
 *    ADC1_IN1 (PA0)           12 bit, half scale subtracted in hardware
 *        |
 *    DMA1_Ch1                 circular, ping / pong over two blocks
 *        |
 *    capture_buffer           -2048 .. +2047, signed
 *        |
 *    [ filter, gain stage, AGC ]        NOT WRITTEN YET
 *        |
 *    scale to full scale int16          x16, so the host reads real levels
 *        |
 *    frame_audio / frame_telem          the provided framing library
 *        |
 *    ring buffer              8 KiB; a frame that will not fit is dropped
 *        |                            and counted, never waited on
 *    DMA1_Ch2 -> LPUART1 -> ST-LINK VCP -> host          921600 baud
 *
 *  Capture. TIM6 emits one TRGO per sample period and does nothing else, so
 *  the sample instants are placed by hardware rather than by the CPU. The ADC
 *  offset unit subtracts half scale from every conversion before the data
 *  register is written, so the DC level the microphone module idles at never
 *  reaches memory: with saturation disabled the result is signed, and
 *  0 V .. 3V3 arrives as -2048 .. +2047. DMA runs circular over two blocks -
 *  the half transfer interrupt hands over the first, the transfer complete
 *  interrupt the second, and the ADC fills the other one meanwhile. Nothing
 *  stops, so capture runs indefinitely on 250 interrupts a second.
 *
 *  Processing. Not written yet. Until it is, the captured block is sent as
 *  both channels, so the two halves of the stereo capture are identical, and
 *  gain_db and the two RMS levels in the telemetry record read zero. The
 *  counters in that record are real.
 *
 *  Streaming. Audio frames go out once per block and status records 20 times
 *  a second, together about 37% of the link. This file is the ring buffer's
 *  only writer; the transmit complete interrupt is its only reader. Draining
 *  happens in the block loop, which hands the largest contiguous run straight
 *  to DMA, so nothing is copied on the way out. The stream interrupts sit a
 *  priority below the capture path, so sending cannot delay a block.
 *
 *  Measurement. PB5 toggles once per sample in the TIM6 update interrupt, for
 *  a scope. That is the only per-sample CPU work here, and it is optional -
 *  see SCOPE_SAMPLE_CLOCK below.
 *
 *  Nothing here allocates. Every buffer is static or on the stack.
 */

#include "assert_custom.h"
#include "board.h"

#include "framing.h"
#include "instrument.h"
#include "ringbuf.h"

#include "stm32g4xx_hal.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

/** Fixed by the project brief. */
#define SAMPLE_RATE_HZ (8000)

/** Status record cadence, per the brief: 20 a second. */
#define TELEMETRY_INTERVAL_MS (50)

/** Heartbeat LED cadence. */
#define HEARTBEAT_INTERVAL_MS (500)

/**
 * Drive the sample clock out on a pin, for a scope.
 *
 * This is the one thing here that costs a CPU interrupt per sample: 8000 a
 * second against the 250 the DMA capture path needs. It is a measurement aid,
 * so comment it out for runs where the timing numbers have to be honest.
 */
#define SCOPE_SAMPLE_CLOCK

/** ADC resolution, and the half scale count the offset unit removes. */
#define ADC_RESOLUTION_BITS (12)
#define ADC_HALF_SCALE (1U << (ADC_RESOLUTION_BITS - 1))

// =================
// === machinery ===

/** One block of samples, sized to match the wire format in framing.h. */
#define CAPTURE_BLOCK_SAMPLES (BLOCK_SAMPLES)
#define CAPTURE_BUFFER_SAMPLES (2 * CAPTURE_BLOCK_SAMPLES)

/** Blocks between heartbeat toggles, so the LED runs at a visible 0.5 Hz. */
#define HEARTBEAT_BLOCKS (SAMPLE_RATE_HZ / CAPTURE_BLOCK_SAMPLES)

/**
 * Frames waiting on the UART. Must be a power of two.
 *
 * 8 KiB is about a quarter of a second of stream at 34 kB/s, which is far
 * more slack than the host needs and still under 7% of RAM.
 */
#define STREAM_BUFFER_BYTES (8192)

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
TIM_HandleTypeDef htim6;
UART_HandleTypeDef hlpuart1;
DMA_HandleTypeDef hdma_lpuart1_tx;

/**
 * DMA destination. The ADC writes half words here continuously; the halves are
 * handed to the application alternately. Signed because the ADC offset unit
 * has already removed half scale by the time the data lands.
 */
static volatile int16_t capture_buffer[CAPTURE_BUFFER_SAMPLES] __attribute__((aligned(4)));

/** Handover from the DMA interrupts to the main loop. */
static volatile size_t capture_block_offset = 0;
static volatile bool capture_block_ready = false;

/**
 * False if the DWT cycle counter does not tick, which on some parts happens
 * unless a debugger is attached. Every block timing number would then read
 * zero. Reported over the wire: host_receive.py calls out a
 * worst_block_cycles of zero for exactly this reason.
 */
static volatile bool cycle_counter_ok = false;

/**
 * Frames on their way to the host.
 *
 * ringbuf.h relies on exactly one writer and exactly one reader. The writer
 * is the block loop in main(); the reader is the UART transmit path, which
 * touches the tail only from HAL_UART_TxCpltCallback().
 */
static uint8_t stream_storage[STREAM_BUFFER_BYTES];
static ringbuf_t stream_ringbuf;
static volatile bool stream_tx_busy = false;
static volatile size_t stream_tx_length = 0; // written by the pump, read by the transmit complete interrupt

/** Where the last trap came from. Read these first under a debugger.
 *  Note the placement of volatile: it is the pointer that must be, not the
 *  string, or the store gets optimised away as dead. */
static const char* volatile trap_file = NULL;
static volatile uint32_t trap_line = 0;

/**
 * @brief Offer a freshly filled block to the main loop.
 *
 * Called from the DMA interrupt. If the previous block has not been picked up
 * yet then the main loop missed its deadline: count it and hand over the new
 * block anyway, because the old one is about to be overwritten regardless.
 *
 * @param offset Index of the first sample of the completed block.
 */
static inline void capture_submit_block(size_t offset) {
  if (capture_block_ready) {
    g_counters.dma_overruns++;
  }
  capture_block_offset = offset;
  capture_block_ready = true;
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
  (void)hadc;
  capture_submit_block(0);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
  (void)hadc;
  capture_submit_block(CAPTURE_BLOCK_SAMPLES);
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef* hadc) {
  if (0 != (hadc->ErrorCode & HAL_ADC_ERROR_OVR)) {
    g_counters.adc_overruns++;
  }
}

/**
 * @brief Keep the UART fed from the ring buffer.
 *
 * Hands the largest contiguous run straight to DMA, so nothing is copied on
 * the way out. Called from the block loop while it waits, never from an
 * interrupt: only the transmit complete interrupt advances the tail, which
 * keeps the single reader rule intact.
 *
 * A transfer cannot already be in flight when stream_tx_busy is false, so the
 * test and the claim below it do not need to be atomic.
 */
static void stream_pump(void) {
  if (stream_tx_busy) {
    return;
  }

  size_t available = 0;
  const uint8_t* data = rb_peek_contiguous(&stream_ringbuf, &available);
  if (0 == available) {
    return;
  }

  stream_tx_length = available;
  stream_tx_busy = true;
  if (HAL_OK != HAL_UART_Transmit_DMA(&hlpuart1, data, (uint16_t)available)) {
    stream_tx_busy = false;
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
  (void)huart;
  rb_consume(&stream_ringbuf, stream_tx_length);
  stream_tx_busy = false;
}

/**
 * @brief Queue a frame, or count it as dropped.
 *
 * All or nothing: half a frame in the stream is worse than no frame. A short
 * write means the host is behind, which is counted and never waited on -
 * stalling the audio path for the UART would be the wrong trade.
 *
 * @param frame Frame bytes.
 * @param length Frame length, or 0 if the frame could not be built.
 */
static void stream_send(const uint8_t* frame, size_t length) {
  ASSERT(0 != length); // the frame buffer is sized from framing.h, so this cannot fail
  if (!rb_write_all(&stream_ringbuf, frame, length)) {
    g_counters.stream_drops++;
  }
}

/**
 * @brief Get the clock frequency driving the APB1 timers.
 *
 * @return uint32_t The timer clock frequency in Hz.
 */
static uint32_t get_apb1_timer_clock_freq(void) {
  RCC_ClkInitTypeDef clkconfig;
  uint32_t flash_latency;
  HAL_RCC_GetClockConfig(&clkconfig, &flash_latency);

  /* The APB timers run at twice PCLK whenever the bus is prescaled at all. */
  if (RCC_HCLK_DIV1 == clkconfig.APB1CLKDivider) {
    return HAL_RCC_GetPCLK1Freq();
  }
  return 2 * HAL_RCC_GetPCLK1Freq();
}

/**
 * @brief General error trap.
 *
 * @param file Source file of the failed check.
 * @param line Line of the failed check.
 */
[[noreturn]] void trap_error(const char* file, uint32_t line) {
  trap_file = file;
  trap_line = line;
  __disable_irq();
  while (1) {
    // Pause here for debugging.
  }
}

void assert_failed(uint8_t* file, uint32_t line) {
  trap_error((const char*)file, line);
}

int main(void) {
  {
    /**
     * @section HAL initialization.
     *
     * Must be executed before any HAL function calls.
     */
    ERROR_CHECK(HAL_Init());
  }

  {
    /**
     * @section System clock configuration.
     *
     * HSI16 / 4 * 85 / 2 = 170 MHz, the part's maximum. Boost mode is
     * mandatory above 150 MHz. The internal oscillator is used because the
     * board ships without a crystal populated for HSE.
     */
    ERROR_CHECK(HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST));

    RCC_OscInitTypeDef RCC_OscInitStruct;
    memset(&RCC_OscInitStruct, 0, sizeof(RCC_OscInitTypeDef));

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
    RCC_OscInitStruct.PLL.PLLN = 85;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    ERROR_CHECK(HAL_RCC_OscConfig(&RCC_OscInitStruct));

    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    memset(&RCC_ClkInitStruct, 0, sizeof(RCC_ClkInitTypeDef));

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    ERROR_CHECK(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4));
  }

  {
    /**
     * @section GPIO initialization.
     */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct;
    memset(&GPIO_InitStruct, 0, sizeof(GPIO_InitTypeDef));

    /* Microphone input. Analog, no pull: the module drives the pin. */
    GPIO_InitStruct.Pin = MIC_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(MIC_PORT, &GPIO_InitStruct);

    /* Heartbeat LED. */
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = LED_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);

    /* Sample clock output. Fast slew so the edge is worth looking at. */
    HAL_GPIO_WritePin(SCOPE_PORT, SCOPE_PIN, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = SCOPE_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(SCOPE_PORT, &GPIO_InitStruct);
  }

  {
    /**
     * @section Virtual com port.
     *
     * LPUART1 reaches the host through the ST-LINK. 921600 baud is what
     * host_receive.py expects and leaves the stream at about a third of the
     * link: 125 audio frames a second at 267 bytes, plus 20 status records
     * at 43, is 34 kB/s against the 92 kB/s the line carries.
     *
     * The kernel clock is PCLK1 at 170 MHz, so the LPUART divisor is
     * 256 * 170e6 / 921600 = 47222, well inside the 0x300 .. 0xFFFFF the
     * hardware allows, and the resulting baud is within 0.001% of nominal.
     */
    __HAL_RCC_LPUART1_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct;
    memset(&GPIO_InitStruct, 0, sizeof(GPIO_InitTypeDef));

    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = VCP_AF;

    GPIO_InitStruct.Pin = VCP_TX_PIN;
    HAL_GPIO_Init(VCP_TX_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = VCP_RX_PIN;
    HAL_GPIO_Init(VCP_RX_PORT, &GPIO_InitStruct);

    hlpuart1.Instance = VCP_UART;
    hlpuart1.Init.BaudRate = VCP_BAUDRATE;
    hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
    hlpuart1.Init.StopBits = UART_STOPBITS_1;
    hlpuart1.Init.Parity = UART_PARITY_NONE;
    hlpuart1.Init.Mode = UART_MODE_TX_RX;
    hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    ERROR_CHECK(HAL_UART_Init(&hlpuart1));

    /* The transmit complete callback arrives through the UART interrupt, not
     * the DMA one: the HAL enables TCIE when the DMA transfer ends and calls
     * back from there. Without this the stream would stall after one frame. */
    HAL_NVIC_SetPriority(LPUART1_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(LPUART1_IRQn);
  }

  {
    /**
     * @section DMA1 initialization.
     *
     * One channel, peripheral to memory, circular over both blocks. The
     * request is routed from ADC1 by DMAMUX.
     */
    __HAL_RCC_DMAMUX1_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    hdma_adc1.Instance = DMA1_Channel1;
    hdma_adc1.Init.Request = DMA_REQUEST_ADC1;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    hdma_adc1.Init.Priority = DMA_PRIORITY_HIGH;

    ERROR_CHECK(HAL_DMA_Init(&hdma_adc1));

    __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);

    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

    /* Stream out. Deliberately a lower priority than the capture path, so
     * that sending can never delay a block arriving. */
    hdma_lpuart1_tx.Instance = DMA1_Channel2;
    hdma_lpuart1_tx.Init.Request = DMA_REQUEST_LPUART1_TX;
    hdma_lpuart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_lpuart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_lpuart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_lpuart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_lpuart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_lpuart1_tx.Init.Mode = DMA_NORMAL;
    hdma_lpuart1_tx.Init.Priority = DMA_PRIORITY_LOW;

    ERROR_CHECK(HAL_DMA_Init(&hdma_lpuart1_tx));

    __HAL_LINKDMA(&hlpuart1, hdmatx, hdma_lpuart1_tx);

    HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  }

  {
    /**
     * @section TIM6 initialization.
     *
     * TIM6 exists to emit one TRGO pulse per sample period and nothing else.
     * Its update event is the ADC trigger, so the sample instants are placed
     * by hardware and never by the CPU.
     */
    __HAL_RCC_TIM6_CLK_ENABLE();

    const uint32_t timer_clock_hz = get_apb1_timer_clock_freq();
    ASSERT(0 == (timer_clock_hz % SAMPLE_RATE_HZ)); // otherwise the sample rate is not exactly 8 kHz

    htim6.Instance = TIM6;
    htim6.Init.Prescaler = 0;
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim6.Init.Period = (timer_clock_hz / SAMPLE_RATE_HZ) - 1;
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    ERROR_CHECK(HAL_TIM_Base_Init(&htim6));

    TIM_MasterConfigTypeDef master_config;
    memset(&master_config, 0, sizeof(TIM_MasterConfigTypeDef));

    master_config.MasterOutputTrigger = TIM_TRGO_UPDATE;
    master_config.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    ERROR_CHECK(HAL_TIMEx_MasterConfigSynchronization(&htim6, &master_config));
  }

  {
    /**
     * @section ADC1 initialization.
     *
     * Single channel, triggered externally, conversions handed straight to
     * DMA. The offset unit removes the microphone module's VDDA/2 idle level
     * in hardware: sign negative and saturation disabled together turn the
     * unsigned 0 .. 4095 range into a signed -2048 .. +2047 one.
     */
    __HAL_RCC_ADC12_CLK_ENABLE();

    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4; // 170 MHz / 4 = 42.5 MHz, inside the 60 MHz limit
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.GainCompensation = 0;
    hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc1.Init.LowPowerAutoWait = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE; // one conversion per trigger
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T6_TRGO;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
    hadc1.Init.DMAContinuousRequests = ENABLE; // circular DMA, so requests must never stop
    hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
    hadc1.Init.OversamplingMode = DISABLE;
    ERROR_CHECK(HAL_ADC_Init(&hadc1));

    ADC_MultiModeTypeDef multimode;
    memset(&multimode, 0, sizeof(ADC_MultiModeTypeDef));

    multimode.Mode = ADC_MODE_INDEPENDENT;
    ERROR_CHECK(HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode));

    ADC_ChannelConfTypeDef channel_config;
    memset(&channel_config, 0, sizeof(ADC_ChannelConfTypeDef));

    channel_config.Channel = MIC_ADC_CHANNEL;
    channel_config.Rank = ADC_REGULAR_RANK_1;
    channel_config.SamplingTime = MIC_ADC_SAMPLETIME;
    channel_config.SingleDiff = ADC_SINGLE_ENDED;
    channel_config.OffsetNumber = ADC_OFFSET_1;
    channel_config.Offset = ADC_HALF_SCALE;
    channel_config.OffsetSign = ADC_OFFSET_SIGN_NEGATIVE;
    channel_config.OffsetSaturation = DISABLE; // keep the negative half of the waveform
    ERROR_CHECK(HAL_ADC_ConfigChannel(&hadc1, &channel_config));

    ERROR_CHECK(HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED));

    HAL_NVIC_SetPriority(ADC1_2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
  }

  cycle_counter_ok = dwt_init();

  ASSERT(rb_init(&stream_ringbuf, stream_storage, sizeof(stream_storage)));

  {
    /**
     * @section Start capture.
     *
     * The ADC has to be armed before the timer starts, otherwise the first
     * triggers arrive with nowhere to put the data.
     */
    ERROR_CHECK(HAL_ADC_Start_DMA(&hadc1, (uint32_t*)capture_buffer, CAPTURE_BUFFER_SAMPLES));

#if defined(SCOPE_SAMPLE_CLOCK)
    /* Top priority, so the edge lands at the same point in every period
     * whatever else is running. The handler is six instructions. */
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
    ERROR_CHECK(HAL_TIM_Base_Start_IT(&htim6));
#else
    ERROR_CHECK(HAL_TIM_Base_Start(&htim6));
#endif
  }

  /* Frame staging. One audio frame is the larger of the two, so it serves
   * for both. Static, like everything else here. */
  static int16_t stream_block[CAPTURE_BLOCK_SAMPLES];
  static uint8_t stream_frame[AUDIO_FRAME_BYTES];

  uint32_t sequence = 0;
  uint32_t telemetry_due_ms = HAL_GetTick();
  uint32_t heartbeat_due_ms = HAL_GetTick();

  while (1) {
    const uint32_t now_ms = HAL_GetTick();

    /* Non-blocking drive of UART data. */
    stream_pump();

    if (capture_block_ready) {
      const volatile int16_t* block = &capture_buffer[capture_block_offset];
      capture_block_ready = false;

      BLOCK_TIMER_START();
      /* The filter, the gain stage and the AGC loop belong here. Until they
      * exist the captured block is sent as both channels, so the two halves of
      * the stereo capture are identical and the link can be judged on its own.
      *
      * The conversion result is 12 bit signed and the host reads samples as
      * int16, so it is scaled to full scale on the way out. Without this every
      * level the host reports would be 24 dB low, which would make the trim pot
      * calibration in the brief actively misleading. Multiply rather than
      * shift: a left shift of a negative value is not defined before C23. */
      for (size_t idx = 0; idx < CAPTURE_BLOCK_SAMPLES; idx++) {
        stream_block[idx] = (int16_t)(block[idx] * 16);
      }

      const size_t audio_bytes = frame_audio(stream_frame, sizeof(stream_frame),
                                            sequence, stream_block, stream_block);
      BLOCK_TIMER_END();

      sequence++;
      stream_send(stream_frame, audio_bytes);
    }

    /**
     * @section Emit telemetry periodically.
     * 
     */
    {
      if ((int32_t)(now_ms - telemetry_due_ms) >= 0) { // signed difference, so the tick wrap is harmless
        telemetry_due_ms = now_ms + TELEMETRY_INTERVAL_MS;

        telem_t telemetry;
        memset(&telemetry, 0, sizeof(telem_t));

        telemetry.timestamp_ms = now_ms;
        /* gain_db and the two RMS levels stay zero until there is a gain stage
        * to report. The counters below are real. */
        telemetry.worst_block_cycles = g_counters.worst_cycles;
        telemetry.dma_overruns = g_counters.dma_overruns;
        telemetry.adc_overruns = g_counters.adc_overruns;
        telemetry.stream_drops = g_counters.stream_drops;
        telemetry.blocks_processed = g_counters.blocks;

        stream_send(stream_frame, frame_telem(stream_frame, sizeof(stream_frame), &telemetry));
      }
    }

    /**
     * @section Drive heartbeat LED.
     */
    {
      if ((int32_t)(now_ms - heartbeat_due_ms) >= 0) {
        HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
        heartbeat_due_ms = now_ms + HEARTBEAT_INTERVAL_MS;
      }
    }
  }
}
