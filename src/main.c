/**
 * @file  main.c
 * @brief Signal generator with rotary encoder control, button-based ON/OFF handling,
 *        DAC sine output via DDS, serial frequency output and OLED visualization.
 *
 * This program runs on the STM32L476RG and generates a sine signal at the DAC output
 * (PA4). The output frequency can be adjusted with a rotary encoder using TIM3 in
 * encoder mode. A push button on PB3 toggles the signal output with controlled
 * startup and shutdown ramps. The currently selected frequency is shown on an SSD1306
 * OLED display via I2C1 and is also sent periodically over UART in Teleplot-compatible
 * format.
 *
 * Functional overview:
 * - Rotary encoder on PC6 / PC7 for frequency adjustment
 * - Push button on PB3 for controlled output enable / disable
 * - DAC output on PA4
 * - DDS-based sine generation using LUT and TIM6 interrupt
 * - Linear frequency-dependent amplitude ramp
 * - UART debug / Teleplot output
 * - SSD1306 OLED display via I2C1 for frequency / amplitude / OFF status
 *
 * Note:
 * The implementation of the I2C / OLED handling and parts of the DDS / LUT-based
 * sine generation were developed with AI-assisted support during implementation.
 * The final integration, adaptation, commenting and verification were performed
 * within the project context.
 */

#include "main.h"
#include "clock_.h"
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <string.h>

// ============================================================================
// Pin Definitions
// ============================================================================

// ---------- Encoder ----------
const uint8_t ENC_CHA_PIN = 6;  // PC6 = TIM3_CH1 (PIN 4)
const uint8_t ENC_CHB_PIN = 7;  // PC7 = TIM3_CH2 (PIN 19)

// ---------- Button and LED ----------
const uint8_t BUTTON_PIN = 3;   // PB3 (PIN 31)
const uint8_t LED_PIN    = 5;   // PA5 (PIN 11)

// ---------- I2C Pins (OLED Display) ----------
const uint8_t OLED_SCL_PIN = 8; // PB8 (PIN 3)
const uint8_t OLED_SDA_PIN = 9; // PB9 (PIN 5)

// ---------- Output ----------
const uint8_t DAC_OUT_PIN = 4;  // PA4 / DAC output (PIN 32)

// ============================================================================
// Timing / UI Settings
// ============================================================================

#define DEBOUNCE_MS             50u
#define USART_PERIOD_MS         50u
#define OLED_UPDATE_PERIOD_MS   500u // Keep this value moderate; overly frequent OLED updates may introduce output disturbances

// ============================================================================
// Frequency Limits
// ============================================================================

#define FREQ_MIN 30.0f
#define FREQ_MAX 1000.0f

// ============================================================================
// Ramp Settings
// ============================================================================

#define RAMP_STEPS         50u
#define RAMP_DELAY_TICKS   10e5  // tuneable (~2s shutdown)

// ============================================================================
// Encoder Settings
// ============================================================================

#define FREQ_PER_REV         10.0f  // Chosen user sensitivity 
#define ENC_PULSES_PER_REV   24.0f  // Encoder: 24 pulses per 360° rotation (datasheet)
#define ENC_COUNTS_PER_REV   (ENC_PULSES_PER_REV * 4.0f)  // Encoder-Mode 3 means, we count every flank (x4)
#define FREQ_STEP_PER_COUNT  (FREQ_PER_REV / ENC_COUNTS_PER_REV)  // 10 Hz per rotation 

// ============================================================================
// DDS / DAC Settings
// ============================================================================

#define SAMPLE_FREQUENCY  20000u
#define TIM6_ARR_VALUE    3999u     // 80 MHz / (3999 + 1) = 20 kHz

#define LUT_SIZE          256u
#define LUT_SCALE         1000    // LUT contains signed values in range -LUT_SCALE ... +LUT_SCALE
#define PI_F              3.14159265358979f

#define DAC_MAX_CODE      4095u
#define DAC_OFFSET_CODE   1861u   // ~1.5 V at 3.3 V reference
#define DAC_AMPL_CODE     1241u   // ~1.0 V peak

// Linear amplitude ramp:
// rough start value derived from approx. 0.8 Vrms at low frequency
#define DAC_AMPL_MIN_CODE 216u

// ============================================================================
// OLED / I2C Settings
// ============================================================================

#define OLED_WIDTH               128u
#define OLED_HEIGHT              64u
#define OLED_BUFFER_SIZE         (OLED_WIDTH * OLED_HEIGHT / 8u)

#define OLED_CONTROL_CMD         0x00u
#define OLED_CONTROL_DATA        0x40u

#define OLED_I2C_ADDR            0x3Cu
#define I2C1_TIMING_100KHZ_80MHZ 0x10909CECu  // Practical timing value for I2C1 @ 80 MHz -> Standard Mode (~100 kHz)

// ============================================================================
// Debug Macro
// ============================================================================

#define DEBUG
#ifdef DEBUG
#define LOG(msg...) printf(msg);
#else
#define LOG(msg...);
#endif

// ============================================================================
// Global State Variables
// ============================================================================

// ---------- Button / LED / Timebase ----------
volatile uint8_t button_pressed    = 0u;  // Set by EXTI interrupt, handled in main loop
volatile uint8_t led_state         = 0u;  // Stores current LED state

volatile uint32_t ms_ticks         = 0u; // System time base in milliseconds
volatile uint32_t last_button_time = 0u; // Timestamp of last accepted button press

// ---------- Encoder / Frequency ----------
volatile int32_t encoder_last_cnt = 0;
volatile int32_t encoder_delta    = 0;
volatile float   frequency_hz     = FREQ_MIN;

// ---------- Output State ----------
volatile uint8_t output_enabled = 1u;    // used for ON/OFF function (DAC)

// ---------- Serial ----------
volatile uint32_t last_plot_time = 0u;

// ---------- DDS / DAC ----------
volatile uint32_t phase_acc       = 0u;
volatile uint32_t tuning_word     = 0u;
volatile uint16_t dac_output_code = 1861u;  // current DAC output (12-bit-code) for serial output

volatile uint16_t dac_ampl_current_code = DAC_AMPL_CODE;
volatile uint16_t dac_ampl_target_code  = DAC_AMPL_CODE;

int16_t sine_lut[LUT_SIZE];

// ---------- OLED ----------
uint8_t oled_buffer[OLED_BUFFER_SIZE];
volatile uint32_t last_oled_update_time = 0u;

// ============================================================================
// Function Prototypes
// ============================================================================

// ---------- Utility ----------
uint32_t get_amplitude_percent_value(void);
uint16_t amplitude_code_from_frequency(float freq);

// ---------- Button / LED ----------
void button_setup(void);
void button(void);

// ---------- Encoder ----------
void encoder_setup(void);
void encoder_update(void);

// ---------- Serial ----------
void serial_setup(void);
void serial_output(void);

// ---------- Startup / Shutdown ----------
void shutdown(void);
void start_up(void);

// ---------- DAC / DDS ----------
void dac_setup(void);
void dac_write(uint16_t value);
void generate_lut(void);
void update_tuning_word(float freq);
void timer6_setup(void);

// ---------- I2C / OLED ----------
void i2c1_setup(void);
uint8_t i2c1_write(uint8_t addr7, const uint8_t *data, uint16_t len);

void oled_send_command(uint8_t cmd);
void oled_send_data_block(const uint8_t *data, uint16_t len);
void oled_init(void);
void oled_clear_buffer(void);
void oled_update(void);

// ---------- OLED Graphics / UI ----------
void oled_draw_pixel(uint8_t x, uint8_t y, uint8_t color);
void oled_draw_char_big(uint8_t x, uint8_t y, char c);
void oled_draw_string_big(uint8_t x, uint8_t y, const char *str);
void oled_show_frequency_or_off(void);
void oled_task(void);

// ============================================================================
// Utility
// ============================================================================

/**
 * @brief Returns amplitude in percent relative to DAC_AMPL_CODE.
 * @return Amplitude percentage in range 0 ... 100
 */
uint32_t get_amplitude_percent_value(void)
{
  if (DAC_AMPL_CODE == 0u)
  {
    return 0u;
  }

  return (uint32_t)(((uint32_t)dac_ampl_current_code * 100u + (DAC_AMPL_CODE / 2u)) / DAC_AMPL_CODE);
}

/**
 * @brief Returns a frequency-dependent DAC amplitude code.
 *        First simple approach: linear ramp from low amplitude at 30 Hz
 *        to maximum amplitude at 1000 Hz.
 *
 * @param freq Frequency in Hz
 * @return DAC amplitude code
 */
uint16_t amplitude_code_from_frequency(float freq)
{
  const float f_min = FREQ_MIN;
  const float f_max = FREQ_MAX;

  const float ampl_min = (float)DAC_AMPL_MIN_CODE;
  const float ampl_max = (float)DAC_AMPL_CODE;

  float k;
  float ampl;

  if (freq <= f_min)
  {
    return (uint16_t)(ampl_min + 0.5f);
  }

  if (freq >= f_max)
  {
    return (uint16_t)(ampl_max + 0.5f);
  }

  k = (freq - f_min) / (f_max - f_min);
  ampl = ampl_min + k * (ampl_max - ampl_min);

  return (uint16_t)(ampl + 0.5f);
}

// ============================================================================
// Interrupt Handlers
// ============================================================================

/**
 * @brief SysTick interrupt handler.
 *        Increments the global millisecond counter every 1 ms.
 */
void SysTick_Handler(void)
{
  ms_ticks++;
}

/**
 * @brief External interrupt handler for EXTI line 3.
 *        Triggered on a falling edge at PB3 (active LOW button).
 *        Applies a simple software debounce using the SysTick time base.
 */
void EXTI3_IRQHandler(void)
{
  if (EXTI->PR1 & (1u << 3))  // Check if EXTI line 3 caused the interrupt
  {
    EXTI->PR1 = (1u << 3);    // Clear EXTI3 pending flag by writing 1

    // Accept button press only if debounce time has elapsed
    if ((ms_ticks - last_button_time) >= DEBOUNCE_MS)
    {
      button_pressed = 1u;
      last_button_time = ms_ticks;
    }
  }
}

// ============================================================================
// Button / LED
// ============================================================================

/**
 * @brief Configures button input, LED output and EXTI interrupt.
 *        - PB3: button input with external pull-up
 *        - PA5: onboard LED output
 *        - EXTI3: interrupt on falling edge for PB3
 */
void button_setup(void)
{
  // ---------- Enable peripheral clocks ----------
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;   // Enable GPIOA clock (LED)
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;   // Enable GPIOB clock (button)
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;  // Enable SYSCFG clock (needed for EXTI routing)

  // ---------- PB3 = button input with external pull-up ----------
  GPIOB->MODER &= ~(0b11 << (2 * BUTTON_PIN));  // input mode
  GPIOB->PUPDR &= ~(0b11 << (2 * BUTTON_PIN));  // Clear pull-up/pull-down bits (external pull-up)

  // ---------- PA5 = LED output ----------
  GPIOA->MODER  &= ~(0b11 << (2 * LED_PIN));    // Clear MODER bits first
  GPIOA->MODER  |=  (0b01 << (2 * LED_PIN));    // Set output mode (01)

  GPIOA->OTYPER &= ~(1u << LED_PIN);            // Set output type to push-pull (0)

  GPIOA->BSRR = (1u << (LED_PIN + 16));         // Reset PA5 -> LED initially OFF

  // ---------- Route PB3 to EXTI line 3 ----------
  SYSCFG->EXTICR[0] &= ~(0b1111 << 12);           // Clear EXTI3 port selection bits
  SYSCFG->EXTICR[0] |=  (0b0001 << 12);           // Select Port B for EXTI3

  // ---------- Configure EXTI line 3 ----------
  EXTI->IMR1  |=  (1u << 3);                    // Unmask EXTI3 interrupt
  EXTI->FTSR1 |=  (1u << 3);                    // Enable falling edge trigger
  EXTI->RTSR1 &= ~(1u << 3);                    // Disable rising edge trigger
  EXTI->PR1   =   (1u << 3);                    // Clear any old pending EXTI3 flag

  // ---------- Enable EXTI3 interrupt in NVIC ----------
  NVIC_SetPriority(EXTI3_IRQn, 2);
  NVIC_EnableIRQ(EXTI3_IRQn);
}

/**
 * @brief Processes button events in the main loop.
 *        Button performs controlled amplitude shutdown / startup.
 */
void button(void)
{
  if (button_pressed)
  {
    button_pressed = 0u;

    if (output_enabled)
    {
      shutdown();
    }
    else
    {
      start_up();
    }
  }
}

// ============================================================================
// Encoder
// ============================================================================

/**
 * @brief Configure TIM3 in encoder mode on PC6 / PC7.
 */
void encoder_setup(void)
{
  // ---------- Enable peripheral clocks ----------
  RCC->AHB2ENR  |= RCC_AHB2ENR_GPIOCEN; // IO port C clock enable (encoder pins)
  RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN; // TIM3 timer clock enable (encoder interface)

  // ---------- PC6, PC7 - Alternate Function ----------
  GPIOC->MODER &= ~(0b11 << (2 * ENC_CHA_PIN));
  GPIOC->MODER &= ~(0b11 << (2 * ENC_CHB_PIN));
  GPIOC->MODER |=  (0b10 << (2 * ENC_CHA_PIN)); // Alternate function mode
  GPIOC->MODER |=  (0b10 << (2 * ENC_CHB_PIN)); // Alternate function mode

  // ---------- Pull-Up Configuration ----------
  GPIOC->PUPDR &= ~(0b11 << (2 * ENC_CHA_PIN));
  GPIOC->PUPDR &= ~(0b11 << (2 * ENC_CHB_PIN));
  GPIOC->PUPDR |=  (0b01 << (2 * ENC_CHA_PIN));
  GPIOC->PUPDR |=  (0b01 << (2 * ENC_CHB_PIN));

  // ---------- High Speed Configuration ----------
  GPIOC->OSPEEDR &= ~(0b11 << (2 * ENC_CHA_PIN));
  GPIOC->OSPEEDR &= ~(0b11 << (2 * ENC_CHB_PIN));
  GPIOC->OSPEEDR |=  (0b10 << (2 * ENC_CHA_PIN)); // High speed
  GPIOC->OSPEEDR |=  (0b10 << (2 * ENC_CHB_PIN)); // High speed

  // ---------- AF2 (Alternate Function) for TIM3 on PC6 / PC7 ----------
  GPIOC->AFR[0] &= ~(0b1111 << (4 * ENC_CHA_PIN));
  GPIOC->AFR[0] &= ~(0b1111 << (4 * ENC_CHB_PIN));
  GPIOC->AFR[0] |=  (0b0010 << (4 * ENC_CHA_PIN));  // AF2 ... TIM3_CH1
  GPIOC->AFR[0] |=  (0b0010 << (4 * ENC_CHB_PIN));  // AF2 ... TIM3_CH2

  // ---------- Reset TIM3 ----------
  RCC->APB1RSTR1 |=  RCC_APB1RSTR1_TIM3RST; // Reset TIM3
  RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_TIM3RST;

  TIM3->CR1   = 0;  // Control register 1
  TIM3->CR2   = 0;  // Control register 2
  TIM3->SMCR  = 0;  // Slave mode control register
  TIM3->DIER  = 0;  // DMA/Interrupt enable register
  TIM3->CCER  = 0;  // Capture/Compare enable register
  TIM3->CCMR1 = 0;  // Capture/Compare mode register 1

  // Prescaler must be 0 for encoder mode
  TIM3->PSC = 0;

  // Full 16-bit range (auto-reload register)
  TIM3->ARR = 0xFFFF;

  // Start in the middle for clean signed delta handling
  TIM3->CNT = 0x8000;
  encoder_last_cnt = 0x8000;

  // CC1 mapped to TI1, CC2 mapped to TI2
  TIM3->CCMR1 |= (0b01 << 0); // Capture/Compare 1 selection
  TIM3->CCMR1 |= (0b01 << 8); // Capture/Compare 2 selection

  // Small digital input filter
  TIM3->CCMR1 |= (0b0011 << 4);  // Input capture 1 filter (0011: fSAMPLING=fCK_INT, N=8)
  TIM3->CCMR1 |= (0b0011 << 12); // Input capture 2 filter (0011: fSAMPLING=fCK_INT, N=8)

  // Non-inverted polarity
  TIM3->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC1NP);
  TIM3->CCER &= ~(TIM_CCER_CC2P | TIM_CCER_CC2NP);

  // Encoder mode 3
  TIM3->SMCR |= (0b0011 << 0);

  // Enable counter
  TIM3->CR1 |= TIM_CR1_CEN;
}

/**
 * @brief Reads encoder movement and updates the selected output frequency.
 *        The selectable frequency is limited to integer values in the range
 *        30 ... 1000 Hz.
 *
 *        When the output is enabled, the new frequency and amplitude are
 *        applied immediately. When the output is disabled, only the selected
 *        frequency value is updated.
 */
void encoder_update(void)
{
  int32_t cnt_now = (int32_t)TIM3->CNT;
  encoder_delta   = (int16_t)(cnt_now - encoder_last_cnt);

  if (encoder_delta != 0)
  {
    encoder_last_cnt = cnt_now;

    frequency_hz += ((float)encoder_delta * FREQ_STEP_PER_COUNT);

    // Allow only integer output frequencies
    frequency_hz = roundf(frequency_hz);

    if (frequency_hz < FREQ_MIN)
    {
      frequency_hz = FREQ_MIN;
    }

    if (frequency_hz > FREQ_MAX)
    {
      frequency_hz = FREQ_MAX;
    }

    // Apply directly only while output is active
    if (output_enabled)
    {
      update_tuning_word(frequency_hz);
      dac_ampl_target_code  = amplitude_code_from_frequency(frequency_hz);
      dac_ampl_current_code = dac_ampl_target_code;
    }
  }
}

// ============================================================================
// Serial
// ============================================================================

/**
 * @brief Setup serial debug output.
 */
void serial_setup(void)
{
  Init_Debug_UART();
}

/**
 * @brief Sends Teleplot-compatible frequency and amplitude values periodically
 *        over UART.
 */
void serial_output(void)
{
  if ((ms_ticks - last_plot_time) >= USART_PERIOD_MS)
  {
    last_plot_time = ms_ticks;

    uint32_t freq_int = (uint32_t)frequency_hz;
    uint32_t ampl_pct = get_amplitude_percent_value();

    LOG(">Freq:%lu\r\n", (unsigned long)freq_int);
    LOG(">AmplPercent:%lu\r\n", (unsigned long)ampl_pct);
  }
}

// ============================================================================
// DAC / DDS / TIM6
// ============================================================================

/**
 * @brief Configures DAC channel 1 on PA4.
 *        PA4 is used as analog output pin DAC1_OUT1.
 */
void dac_setup(void)
{
  // ---------- Enable peripheral clocks ----------
  RCC->AHB2ENR  |= RCC_AHB2ENR_GPIOAEN; // GPIOA clock enable
  RCC->APB1ENR1 |= RCC_APB1ENR1_DAC1EN; // DAC1 clock enable

  // ---------- PA4 - Analog Mode ----------
  GPIOA->MODER &= ~(0b11 << (2 * DAC_OUT_PIN));
  GPIOA->MODER |=  (0b11 << (2 * DAC_OUT_PIN)); // Analog mode

  // ---------- No Pull-Up / Pull-Down ---------- 
  GPIOA->PUPDR &= ~(0b11 << (2 * DAC_OUT_PIN));

  // ---------- DAC Channel 1 Configuration ----------
  DAC1->CR &= ~(1u << 2); // TEN1 = 0 ... no trigger, direct write
  DAC1->CR &= ~(1u << 1); // BOFF1 = 0 ... output buffer enabled
  DAC1->CR |=  (1u << 0); // EN1 = 1 ... enable DAC channel 1

  dac_ampl_current_code = amplitude_code_from_frequency(frequency_hz);
  dac_ampl_target_code  = dac_ampl_current_code;

  // dac_write(DAC_OFFSET_CODE);
  dac_output_code = DAC_OFFSET_CODE;
  DAC1->DHR12R1   = dac_output_code;
}

/**
 * @brief Write a 12-bit value to DAC channel 1.
 * @param value DAC value in range 0 ... 4095
 */
void dac_write(uint16_t value)
{
  value &= 0x0FFF;  // Limit to 12 bit
  dac_output_code = value;
  DAC1->DHR12R1   = dac_output_code;  // 12-bit right-aligned data for channel 1
}

/**
 * @brief Generates a sine lookup table with signed values in the range
 *        -LUT_SCALE ... +LUT_SCALE.
 */
void generate_lut(void)
{
  for (uint32_t i = 0; i < LUT_SIZE; i++)
  {
    float angle  = (2.0f * PI_F * (float)i) / (float)LUT_SIZE;
    float s      = sinf(angle);
    float scaled = s * (float)LUT_SCALE;
    
    // Rounding to the nearest integer value
    if (scaled >= 0.0f)
    {
      sine_lut[i] = (int16_t)(scaled + 0.5f);
    }
    else
    {
      sine_lut[i] = (int16_t)(scaled - 0.5f);
    }
  }
}

/**
 * @brief Update DDS tuning word from desired output frequency.
 *        Internal DDS path may use 0 Hz for shutdown / OFF.
 * @param freq Desired output frequency in Hz.
 */
void update_tuning_word(float freq)
{
  if (freq < 0.0f)
  {
    freq = 0.0f;
  }

  if (freq > FREQ_MAX)
  {
    freq = FREQ_MAX;
  }

  double tw = ((double)freq * 4294967296.0) / (double)SAMPLE_FREQUENCY;

  // Rounding to the nearest integer value
  if (tw >= 0.0)
  {
    tuning_word = (uint32_t)(tw + 0.5);
  }
  else
  {
    tuning_word = (uint32_t)(tw - 0.5);
  }
}

/**
 * @brief Configure TIM6 to generate update interrupts at 20 kHz.
 */
void timer6_setup(void)
{
  // ---------- Enable peripheral clock ----------
  RCC->APB1ENR1 |= RCC_APB1ENR1_TIM6EN;

  // ---------- Reset basic timer configuration ----------
  TIM6->CR1 = 0;  // clear control register
  TIM6->PSC = 0;  // prescaler = 0 (timer runs directly at 80 MHz)

  // ---------- Set auto-reload value ----------
  // 80 MHz / (3999 + 1) = 20 kHz
  TIM6->ARR = TIM6_ARR_VALUE;

  TIM6->EGR  = TIM_EGR_UG;    // Update generation
  TIM6->SR   = 0;             // Clear pending status flags
  TIM6->DIER |= TIM_DIER_UIE; // Enable update interrupt
  TIM6->CR1  |= TIM_CR1_CEN;  // counter enabled (start timer)

  // ---------- Enable TIM6 interrupt in NVIC ----------
  NVIC_SetPriority(TIM6_DAC_IRQn, 1);
  NVIC_EnableIRQ(TIM6_DAC_IRQn);
}

/**
 * @brief TIM6 interrupt handler.
 *
 * This interrupt is executed once per sample period.
 * It performs one DDS step and writes one new value to the DAC.
 *
 * Sequence:
 * 1. Clear timer update flag
 * 2. Advance DDS phase accumulator
 * 3. Use the upper 8 bits as LUT index (for 256-entry LUT)
 * 4. Read signed sine sample from LUT
 * 5. Scale the LUT sample with the current DAC amplitude and add the DAC offset
 * 6. Limit the result to the valid 12-bit DAC range
 * 7. Write the final value to the DAC
 */
void TIM6_DAC_IRQHandler(void)
{
  // Check if update interrupt flag is set
  if (TIM6->SR & TIM_SR_UIF)
  {
    // Clear update interrupt flag
    TIM6->SR &= ~TIM_SR_UIF;

    // DDS phase step
    phase_acc += tuning_word;

    // ---------- Convert phase to LUT index ----------
    // LUT_SIZE = 256 -> need 8 index bits
    // Therefore: use the upper 8 bits of the 32-bit phase accumulator
    uint8_t index   = (uint8_t)(phase_acc >> 24);

    // ---------- Get normalized sine value from LUT ----------
    int32_t lut_val = sine_lut[index];

    // ---------- Scale LUT value to DAC range ----------
    //
    // dac = offset + amplitude * normalized_sine
    //
    // with normalized_sine = lut_val / LUT_SCALE
    int32_t dac_val = (int32_t)DAC_OFFSET_CODE
                    + ((int32_t)dac_ampl_current_code * lut_val) / LUT_SCALE;

    // ---------- Clamp result to valid 12-bit DAC range ----------
    if (dac_val < 0)
    {
      dac_val = 0;
    }

    if (dac_val > DAC_MAX_CODE)
    {
      dac_val = DAC_MAX_CODE;
    }

    // ---------- Write sample to DAC and update debug variable ----------
    dac_write((uint16_t)dac_val);
  }
}

// ============================================================================
// Startup / Shutdown Control
// ============================================================================

/**
 * @brief Ramps the output amplitude down to zero while keeping the current
 *        frequency constant.
 *
 *        After the ramp-down, only the DAC offset remains at the output.
 */
void shutdown(void)
{
  uint16_t start_ampl = dac_ampl_current_code;

  // ---------- Controlled ramp-down ----------
  for (uint32_t step = 0; step <= RAMP_STEPS; step++)
  {
    float k = (float)step / (float)RAMP_STEPS;

    // ---------- Amplitude: current -> 0 ----------
    dac_ampl_current_code = (uint16_t)((float)start_ampl * (1.0f - k) + 0.5f);

    delay(RAMP_DELAY_TICKS);
  }

  // ---------- Final OFF state ----------
  dac_ampl_current_code = 0u;
  dac_ampl_target_code  = amplitude_code_from_frequency(frequency_hz);

  output_enabled = 0u;

  led_state = 0u;
  GPIOA->BSRR = (1u << (LED_PIN + 16));   // LED OFF

  oled_show_frequency_or_off();
}

/**
 * @brief Starts the signal output at the minimum frequency (30 Hz) and performs
 *        a controlled amplitude ramp-up.
 */
void start_up(void)
{
  // ---------- Start directly at 30 Hz ----------
  frequency_hz = FREQ_MIN;
  update_tuning_word(frequency_hz);

  // ---------- Start from zero amplitude ----------
  dac_ampl_current_code = 0u;
  dac_ampl_target_code  = amplitude_code_from_frequency(frequency_hz);

  phase_acc = 0u;
  output_enabled = 1u;

  led_state = 1u;
  GPIOA->BSRR = (1u << LED_PIN);   // LED ON

  oled_show_frequency_or_off();

  // ---------- Controlled ramp-up ----------
  for (uint32_t step = 0; step <= RAMP_STEPS; step++)
  {
    float k = (float)step / (float)RAMP_STEPS;

    dac_ampl_current_code = (uint16_t)((float)dac_ampl_target_code * k + 0.5f);

    delay(RAMP_DELAY_TICKS);
  }

  dac_ampl_current_code = dac_ampl_target_code;
  oled_show_frequency_or_off();
}

// ============================================================================
// I2C1 + OLED
// ============================================================================

/**
 * @brief Configure I2C1 on PB8 (SCL) and PB9 (SDA).
 */
void i2c1_setup(void)
{
  // ---------- Enable peripheral clocks ----------
  RCC->AHB2ENR  |= RCC_AHB2ENR_GPIOBEN;
  RCC->APB1ENR1 |= RCC_APB1ENR1_I2C1EN;

  // ---------- PB8, PB9 - Alternate Function ----------
  GPIOB->MODER &= ~(0b11 << (2 * OLED_SCL_PIN));
  GPIOB->MODER &= ~(0b11 << (2 * OLED_SDA_PIN));
  GPIOB->MODER |=  (0b10 << (2 * OLED_SCL_PIN));
  GPIOB->MODER |=  (0b10 << (2 * OLED_SDA_PIN));

  // ---------- Open-Drain ----------
  GPIOB->OTYPER |= (1u << OLED_SCL_PIN);
  GPIOB->OTYPER |= (1u << OLED_SDA_PIN);

  // ---------- Pull-Up ----------
  GPIOB->PUPDR &= ~(0b11 << (2 * OLED_SCL_PIN));
  GPIOB->PUPDR &= ~(0b11 << (2 * OLED_SDA_PIN));
  GPIOB->PUPDR |=  (0b01 << (2 * OLED_SCL_PIN));
  GPIOB->PUPDR |=  (0b01 << (2 * OLED_SDA_PIN));

  // ---------- High Speed ----------
  GPIOB->OSPEEDR &= ~(0b11 << (2 * OLED_SCL_PIN));
  GPIOB->OSPEEDR &= ~(0b11 << (2 * OLED_SDA_PIN));
  GPIOB->OSPEEDR |=  (0b10 << (2 * OLED_SCL_PIN));
  GPIOB->OSPEEDR |=  (0b10 << (2 * OLED_SDA_PIN));

  // ---------- AF4 for I2C1 ----------
  GPIOB->AFR[1] &= ~(0b1111 << (4 * (OLED_SCL_PIN - 8)));
  GPIOB->AFR[1] &= ~(0b1111 << (4 * (OLED_SDA_PIN - 8)));
  GPIOB->AFR[1] |=  (0b0100 << (4 * (OLED_SCL_PIN - 8)));
  GPIOB->AFR[1] |=  (0b0100 << (4 * (OLED_SDA_PIN - 8)));

  // ---------- Reset I2C1 ----------
  RCC->APB1RSTR1 |=  RCC_APB1RSTR1_I2C1RST;
  RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_I2C1RST;

  // ---------- Disable before config ----------
  I2C1->CR1 &= ~I2C_CR1_PE;

  // ---------- Timing ----------
  I2C1->TIMINGR = I2C1_TIMING_100KHZ_80MHZ;

  // ---------- Address registers ----------
  I2C1->OAR1 = 0;
  I2C1->OAR2 = 0;

  // ---------- Enable I2C ----------
  I2C1->CR1 |= I2C_CR1_PE;
}

/**
 * @brief Write byte block to 7-bit I2C slave.
 * @return 1 on success, 0 on error
 */
uint8_t i2c1_write(uint8_t addr7, const uint8_t *data, uint16_t len)
{
  uint32_t timeout;

  if ((data == 0) || (len == 0) || (len > 255u))
  {
    return 0u;
  }

  timeout = 100000u;
  while ((I2C1->ISR & I2C_ISR_BUSY) && timeout--)
  {
  }
  if (timeout == 0u)
  {
    return 0;
  }

  I2C1->ICR = I2C_ICR_STOPCF | I2C_ICR_NACKCF | I2C_ICR_BERRCF | I2C_ICR_ARLOCF;

  I2C1->CR2 = 0;
  I2C1->CR2 |= ((uint32_t)addr7 << 1);
  I2C1->CR2 |= ((uint32_t)len << 16);
  I2C1->CR2 |= I2C_CR2_AUTOEND;
  I2C1->CR2 |= I2C_CR2_START;

  for (uint16_t i = 0; i < len; i++)
  {
    timeout = 100000u;

    while (!(I2C1->ISR & I2C_ISR_TXIS))
    {
      if (I2C1->ISR & I2C_ISR_NACKF)
      {
        I2C1->ICR = I2C_ICR_NACKCF;
        return 0u;
      }

      if (timeout-- == 0u)
      {
        return 0u;
      }
    }

    I2C1->TXDR = data[i];
  }

  timeout = 100000u;
  while (!(I2C1->ISR & I2C_ISR_STOPF))
  {
    if (timeout-- == 0u)
    {
      return 0u;
    }
  }

  I2C1->ICR = I2C_ICR_STOPCF;
  return 1u;
}

void oled_send_command(uint8_t cmd)
{
  uint8_t packet[2];
  packet[0] = OLED_CONTROL_CMD;
  packet[1] = cmd;
  i2c1_write(OLED_I2C_ADDR, packet, 2);
}

void oled_send_data_block(const uint8_t *data, uint16_t len)
{
  uint8_t packet[17];
  packet[0] = OLED_CONTROL_DATA;

  while (len > 0u)
  {
    uint16_t chunk = (len > 16u) ? 16u : len;

    for (uint16_t i = 0; i < chunk; i++)
    {
      packet[1 + i] = data[i];
    }

    i2c1_write(OLED_I2C_ADDR, packet, (uint16_t)(chunk + 1u));

    data += chunk;
    len  -= chunk;
  }
}

/**
 * @brief Initialize SSD1306 OLED.
 */
void oled_init(void)
{
  delay(200000);

  oled_send_command(0xAE); // Display OFF

  oled_send_command(0xD5); // Set display clock divide ratio / oscillator frequency
  oled_send_command(0x80);

  oled_send_command(0xA8); // Set multiplex ratio
  oled_send_command(0x3F); // 64 MUX

  oled_send_command(0xD3); // Set display offset
  oled_send_command(0x00);

  oled_send_command(0x40); // Set display start line

  oled_send_command(0x8D); // Charge pump
  oled_send_command(0x14); // Enable charge pump

  oled_send_command(0x20); // Memory addressing mode
  oled_send_command(0x00); // Horizontal addressing mode

  oled_send_command(0xA1); // Segment remap
  oled_send_command(0xC8); // COM scan direction remapped

  oled_send_command(0xDA); // COM pins hardware configuration
  oled_send_command(0x12);

  oled_send_command(0x81); // Contrast control
  oled_send_command(0x66);

  oled_send_command(0xD9); // Pre-charge period
  oled_send_command(0xF1);

  oled_send_command(0xDB); // VCOMH deselect level
  oled_send_command(0x30);

  oled_send_command(0xA4); // Entire display ON follows RAM
  oled_send_command(0xA6); // Normal display

  oled_clear_buffer();
  oled_update();

  oled_send_command(0xAF); // Display ON
}

void oled_clear_buffer(void)
{
  memset(oled_buffer, 0, sizeof(oled_buffer));
}

void oled_update(void)
{
  oled_send_command(0x21); // Set column address
  oled_send_command(0x00);
  oled_send_command(0x7F);

  oled_send_command(0x22); // Set page address
  oled_send_command(0x00);
  oled_send_command(0x07);

  oled_send_data_block(oled_buffer, OLED_BUFFER_SIZE);
}

// ============================================================================
// OLED Graphics / UI
// ============================================================================

void oled_draw_pixel(uint8_t x, uint8_t y, uint8_t color)
{
  if ((x >= OLED_WIDTH) || (y >= OLED_HEIGHT))
  {
    return;
  }

  uint16_t index = x + ((uint16_t)(y / 8u) * OLED_WIDTH);

  if (color)
  {
    oled_buffer[index] |= (1u << (y & 7u));
  }
  else
  {
    oled_buffer[index] &= ~(1u << (y & 7u));
  }
}

// ---------- Minimal font for required characters only ----------
typedef struct
{
  char c;
  uint8_t data[5];
} Font5x7;

static const Font5x7 font5x7[] =
{
  { ' ', {0x00,0x00,0x00,0x00,0x00} },
  { ':', {0x00,0x36,0x36,0x00,0x00} },
  { '%', {0x63,0x13,0x08,0x64,0x63} },

  { '0', {0x3E,0x51,0x49,0x45,0x3E} },
  { '1', {0x00,0x42,0x7F,0x40,0x00} },
  { '2', {0x42,0x61,0x51,0x49,0x46} },
  { '3', {0x21,0x41,0x45,0x4B,0x31} },
  { '4', {0x18,0x14,0x12,0x7F,0x10} },
  { '5', {0x27,0x45,0x45,0x45,0x39} },
  { '6', {0x3C,0x4A,0x49,0x49,0x30} },
  { '7', {0x01,0x71,0x09,0x05,0x03} },
  { '8', {0x36,0x49,0x49,0x49,0x36} },
  { '9', {0x06,0x49,0x49,0x29,0x1E} },

  { 'A', {0x7E,0x11,0x11,0x11,0x7E} },
  { 'E', {0x7F,0x49,0x49,0x49,0x41} },
  { 'F', {0x7F,0x09,0x09,0x09,0x01} },
  { 'H', {0x7F,0x08,0x08,0x08,0x7F} },
  { 'L', {0x7F,0x40,0x40,0x40,0x40} },
  { 'M', {0x7F,0x02,0x0C,0x02,0x7F} },
  { 'O', {0x3E,0x41,0x41,0x41,0x3E} },
  { 'P', {0x7F,0x09,0x09,0x09,0x06} },
  { 'Q', {0x3E,0x41,0x51,0x21,0x5E} },
  { 'R', {0x7F,0x09,0x19,0x29,0x46} },

  { 'z', {0x44,0x64,0x54,0x4C,0x44} }
};

static const uint8_t *font_find(char c)
{
  for (uint32_t i = 0; i < (sizeof(font5x7) / sizeof(font5x7[0])); i++)
  {
    if (font5x7[i].c == c)
    {
      return font5x7[i].data;
    }
  }

  return font5x7[0].data; // fallback = space
}

/**
 * @brief Draw one character in 2x scaled size.
 */
void oled_draw_char_big(uint8_t x, uint8_t y, char c)
{
  const uint8_t *glyph = font_find(c);

  for (uint8_t col = 0; col < 5; col++)
  {
    uint8_t line = glyph[col];

    for (uint8_t row = 0; row < 7; row++)
    {
      if (line & (1u << row))
      {
        // 2x horizontal + 2x vertical scaling
        oled_draw_pixel(x + (2u * col) + 0u, y + (2u * row) + 0u, 1u);
        oled_draw_pixel(x + (2u * col) + 1u, y + (2u * row) + 0u, 1u);
        oled_draw_pixel(x + (2u * col) + 0u, y + (2u * row) + 1u, 1u);
        oled_draw_pixel(x + (2u * col) + 1u, y + (2u * row) + 1u, 1u);
      }
    }
  }
}

/**
 * @brief Draw string in 2x scaled size.
 *        Character pitch = 12 px (10 px glyph + 2 px spacing)
 */
void oled_draw_string_big(uint8_t x, uint8_t y, const char *str)
{
  while (*str != '\0')
  {
    oled_draw_char_big(x, y, *str);
    x += 12u;
    str++;
  }
}

/**
 * @brief Updates the OLED content.
 *
 *        If the output is disabled, "OFF" is shown.
 *        If the output is enabled, the current frequency and amplitude
 *        percentage are displayed.
 */
void oled_show_frequency_or_off(void)
{
  char line1[16];
  char line2[16];
  uint8_t x1;
  uint8_t x2;
  uint32_t freq_int;
  uint32_t ampl_pct;
  uint8_t len1;
  uint8_t len2;

  oled_clear_buffer();

  if (output_enabled == 0u)
  {
    // "OFF" -> 3 chars * 12 px = 36 px, centered on 128 px
    x1 = (uint8_t)((OLED_WIDTH - 36u) / 2u);
    oled_draw_string_big(x1, 24u, "OFF");
  }
  else
  {
    freq_int = (int32_t)(frequency_hz);
    ampl_pct = get_amplitude_percent_value();

    snprintf(line1, sizeof(line1), "FREQ:%luHz", (unsigned long)freq_int);
    snprintf(line2, sizeof(line2), "AMPL:%lu%% ", (unsigned long)ampl_pct);

    len1 = (uint8_t)strlen(line1);
    len2 = (uint8_t)strlen(line2);

    x1 = (uint8_t)((OLED_WIDTH - (len1 * 12u)) / 2u);
    x2 = (uint8_t)((OLED_WIDTH - (len2 * 12u)) / 2u);

    oled_draw_string_big(x1, 6u,  line1);
    oled_draw_string_big(x2, 34u, line2);
  }

  oled_update();
}

/**
 * @brief Refresh OLED periodically.
 */
void oled_task(void)
{
  if ((ms_ticks - last_oled_update_time) >= OLED_UPDATE_PERIOD_MS)
  {
    last_oled_update_time = ms_ticks;
    oled_show_frequency_or_off();
  }
}

// ============================================================================
// Main
// ============================================================================

/**
 * @brief Main entry point.
 */
int main(void)
{
  SystemClock_Config_80MHz();
  SysTick_Config(80000000u / 1000u);

  button_setup();
  encoder_setup();
  serial_setup();
  dac_setup();

  i2c1_setup();
  oled_init();

  generate_lut();
  update_tuning_word(frequency_hz);
  timer6_setup();

  oled_show_frequency_or_off();

  while (1)
  {
    button();
    encoder_update();
    serial_output();
    oled_task();
  }
}