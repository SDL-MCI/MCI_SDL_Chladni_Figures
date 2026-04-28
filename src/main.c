/**
 * @file  main.c
 * @brief Signal generator with rotary encoder control, button-based ON/OFF handling,
 *        DAC sine output via DDS, serial frequency output and OLED visualization.
 *
 * This program runs on the STM32L476RG and generates a sine signal at the DAC output
 * (PA4). The output frequency can be adjusted with a rotary encoder using TIM3 in
 * encoder mode. A push button on PB3 toggles the signal output with controlled
 * startup and shutdown ramps. The currently selected frequency and amplitude are
 * shown on an SSD1306 OLED display via I2C1 and are also sent periodically over
 * UART in Teleplot-compatible format.
 *
 * Functional overview:
 * - Rotary encoder on PC6 / PC7 for frequency adjustment
 * - Push button on PB3 for controlled output enable / disable
 * - DAC output on PA4
 * - DDS-based sine generation using LUT and TIM6 interrupt
 * - Frequency-dependent amplitude selection using an empirical lookup table
 * - UART debug / Teleplot output
 * - SSD1306 OLED display via I2C1 for frequency / amplitude / OFF status
 *
 * Implementation notes:
 * - The amplitude table uses a conservative set of 7 well-distributed support points
 *   to provide stable behavior over the selected frequency range.
 * - I2C communication uses timeout handling to prevent long blocking periods.
 * - OLED updates are performed periodically with a moderate refresh rate to reduce
 *   I2C bus activity during signal generation.
 * - The button must be held for 2 seconds before shutdown / startup is executed.
 * - Wider amplitude lookup windows are used to reduce interpolation sensitivity.
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

#define DEBOUNCE_MS             300u
#define BUTTON_HOLD_MS          2000u
#define USART_PERIOD_MS         50u
#define OLED_UPDATE_PERIOD_MS   1000u  // Moderate refresh rate to reduce I2C bus activity

// ============================================================================
// Frequency Limits
// ============================================================================

#define FREQ_MIN 30.0f
#define FREQ_MAX 400.0f

// ============================================================================
// Ramp Settings
// ============================================================================

#define RAMP_STEPS         50u
#define RAMP_DELAY_TICKS   10e5  // Tunable (~2s shutdown)

// ============================================================================
// Encoder Settings
// ============================================================================

#define FREQ_PER_REV         10.0f  // Chosen user sensitivity 
#define ENC_PULSES_PER_REV   24.0f  // Encoder: 24 pulses per 360° rotation (datasheet)
#define ENC_COUNTS_PER_REV   (ENC_PULSES_PER_REV * 4.0f)  // Encoder-Mode 3 means we count every flank (x4)
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
#define DAC_AMPL_CODE     1241u   // ~1.0 V peak (100% amplitude)

// ============================================================================
// Frequency-dependent Amplitude Settings
// ============================================================================
//
// The table contains relative STM DAC amplitudes.
// 1.0 corresponds to DAC_AMPL_CODE, i.e. approximately 1 V peak at the STM DAC.
// 0.5 corresponds to approximately 0.5 V peak, etc.
//
// The amplitude table is intentionally conservative:
// - 7 well-distributed support points
// - Smooth characteristic without sharp amplitude jumps
// - Stable behavior over the full frequency range
//
// The values were derived from measured amplifier LEVEL settings.
// The external amplifier LEVEL can later be set to a fixed suitable value.

#define AMP_WINDOW_WIDE 5.0f  // Wide tolerance (prevents interpolation precision issues)
#define MIN_DAC_AMPL_CODE 50u // Minimum amplitude guard

typedef struct
{
  float freq_hz;
  float ampl_rel;
} AmpPoint;

// Frequency-dependent amplitude table: 7 smooth support points
static const AmpPoint amp_table[] =
{
  { 30.0f,  0.20f },   // Anchor (minimum)
  { 100.0f, 0.40f },   // Average of 56, 60 Hz region
  { 150.0f, 0.40f },   // Average of 130, 142, 164 Hz region
  { 200.0f, 0.50f },   // Average of 172, 214, 223 Hz region
  { 250.0f, 0.60f },   // Average of 247 Hz region
  { 350.0f, 0.40f },   // 348 Hz
  { 400.0f, 0.25f }    // Anchor (tail)
};

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
// I2C Timeout Settings
// ============================================================================

// Timeout limits long I2C stalls and keeps the TIM6 DAC interrupt responsive
#define I2C_TIMEOUT_COUNT 5000u  // Timeout loop count

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
volatile uint8_t button_hold_active = 0u;  // Active while the button hold check is running
volatile uint8_t button_action_done = 0u;  // Prevents repeated action during one button hold
volatile uint8_t led_state          = 0u;  // Stores current LED state for debugging / future use

volatile uint32_t ms_ticks                 = 0u; // System time base in milliseconds
volatile uint32_t last_button_time         = 0u; // Timestamp of last accepted button action
volatile uint32_t button_hold_start_time   = 0u; // Timestamp when button hold started

// ---------- Encoder / Frequency ----------
volatile int32_t encoder_last_cnt = 0;
volatile int32_t encoder_delta    = 0;
volatile float   frequency_hz     = FREQ_MIN;

// ---------- Output State ----------
volatile uint8_t output_enabled = 1u;    // Used for ON/OFF function (DAC)

// ---------- Serial ----------
volatile uint32_t last_plot_time = 0u;

// ---------- DDS / DAC ----------
volatile uint32_t phase_acc       = 0u;
volatile uint32_t tuning_word     = 0u;
volatile uint16_t dac_output_code = DAC_OFFSET_CODE;  // Current DAC output (12-bit-code) for serial output

volatile uint16_t dac_ampl_current_code = DAC_AMPL_CODE;
volatile uint16_t dac_ampl_target_code  = DAC_AMPL_CODE;

int16_t sine_lut[LUT_SIZE];

// ---------- OLED ----------
uint8_t oled_buffer[OLED_BUFFER_SIZE];
volatile uint32_t last_oled_update_time = 0u;
volatile uint8_t oled_update_pending = 0u;  // Reserved flag for possible OLED update scheduling

// ============================================================================
// Function Prototypes
// ============================================================================

// ---------- Utility ----------
uint32_t get_amplitude_percent_value(void);
uint32_t get_frequency_display_value(void);
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
// Main
// ============================================================================

/**
 * @brief Main entry point.
 */
int main(void)
{
  SystemClock_Config_80MHz();
  SysTick_Config(SYSCLK_FREQ / 1000u);

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
 * @brief Returns the selected frequency rounded to the nearest integer Hz.
 * @return Frequency in whole Hz within the valid range
 */
uint32_t get_frequency_display_value(void)
{
  if (frequency_hz <= FREQ_MIN)
  {
    return (uint32_t)FREQ_MIN;
  }

  if (frequency_hz >= FREQ_MAX)
  {
    return (uint32_t)FREQ_MAX;
  }

  return (uint32_t)(frequency_hz + 0.5f);
}

/**
 * @brief Returns the DAC amplitude code from the selected frequency.
 *
 *        The amplitude is calculated from an empirical frequency table.
 *        Wide lookup windows around the support points reduce unnecessary
 *        interpolation and improve stable behavior.
 *
 *        Implementation details:
 *        - Wide tolerance windows around support points
 *        - Conservative table with smooth amplitude transitions
 *        - Double-precision intermediate calculations
 *        - Explicit bounds checking
 *        - Minimum amplitude guard to prevent signal collapse
 *
 * @param freq Frequency in Hz
 * @return DAC amplitude code (never 0 unless explicitly disabled)
 */
uint16_t amplitude_code_from_frequency(float freq)
{
  const uint32_t n = sizeof(amp_table) / sizeof(amp_table[0]);

  // ---------- Input validation ----------
  if (freq < FREQ_MIN)
  {
    freq = FREQ_MIN;
  }
  if (freq > FREQ_MAX)
  {
    freq = FREQ_MAX;
  }

  float ampl_target = amp_table[0].ampl_rel;  // Default to first point

  // ---------- Check constant windows first ----------
  for (uint32_t i = 0; i < n; i++)
  {
    float f_center = amp_table[i].freq_hz;
    float f_min = f_center - AMP_WINDOW_WIDE;
    float f_max = f_center + AMP_WINDOW_WIDE;

    if ((freq >= f_min) && (freq <= f_max))
    {
      // We're within a constant-amplitude window -> use this value directly
      ampl_target = amp_table[i].ampl_rel;
      goto amplitude_clamp;  // Jump to clamping section
    }
  }

  // ---------- Interpolate between nearest points if outside all windows ----------
  // Only do this if absolutely necessary (i.e., freq not in any window)
  
  // Find the two bracketing points
  for (uint32_t i = 0; i < (n - 1u); i++)
  {
    float f0 = amp_table[i].freq_hz;
    float f1 = amp_table[i + 1u].freq_hz;

    // Check if freq is between the two points (outside their windows)
    float f0_max = f0 + AMP_WINDOW_WIDE;
    float f1_min = f1 - AMP_WINDOW_WIDE;

    if ((freq > f0_max) && (freq < f1_min))
    {
      // Linear interpolation between f0 and f1
      float a0 = amp_table[i].ampl_rel;
      float a1 = amp_table[i + 1u].ampl_rel;

      // Use double precision for intermediate calculation to reduce rounding error
      double freq_d = (double)freq;
      double f0_d = (double)f0;
      double f1_d = (double)f1;
      double a0_d = (double)a0;
      double a1_d = (double)a1;

      double k_d = (freq_d - f0_d) / (f1_d - f0_d);
      double ampl_d = a0_d + k_d * (a1_d - a0_d);

      ampl_target = (float)ampl_d;
      goto amplitude_clamp;
    }
  }

  // ---------- Fallback: If freq is above all points, use last ----------
  ampl_target = amp_table[n - 1u].ampl_rel;

amplitude_clamp:
  // ---------- Clamp relative amplitude ----------
  if (ampl_target < 0.0f)
  {
    ampl_target = 0.0f;
  }
  if (ampl_target > 1.0f)
  {
    ampl_target = 1.0f;
  }

  // ---------- Convert relative amplitude to DAC code ----------
  uint16_t code = (uint16_t)((float)DAC_AMPL_CODE * ampl_target + 0.5f);
  
  
  // ---------- Enforce minimum amplitude to prevent collapse ----------
  // Even at very low amplitude, maintain at least 50 DAC codes.
  // This prevents the signal from disappearing due to rounding/precision loss.

  if (code < MIN_DAC_AMPL_CODE)
  {
    code = MIN_DAC_AMPL_CODE;
  }

  return code;
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
 *
 *        The interrupt only starts the hold detection.
 *        The actual shutdown / startup action is executed in the main loop
 *        only if the button remains pressed for BUTTON_HOLD_MS.
 */
void EXTI3_IRQHandler(void)
{
  if (EXTI->PR1 & (1u << 3))  // Check if EXTI line 3 caused the interrupt
  {
    EXTI->PR1 = (1u << 3);    // Clear EXTI3 pending flag by writing 1

    // Start hold detection only if no hold is currently active
    // and debounce time has elapsed since the last accepted action.
    if ((button_hold_active == 0u) &&
        ((ms_ticks - last_button_time) >= DEBOUNCE_MS))
    {
      // Button is active LOW.
      // Only start hold detection if PB3 is currently LOW.
      if ((GPIOB->IDR & (1u << BUTTON_PIN)) == 0u)
      {
        button_hold_active = 1u;
        button_action_done = 0u;
        button_hold_start_time = ms_ticks;
      }
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
  GPIOB->MODER &= ~(0b11 << (2 * BUTTON_PIN));  // Input mode

  GPIOB->PUPDR &= ~(0b11 << (2 * BUTTON_PIN));  // Clear pull-up/pull-down bits (external pull-up)
  GPIOB->PUPDR |=  (0b01 << (2 * BUTTON_PIN));  // Internal pull-up enabled

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
 * @brief Processes button hold events in the main loop.
 *
 *        The button must be held LOW continuously for BUTTON_HOLD_MS
 *        before shutdown / startup is executed.
 *
 *        If the button is released before the hold time is reached,
 *        the event is cancelled. This rejects short EMI spikes.
 */
void button(void)
{
  // No active hold detection running
  if (button_hold_active == 0u)
  {
    return;
  }

  // Button is active LOW.
  // If PB3 is HIGH again, the button was released before the hold time.
  if ((GPIOB->IDR & (1u << BUTTON_PIN)) != 0u)
  {
    button_hold_active = 0u;
    button_action_done = 0u;
    return;
  }

  // Button is still pressed.
  // Execute action only once after BUTTON_HOLD_MS.
  if ((button_action_done == 0u) &&
      ((ms_ticks - button_hold_start_time) >= BUTTON_HOLD_MS))
  {
    button_action_done = 1u;
    button_hold_active = 0u;
    last_button_time = ms_ticks;

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
  TIM3->ARR = MAX_RELOAD;

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
 *        The internal frequency is kept as a float for smooth encoder behavior.
 *        Only the applied output frequency is rounded to integer Hz.
 *
 *        When the output is enabled, the rounded frequency and corresponding
 *        amplitude are applied immediately. When the output is disabled,
 *        only the selected frequency value is updated internally.
 */
void encoder_update(void)
{
  int32_t cnt_now = (int32_t)TIM3->CNT;
  encoder_delta   = (int16_t)(cnt_now - encoder_last_cnt);

  if (encoder_delta != 0)
  {
    float freq_apply;

    encoder_last_cnt = cnt_now;

    // Keep internal frequency continuous
    frequency_hz += ((float)encoder_delta * FREQ_STEP_PER_COUNT);

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
      freq_apply = (float)get_frequency_display_value();

      update_tuning_word(freq_apply);
      dac_ampl_target_code  = amplitude_code_from_frequency(freq_apply);
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

    uint32_t freq_int = get_frequency_display_value();
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

  dac_ampl_current_code = amplitude_code_from_frequency((float)get_frequency_display_value());
  dac_ampl_target_code  = dac_ampl_current_code;

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
  TIM6->CR1 = 0;  // Clear control register
  TIM6->PSC = 0;  // Prescaler = 0 (timer runs directly at 80 MHz)

  // ---------- Set auto-reload value ----------
  // 80 MHz / (3999 + 1) = 20 kHz
  TIM6->ARR = TIM6_ARR_VALUE;

  TIM6->EGR  = TIM_EGR_UG;    // Update generation
  TIM6->SR   = 0;             // Clear pending status flags
  TIM6->DIER |= TIM_DIER_UIE; // Enable update interrupt
  TIM6->CR1  |= TIM_CR1_CEN;  // Counter enabled (start timer)

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
  dac_ampl_target_code  = amplitude_code_from_frequency((float)get_frequency_display_value());

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
  update_tuning_word((float)get_frequency_display_value());

  // ---------- Start from zero amplitude ----------
  dac_ampl_current_code = 0u;
  dac_ampl_target_code  = amplitude_code_from_frequency((float)get_frequency_display_value());

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
 *
 *        The timeout limits I2C blocking time so that the TIM6 DAC interrupt
 *        remains responsive. If I2C communication fails, the function returns
 *        with an error code and the main loop can continue running.
 *
 * @return 1 on success, 0 on error (I2C bus busy or timeout)
 */
uint8_t i2c1_write(uint8_t addr7, const uint8_t *data, uint16_t len)
{
  uint32_t timeout;

  if ((data == 0) || (len == 0) || (len > 255u))
  {
    return 0u;
  }

  // Limit waiting time to prevent long I2C stalls
  timeout = I2C_TIMEOUT_COUNT;
  while ((I2C1->ISR & I2C_ISR_BUSY) && timeout--)
  {
    // Small delay per iteration to simulate timing
    for (volatile int delay_cnt = 0; delay_cnt < 10; delay_cnt++);
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
    timeout = I2C_TIMEOUT_COUNT;

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
      
      // Small delay per iteration
      for (volatile int delay_cnt = 0; delay_cnt < 10; delay_cnt++);
    }

    I2C1->TXDR = data[i];
  }

  timeout = I2C_TIMEOUT_COUNT;
  while (!(I2C1->ISR & I2C_ISR_STOPF))
  {
    if (timeout-- == 0u)
    {
      return 0u;
    }
    
    // Small delay per iteration
    for (volatile int delay_cnt = 0; delay_cnt < 10; delay_cnt++);
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

  return font5x7[0].data; // Fallback = space
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
    freq_int = get_frequency_display_value();
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
 *
 *        The OLED is updated periodically with a moderate refresh rate.
 *        This reduces I2C bus activity and keeps the DAC interrupt responsive.
 */
void oled_task(void)
{
  if ((ms_ticks - last_oled_update_time) >= OLED_UPDATE_PERIOD_MS)
  {
    last_oled_update_time = ms_ticks;
    oled_show_frequency_or_off();
  }
}