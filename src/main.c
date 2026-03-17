#include "main.h"
#include "clock_.h"
#include <stdio.h>
#include <stdint.h>

// ---------- Encoder ----------
const uint8_t ChA_input_pin = 6;  // PC6 = TIM3_CH1 (PIN 4)
const uint8_t ChB_input_pin = 7;  // PC7 = TIM3_CH2 (PIN 19)

// ---------- Button and LED ----------
const uint8_t button_pin = 3;     // PB3 (PIN 31)
const uint8_t led_pin = 5;        // PA5 (PIN 11)

// ---------- Output ----------
const uint8_t voltage_out = 4;    // PA4 / DAC output (PIN 32)

// ---------- Button / Timebase ----------
volatile uint8_t button_pressed = 0;  // Set by EXTI interrupt, handled in main loop
volatile uint8_t led_state = 0;       // Stores current LED state

volatile uint32_t ms_ticks = 0;         // System time base in milliseconds
volatile uint32_t last_button_time = 0; // Timestamp of last accepted button press

// ---------- Encoder / Frequency ----------
volatile int32_t encoder_last_cnt = 0;
volatile int32_t encoder_delta = 0;
volatile float frequency_hz = 30.0f;

// ---------- Serial (Teleplot) ----------
volatile uint32_t last_plot_time = 0;

// ---------- Limits ----------
#define DEBOUNCE_MS         50u
#define TELEPLOT_PERIOD_MS  50u

#define FREQ_MIN 30.0f
#define FREQ_MAX 300.0f

// ---------- Encoder Settings ----------
#define FREQ_PER_REV         5.0f   // Chosen user sensitivity 
#define ENC_PULSES_PER_REV   24.0f  // Encoder: 24 pulses per 360° rotation (datasheet)
#define ENC_COUNTS_PER_REV   (ENC_PULSES_PER_REV * 4.0f)  // Encoder-Mode 3 means, we count every flank (x4)
#define FREQ_STEP_PER_COUNT  (FREQ_PER_REV / ENC_COUNTS_PER_REV)  // 5 Hz per rotation 

// ---------- Debug Macro ----------
#define DEBUG
#ifdef DEBUG
#define LOG(msg...) printf(msg);
#else
#define LOG(msg...);
#endif


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
      button_pressed = 1;
      last_button_time = ms_ticks;
    }
  }
}

/**
 * @brief Configures button input, LED output and EXTI interrupt.
 *        - PB3: button input with internal pull-up
 *        - PA5: onboard LED output
 *        - EXTI3: interrupt on falling edge for PB3
 */
void button_setup(void)
{
  // ---------- Enable peripheral clocks ----------
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;   // Enable GPIOA clock (LED)
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;   // Enable GPIOB clock (button)
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;  // Enable SYSCFG clock (needed for EXTI routing)

  // ---------- PB3 = button input with pull-up ----------
  GPIOB->MODER &= ~(0b11 << (2 * button_pin));  // Clear MODER bits -> input mode (00)

  GPIOB->PUPDR &= ~(0b11 << (2 * button_pin));  // Clear pull-up/pull-down bits
  GPIOB->PUPDR |=  (0b01 << (2 * button_pin));  // Set pull-up (01), needed for active LOW button

  // ---------- PA5 = LED output ----------
  GPIOA->MODER  &= ~(0b11 << (2 * led_pin));    // Clear MODER bits first
  GPIOA->MODER  |=  (0b01 << (2 * led_pin));    // Set output mode (01)

  GPIOA->OTYPER &= ~(1u << led_pin);            // Set output type to push-pull (0)

  GPIOA->BSRR = (1u << (led_pin + 16));         // Reset PA5 -> LED initially OFF

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
 *        Toggles the LED whenever a valid button press was detected.
 */
void button(void)
{
  if (button_pressed)
  {
    button_pressed = 0;     // Clear software event flag
    led_state ^= 1u;        // Toggle stored LED state

    if (led_state)
    {
      GPIOA->BSRR = (1u << led_pin);            // Set PA5 -> LED ON
    }
    else
    {
      GPIOA->BSRR = (1u << (led_pin + 16));     // Reset PA5 -> LED OFF
    }
  }
}

/**
 * @brief Configure TIM3 in encoder mode on PC6 / PC7.
 */
void encoder_setup(void)
{
  // ---------- Enable peripheral clocks ----------
  RCC->AHB2ENR  |= RCC_AHB2ENR_GPIOCEN; // IO port C clock enable (encoder pins)
  RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN; // TIM3 timer clock enable (encoder interface)

  // ---------- PC6, PC7 - Alternate Function ----------
  GPIOC->MODER &= ~(0b11 << (2 * ChA_input_pin));
  GPIOC->MODER &= ~(0b11 << (2 * ChB_input_pin));
  GPIOC->MODER |=  (0b10 << (2 * ChA_input_pin)); // Alternate function mode
  GPIOC->MODER |=  (0b10 << (2 * ChB_input_pin)); // Alternate function mode

  // ---------- Pull-Up Configuration ----------
  GPIOC->PUPDR &= ~(0b11 << (2 * ChA_input_pin));
  GPIOC->PUPDR &= ~(0b11 << (2 * ChB_input_pin));
  GPIOC->PUPDR |=  (0b01 << (2 * ChA_input_pin));
  GPIOC->PUPDR |=  (0b01 << (2 * ChB_input_pin));

  // ---------- High Speed Configuration ----------
  GPIOC->OSPEEDR &= ~(0b11 << (2 * ChA_input_pin));
  GPIOC->OSPEEDR &= ~(0b11 << (2 * ChB_input_pin));
  GPIOC->OSPEEDR |=  (0b10 << (2 * ChA_input_pin)); // High speed
  GPIOC->OSPEEDR |=  (0b10 << (2 * ChB_input_pin)); // High speed

  // ---------- AF2 (Alternate Function) for TIM3 on PC6 / PC7 ----------
  GPIOC->AFR[0] &= ~(0b1111 << (4 * ChA_input_pin));
  GPIOC->AFR[0] &= ~(0b1111 << (4 * ChB_input_pin));
  GPIOC->AFR[0] |=  (0b0010 << (4 * ChA_input_pin));  // AF2 ... TIM3_CH1
  GPIOC->AFR[0] |=  (0b0010 << (4 * ChB_input_pin));  // AF2 ... TIM3_CH2

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
  TIM3->ARR = 0b1111111111111111;

  // Start in the middle for clean signed delta handling
  TIM3->CNT = 0b1000000000000000;
  encoder_last_cnt = 0b1000000000000000;

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
 * @brief Read encoder movement and update frequency.
 */
void encoder_update(void)
{
  int32_t cnt_now = (int32_t)TIM3->CNT;
  encoder_delta   = (int16_t)(cnt_now - encoder_last_cnt);

  if (encoder_delta != 0)
  {
    encoder_last_cnt = cnt_now;

    frequency_hz += ((float)encoder_delta * FREQ_STEP_PER_COUNT);

    if (frequency_hz < FREQ_MIN)
    {
      frequency_hz = FREQ_MIN;
    }

    if (frequency_hz > FREQ_MAX)
    {
      frequency_hz = FREQ_MAX;
    }
  }
}

/**
 * @brief Setup serial debug output.
 *        Uses the existing debug UART helper from your lab environment.
 */
void serial_setup(void)
{
  Init_Debug_UART();
}

/**
 * @brief Send Teleplot-compatible values periodically.
 */
void teleplot_output(void)
{
  if ((ms_ticks - last_plot_time) >= TELEPLOT_PERIOD_MS)
  {
    last_plot_time = ms_ticks;

    LOG(">Freq:%d\r\n", (int)frequency_hz);
    // Optional additional traces:
    // LOG(">Delta:%d\n", (int)encoder_delta);
    // LOG(">CNT:%d\n",   (int)TIM3->CNT);
  }
}

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

  while (1)
  {
    button();
    encoder_update();
    teleplot_output();
  }
}