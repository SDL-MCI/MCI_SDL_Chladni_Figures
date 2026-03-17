#include "clock_.h"
#include <stdio.h>

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSI16)
  *            SYSCLK(Hz)                     = 80000000
  *            HCLK(Hz)                       = 80000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            PLL Source                     = HSI16 (16 MHz)
  *            PLLM                           = 1
  *            PLLN                           = 10
  *            PLLR                           = 2
  *            VCO Frequency(Hz)              = 160000000
  *            Flash Latency(WS)              = 4
  */
void SystemClock_Config_80MHz(void)
{
  // 1.) PWR (Power control) clock enable (APB1ENR1, Bit 28 = HIGH)
  // see Chapter "6.4.19" (Page 253-256)
  RCC->APB1ENR1 |= (1u << 28);

  // 2.) PWR: Voltage scaling range selection: Range 1 (for 80MHz)
  // see Chapter "5.4.1" (Page 184)
  PWR->CR1 = (PWR->CR1 & ~(0b11 << 9)) | (0b01 << 9);

  // 3.) Flash latency (typically 4 WS (wait states) at 80MHz)
  // see Chapter "3.7.1" (Page 124-125)
  FLASH->ACR &= ~(0b111 << 0);
  FLASH->ACR |= (0b100 << 0);

  // 4.) HSI16 clock enable (HSI16)
  // see Chapter "6.4.1" (Page 223-226)
  RCC->CR |= (1u << 8);
  // wait for clock to become stable before continuing (HSIRDY == 1)
  while (!(RCC->CR & (1u << 10)));

  // 5.) PLL off
  // see Chapter "6.4.1" (Page 223-226)
  RCC->CR &= ~(1u << 24);
  // wait until PLL is really off before continuing (PLLRDY = 0)
  while ((RCC->CR & (1u << 25)));

  // 6.) PLL configuration
  // see Chapter "6.4.4" (Page 229-232)

  // PLLSRC: Main PLL entry clock source
  RCC->PLLCFGR &= ~(0b11 << 0);
  RCC->PLLCFGR |= (0b10 << 0);  // HSI16 clock selected

  // PLLM: Division factor for the main PLL input clock
  RCC->PLLCFGR &= ~(0b111 << 4);
  RCC->PLLCFGR |= (0b000 << 4);

  // PLLN: Main PLL multiplication factor for VCO
  RCC->PLLCFGR &= ~(0b1111111 << 8);
  RCC->PLLCFGR |= (0b0001010 << 8);

  // PLLR: Main PLL division factor for PLLCLK (system clock)
  RCC->PLLCFGR &= ~(0b11 << 25);
  RCC->PLLCFGR |= (0b00 << 25);

  // PLLREN: Main PLL PLLCLK output enable
  RCC->PLLCFGR |= (1u << 24);

  // PLLQEN (PLL48M1CLK) and PLLPEN (PLLSAI3CLK) off
  RCC->PLLCFGR &= ~(1u << 20);
  RCC->PLLCFGR &= ~(1u << 16);

  // 7.) PLL enable
  // see Chapter "6.4.1" (Page 223-226)
  RCC->CR |= (1u << 24);
  // wait for clock to become stable before continuing (PLLRDY == 1)
  while (!(RCC->CR & (1u << 25)));

  // 8) Prescaler: AHB/APB1/APB2 = /1 (not divided)
  // see Chapter "6.4.3" (Page 227-229)
  // HPRE  bits [7:4]  = 0000
  // PPRE1 bits [10:8] = 000
  // PPRE2 bits [13:11]= 000
  RCC->CFGR &= ~(0b1111 << 4); // AHB prescaler
  RCC->CFGR &= ~(0b111 << 8); // APB1 prescaler
  RCC->CFGR &= ~(0b111 << 11); // APB2 prescaler

  // SYSCLK switch to PLL
  // see Chapter "6.4.3" (Page 227-229)
  RCC->CFGR &= ~(0b11 << 0);
  RCC->CFGR |= (0b11 << 0); // PLL selected as system clock
  // wait for system clock switch status (SWS == 11)
  while (((RCC->CFGR >> 2) & 0b11) != 0b11);
}

/**
 * @brief  Initializes USART2 for debug output / Teleplot.
 *         USART2_TX = PA2, USART2_RX = PA3
 *         On STM32L476, PA2/PA3 use AF7 for USART2.
 */
void Init_Debug_UART(void)
{
  const uint8_t USART2_TX_PIN = 2;   // PA2
  const uint8_t USART2_RX_PIN = 3;   // PA3

  // ---------- Enable peripheral clocks ----------
  RCC->AHB2ENR  |= RCC_AHB2ENR_GPIOAEN;
  RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;

  // ---------- PA2, PA3 - Alternate Function ----------
  GPIOA->MODER &= ~(0b11 << (2 * USART2_TX_PIN));
  GPIOA->MODER &= ~(0b11 << (2 * USART2_RX_PIN));
  GPIOA->MODER |=  (0b10 << (2 * USART2_TX_PIN));
  GPIOA->MODER |=  (0b10 << (2 * USART2_RX_PIN));

  // ---------- Pull-Up Configuration ----------
  GPIOA->PUPDR &= ~(0b11 << (2 * USART2_TX_PIN));
  GPIOA->PUPDR &= ~(0b11 << (2 * USART2_RX_PIN));
  GPIOA->PUPDR |=  (0b01 << (2 * USART2_TX_PIN));
  GPIOA->PUPDR |=  (0b01 << (2 * USART2_RX_PIN));

  // ---------- High Speed Configuration ----------
  GPIOA->OSPEEDR &= ~(0b11 << (2 * USART2_TX_PIN));
  GPIOA->OSPEEDR &= ~(0b11 << (2 * USART2_RX_PIN));
  GPIOA->OSPEEDR |=  (0b10 << (2 * USART2_TX_PIN)); // High speed
  GPIOA->OSPEEDR |=  (0b10 << (2 * USART2_RX_PIN)); // High speed

  // ---------- AF7 for USART2 ----------
  GPIOA->AFR[0] &= ~(0b1111 << (4 * USART2_TX_PIN));
  GPIOA->AFR[0] &= ~(0b1111 << (4 * USART2_RX_PIN));
  GPIOA->AFR[0] |=  (0b0111 << (4 * USART2_TX_PIN)); // AF7 ... USART2_TX
  GPIOA->AFR[0] |=  (0b0111 << (4 * USART2_RX_PIN)); // AF7 ... USART2_RX

  // ---------- Disable USART before config ----------
  USART2->CR1 &= ~(1u << 0);

  // ---------- Reset basic config ----------
  USART2->CR1 = 0;
  USART2->CR2 = 0;
  USART2->CR3 = 0;

  // ---------- Baud rate ----------
  // oversampling by 16 (default)
  // BRR = fck / baud = 80 MHz / 115200 = 694.44 -> 694
  USART2->BRR = (SYSCLK_FREQ / BAUDRATE);

  // ---------- Enable TX and RX ----------
  USART2->CR1 |= (1u << 3);   // Transmitter enable
  USART2->CR1 |= (1u << 2);   // Receiver enable

  // ---------- Enable USART ----------
  USART2->CR1 |= (0b1u << 0);   // USART enable
}

/**
 * @brief Simple blocking delay.
 *        Crude software delay only.
 */
void delay(uint32_t time)
{
  for (uint32_t i = 0; i < time; i++)
  {
    asm("nop"); // No operation, used for delaying
  }
  return 0;
}

/**
 * @brief Redirect printf() to USART2.
 */
int _write(int handle, char *data, int size)
{
  int count = size;
  while (count--)
  {
    while (!(USART2->ISR & USART_ISR_TXE))
    {
    };
    USART2->TDR = *data++;
  }
  return size;
}