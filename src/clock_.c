#include "clock_.h"

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
