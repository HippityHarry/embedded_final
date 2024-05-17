/*
Question 1: System requirements:
  1. Clock frequency of the CPU is at 50 MHz (checked)
  2. UART channel 0 specification:
    a. UART clock source is 22.1184 MHz (checked)
    b. UART0 channel is used to transmit data and receive data. (checked)
    c. Data packet: 1 start bit + 8 data bit + no parity bit + 1 stop bit
(checked)
    d. Baud rate: 115200 bps (checked)
 * */

#include "NUC100Series.h"
#include <stdio.h>

// Function declaration
void system_config();
void UART02_IRQHandler();

int main(void) {
  uint32_t data;

  system_config();
  while (1) {
    // wait until RX FIFO is not full
    while (UART0->FSR & (1 << 15))
      ;
    data = UART0->DATA;
    while (UART0->FSR & (1 << 23))
      ; // wait until TX FIFO is not full
    UART0->DATA = 'H';
    CLK_SysTickDelay(200);
  }
}

void system_config(void) {
  // --- UNLOCK PROTECTED REGISTERS ---
  SYS_UnlockReg();

  // --- POWER ---
  // Power on HXT
  CLK->PWRCON |= (1 << 0);
  while (!(CLK->CLKSTATUS & 1 << 0))
    ;

  // --- PLL ---
  // PLL input to HXT 12MHz
  CLK->PLLCON &= ~(1ul << 19);
  // Enable PLL clock output
  CLK->PLLCON &= ~(1ul << 18);
  // PLL in normal mode
  CLK->PLLCON &= ~(1ul << 16);
  // Clear feedback
  CLK->PLLCON &= ~0x01FFul;
  // Set output to 50 MHz
  CLK->PLLCON |= 48;
  while (!(CLK->CLKSTATUS & (1ul << 2)))
    ;

  // --- CPU CLOCK ---
  // CPU clock source to PLL
  CLK->CLKSEL0 &= (~(0b111 << 0));
  CLK->CLKSEL0 |= (0b010);
  // Normal mode
  CLK->PWRCON &= ~(1 << 7);
  // Frequency divider
  CLK->CLKDIV &= (~(0xF << 0));

  // --- UART0 ---
  // Clock source to 22.1184 Mhz
  CLK->CLKSEL1 |= (0b11 << 24);
  // Clock divier to 1
  CLK->CLKDIV &= ~(0xF << 8);
  // Enable clock
  CLK->APBCLK |= (1 << 16);
  // PB.0 input
  PB->PMD &= ~(0b11 << 0);
  // PB.1 output
  PB->PMD &= ~(0b11 << 2);
  PB->PMD |= (0b01 << 2);
  // UART0 RXD to PB.0
  SYS->GPB_MFP |= (1 << 0);
  // UART0 TXD to PB.1
  SYS->GPB_MFP |= (1 << 1);
  // 8 data bit
  UART0->LCR |= (0b11 << 0);
  // No parity bit
  UART0->LCR &= ~(1 << 3);
  // 1 stop bit
  UART0->LCR &= ~(1 << 2);
  // FIFO trigger level to 1 byte
  UART0->FCR &= ~(0xF << 16);
  // FIFO reset RX field
  UART0->FCR |= (1 << 1);
  // FIFO reset TX field
  UART0->FCR |= (1 << 2);

  // UART_CLK/[16*(A+2)] = 22.1184 MHz/[16*(10+2)] = 115200 bps
  UART0->BAUD &= ~(0b11 << 28);
  UART0->BAUD &= ~(0xFFFF << 0);
  UART0->BAUD |= 10;

  // UART interrupt
  UART0->IER |= 1 << 0;
  NVIC->ISER[0] |= 1 << 12;
  NVIC->IP[3] &= ~(0b11 << 6);

  SYS_LockReg();
}

void UART02_IRQHandler(void) {
  volatile uint8_t data;

  if (UART0->ISR & 1 << 0) {
    data = UART0->RBR;
    while (UART0->FSR & (1 << 23))
      ;
    UART0->DATA = data;
  }
}
