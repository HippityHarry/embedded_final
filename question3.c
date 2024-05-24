#include "LCD.h"
#include "MCU_init.h"
#include "NUC100Series.h"
#include "SYS_init.h"
#include <stdio.h>

void system_config(void);
void lcd_config(void);

int main(void) {
  system_config();

  while (1) {
  }
}

void system_config(void) {
  // --- UNLOCK PROTECTED REGISTERS ---
  SYS_UnlockReg();

  // --- POWER ---
  // Power on HXT (32.768)
  CLK->PWRCON |= (1 << 0);
  while (!(CLK->CLKSTATUS & 1 << 0))
    ;
  // Power on 10kHz
  CLK->PWRCON |= (1 << 3);
  while (!(CLK->CLKSTATUS & (1 << 3)))
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

  // --- Debounce ---
  // Enable
  PB->DBEN |= (1 << 15);
  // 10kHz clock source
  GPIO->DBNCECON |= (1 << 4);
  // Sampling cycle selection
  GPIO->DBNCECON |= 8;

  // --- GPIO ---
  // PB.15 interrupt
  PB->PMD &= (~(0b11 << 30));
  PB->IMD &= (~(1 << 15));
  // Falling edge trigger
  PB->IEN |= (1 << 15);
  NVIC->ISER[0] |= 1 << 3;
  NVIC->IP[0] &= (~(0b11 << 30));

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

  // --- SPI3 ---
  // Enable clock
  CLK->APBCLK |= 1 << 15;
  // Output pin mode for buzzer
  PB->PMD &= (~(0x03 << 22));
  PB->PMD |= (0b01 << 22);

  // --- 7 segments ---
  // Set mode for PC4 to PC7
  PC->PMD &= (~(0xFF << 8));    // clear PMD[15:8]
  PC->PMD |= (0b01010101 << 8); // Set output push-pull for PC4 to PC7

  // Set mode for PE0 to PE7
  PE->PMD &= (~(0xFFFF << 0));        // clear PMD[15:0]
  PE->PMD |= 0b0101010101010101 << 0; // Set output push-pull for PE0 to PE7

  SYS_LockReg(); // Lock protected registers
}

void lcd_config(void) {
  // --- LCD ---
  // System reset
  lcdWriteCommand(0xE2);
  // 100 fps framerate
  lcdWriteCommand(0xA1);
  // Bias ratio to 9
  lcdWriteCommand(0xEB);
  // V_Bias potentiometer (A0 -> 160)
  lcdWriteCommand(0x81);
  lcdWriteCommand(0xA0);
  // Mapping to X=0, Y=0
  lcdWriteCommand(0xC0);
  // Enable display
  lcdWriteCommand(0xAF);
}
