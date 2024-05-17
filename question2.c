/*
Question 2: System requirements:
  1. Clock for the CPU is at 50 MHz (checked)
  2. ADC channel 7 (12-bit A/D converter with reference voltage, Vref = 3.3 V )
is used to continuously sample analog voltage at GPIO port A pin 7 and convert
it into a corresponding digital value. The configuration for ADC:
    - ADC clock frequency is 1 MHz
    - Operating mode: continuous scan
  3. SPI2 of the NUC140 MCU will be configured as a master device to transmit
data to an off-chip slave device (in this Assignment, we don’t have to worry
about what device is this). The configuration for the SPI2:
    - SPI serial clock is 1 MHz
    - SPI serial clock is IDLE at HIGH state
    - Data bit is transmitted on the POSITIVE edge of serial clock
    - Data is transferred from LSB first
    - One BYTE of data to be transmitted/received in a transaction
    - SPI2 pins: GPIOD.0 (SPI2_SS), GPIOD.1 (SPI2_CLK), GPIOD.3 (SPI2_MOSI)
*/

#include "NUC100Series.h"
#include <stdio.h>

// Function declaration
void system_config();
void UART02_IRQHandler();

int main(void) { system_config(); }

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

  SYS_LockReg();
}
