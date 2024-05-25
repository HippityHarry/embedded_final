#include "LCD.h"
#include "MCU_init.h"
#include "NUC100Series.h"
#include "SYS_init.h"
#include <stdio.h>

void system_config(void);
void lcd_config(void);
void UART02_IRQHandler(void);

enum GameStatus { WAITING, LOADED };

volatile enum GameStatus status = WAITING;
volatile uint32_t map_x = 0;
volatile uint32_t map_y = 0;
volatile uint32_t map_data[8][8];

int main(void) {
  system_config();
  lcd_config();

  while (1) {
    if (status == WAITING) {
      clear_LCD();
      printS_5x7(21, 32, "RMIT Battleship~");
      printS_5x7(18, 48, "Waiting for map...");

      continue;
    }

    if (status == LOADED) {
      clear_LCD();
      for (int16_t x = 0; x < 8; x++) {
        for (int16_t y = 0; y < 8; y++) {
          printC_5x7(x * 6, y * 8, map_data[x][y]);
        }
      }

      continue;
    }
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
  SYS->GPD_MFP |= 1 << 11; // 1: PD11 is configured for alternative function
  SYS->GPD_MFP |= 1 << 9;  // 1: PD9 is configured for alternative function
  SYS->GPD_MFP |= 1 << 8;  // 1: PD8 is configured for alternative function

  SPI3->CNTRL &= ~(1 << 23); // 0: disable variable clock feature
  SPI3->CNTRL &= ~(1 << 22); // 0: disable two bits transfer mode
  SPI3->CNTRL &= ~(1 << 18); // 0: select Master mode
  SPI3->CNTRL &= ~(1 << 17); // 0: disable SPI interrupt
  SPI3->CNTRL |= 1 << 11;    // 1: SPI clock idle high
  SPI3->CNTRL &= ~(1 << 10); // 0: MSB is sent first
  SPI3->CNTRL &= ~(3 << 8); // 00: one transmit/receive word will be executed in
                            // one data transfer

  SPI3->CNTRL &= ~(31 << 3); // Transmit/Receive bit length
  SPI3->CNTRL |= 9 << 3;     // 9: 9 bits transmitted/received per data transfer

  SPI3->CNTRL |= (1 << 2); // 1: Transmit at negative edge of SPI CLK
  SPI3->DIVIDER =
      0; // SPI clock divider. SPI clock = HCLK / ((DIVIDER+1)*2). HCLK = 50 MHz

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

void UART02_IRQHandler(void) {
  if ((map_x == 0 && map_y == 0) && (UART0->ISR & 1 << 0)) {
    map_data[map_x][map_y] = UART0->RBR;
    while (UART0->FSR & (1 << 23))
      ;
    UART0->DATA = map_x;
    while (UART0->FSR & (1 << 23))
      ;
    UART0->DATA = map_x;
    while (UART0->FSR & (1 << 23))
      ;
    UART0->DATA = map_data[map_x][map_y];

    map_x++;
    if (map_x > 8) {
      map_x = 0;
      map_y++;
    }
    if (map_y == 8) {
      status = LOADED;
      // reset map loader
      map_x = 0;
      map_y = 0;
    }
  }
}

