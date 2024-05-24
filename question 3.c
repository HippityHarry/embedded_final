#include <stdio.h>
#include "NUC100Series.h"
#include "MCU_init.h"
#include "SYS_init.h"
#include "LCD.h"

void System_Config(void);
void UART0_Config(void);

int main(void){}

void System_Config(void) {
SYS_UnlockReg(); // Unlock protected registers

CLK->PWRCON |= (1 << 0); //Enable 12MHz clock
while(!(CLK->CLKSTATUS & (1 << 0)));

CLK->PWRCON |= (1 << 3); //Enable 10Khz clock (for button de-bouncing)
while(!(CLK->CLKSTATUS & (1 << 3)));
		
//PLL configuration starts
CLK->PLLCON &= ~(1<<19); //0: PLL input is HXT
CLK->PLLCON &= ~(1<<16); //PLL in normal mode
CLK->PLLCON &= (~(0x01FF << 0)); //clear
CLK->PLLCON |= 48; //to generate 50MHz clock signal
CLK->PLLCON &= ~(1<<18); //0: enable PLLOUT
while(!(CLK->CLKSTATUS & (1 << 2))); //wait for stabilization
//PLL configuration ends
		
//clock source selection
CLK->CLKSEL0 &= (~(0x07 << 0)); //clear
CLK->CLKSEL0 |= (Ob10 << 0); //select PLL as clock source
//clock frequency division
CLK->CLKDIV &= (~0x0F << 0); //cleared as not needed

//UART0 Clock selection and configuration
CLK->CLKSEL1 |= (0b11 << 24); // UART0 clock source is 22.1184 MHz
CLK->CLKDIV &= ~(0xF << 8); // clock divider is 1
CLK->APBCLK |= (1 << 16); // enable UART0 clock

//enable clock of SPI3
CLK->APBCLK |= 1 << 15;
	
//Output pin mode for buzzer
PB->PMD &= (~(0x03 << 22));//clear
PB->PMD |= (0b01 << 22);

//GPIO Interrupt configuration. GPIO-B15 is the interrupt source
PB->PMD &= (~(0b11 << 30)); //Input mode
PB->IMD &= (~(1 << 15)); //Edge trigger interrupt
PB->IEN |= (1 << 15); //Enable interrupt (falling edge trigger)
	
PB->DBEN |= (1 << 15); //Enable de-bounce function
GPIO->DBNCECON |= (1 << 4); //Clock source is 10 kHz clock
GPIO->DBNCECON |= 8; //Sampling cycle selection
	
//NVIC interrupt configuration for GPIO-B15 interrupt source
NVIC->ISER[0] |= 1 << 3;
NVIC->IP[0] &= (~(3 << 30));
	
// 7 segments
//Set mode for PC4 to PC7 
    PC->PMD &= (~(0xFF<< 8));		//clear PMD[15:8] 
    PC->PMD |= (0b01010101 << 8);    	//Set output push-pull for PC4 to PC7
	
	//Set mode for PE0 to PE7
	PE->PMD &= (~(0xFFFF<< 0));		//clear PMD[15:0] 
	PE->PMD |= 0b0101010101010101<<0;   //Set output push-pull for PE0 to PE7
		
SYS_LockReg();  // Lock protected registers
}

void UART0_Config(void) {
    // UART0 pin configuration. 
	//PB.1 pin is for UART0 TX (data transmission)
    PB->PMD &= ~(0b11 << 2); //clear
    PB->PMD |= (0b01 << 2); // PB.1 is output pin
    SYS->GPB_MFP |= (1 << 1); // GPB_MFP[1] = 1 -> PB.1 is UART0 TX pin
	
	// PB.0 pin is for UART0 RX (data reception)
	PB->PMD &= ~(0b11 << 0); // Set PB.0 as input pin
	PB->IMD &= ~(1<<0); //Edge trigger interrupt
	PB->IEN |= (1<<0); //Enable interrupt (falling edge trigger)
	SYS->GPB_MFP |= (1 << 0); // GPB_MFP[0] = 1 -> PB.0 is UART0 RX pin

    // UART0 operation configuration
    UART0->LCR |= (0b11 << 0); // 8 data bit
    UART0->LCR &= ~(1 << 2); // one stop bit    
    UART0->LCR &= ~(1 << 3); // no parity bit
    UART0->FCR |= (1 << 1); // clear RX FIFO
    UART0->FCR |= (1 << 2); // clear TX FIFO
    UART0->FCR &= ~(0xF << 16); // FIFO Trigger Level is 1 byte]
    
    //Baud rate config
    UART0->BAUD &= ~(0b11 << 28); //Clear to set mode 0
    UART0->BAUD &= ~(0xFFFF << 0);//clear
    UART0->BAUD |= 10;  //A = 10
     //--> Mode 0, Baud rate = UART_CLK/[16*(A+2)] = 22.1184 MHz/[16*(10+2)]= 115200 bps

		
	//set up UART interrupt and interrupt priority
	UART0->IER |= 1<<0;
	NVIC->ISER[0] |= 1<<12;
	NVIC->IP[3] &= ~(0b11<<6);
}
