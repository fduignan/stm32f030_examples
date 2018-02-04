// This is a test program that controls an array of 3 dotstar LED's over SPI.
// The LEDs are controlled using a data frame that begins with a sequence of the following
// form
// 0x00 0x00 0x00 0x00 ii bb gg rr 0xff 0xff 0xff 0xff
// The leading 4 zeros are a start of data frame delimiter.
// These are followed by a 1 byte intensity or brightness value which ranges from
// 0 to 31.  The most significant 3 bits of the brightness field must be 1
// Values in the range 0 to 255 follow for blue green and red
// Finally an end of frame consisting of 4 bytes of 0xff is sent.
// The test program below outputs signals that cycle 3 LED's through a range of colours
// The program was developed using an STM32F030F4P6 breakout board and ST-Link V2 
// SWD interface.
#include "stm32f030xx.h"
#include "spi.h"

void delay(int dly)
{
  while( dly--);
}

void initClock()
{
// This is potentially a dangerous function as it could
// result in a system with an invalid clock signal - result: a stuck system
        // Set the PLL up
        // First ensure PLL is disabled
        RCC_CR &= ~BIT24;
        while( (RCC_CR & BIT25)); // wait for PLL ready to be cleared
        // set PLL multiplier to 12 (yielding 48MHz)
  // Warning here: if system clock is greater than 24MHz then wait-state(s) need to be
        // inserted into Flash memory interface
        FLASH_ACR |= BIT0;
        FLASH_ACR &=~(BIT2 | BIT1);

        // Turn on FLASH prefetch buffer
        FLASH_ACR |= BIT4;
        RCC_CFGR &= ~(BIT21 | BIT20 | BIT19 | BIT18);
        RCC_CFGR |= (BIT21 | BIT19 ); 

        // Need to limit ADC clock to below 14MHz so will change ADC prescaler to 4
        RCC_CFGR |= BIT14;

// Do the following to push HSI clock out on PA8 (MCO)
// for measurement purposes.  Should be 8MHz or thereabouts (verified with oscilloscope)
/*
        RCC_CFGR |= ( BIT26 | BIT24 );
        RCC_AHBENR |= BIT17;
        GPIOA_MODER |= BIT17;
*/

        // and turn the PLL back on again
        RCC_CR |= BIT24;        
        // set PLL as system clock source 
        RCC_CFGR |= BIT1;
}

void configPins()
{
	// Power up PORTA
	RCC_AHBENR |= BIT17;
	GPIOA_MODER |= BIT8; // make bit 4 an output
	GPIOA_MODER &= ~BIT9; // make bit 4 an output (There's an LED on it)
	
}	
void startFrame()
{
    transferSPI(0x00);
    transferSPI(0x00);
    transferSPI(0x00);
    transferSPI(0x00);    
}
void endFrame()
{
    transferSPI(0xff);
    transferSPI(0xff);
    transferSPI(0xff);
    transferSPI(0xff);    
}
void writeColour(uint8_t level, uint8_t r,uint8_t g,uint8_t b)
{
    // Transfer a particular colour mix & brightness to all 3 leds
    startFrame();
    level = level | 0xe0;
    transferSPI(level);
    
    transferSPI(b);    
    transferSPI(g);
    transferSPI(r);
    
    transferSPI(level);
    transferSPI(b);    
    transferSPI(g);
    transferSPI(r);
    
    transferSPI(level);
    transferSPI(b);    
    transferSPI(g);
    transferSPI(r);
    
    endFrame();
}
void doRainbow(int brightness)
{   // Cycle through the colours of the rainbow (non-uniform brightness however)
	// Inspired by : http://academe.co.uk/2012/04/arduino-cycling-through-colours-of-the-rainbow/
	static unsigned Red = 255;
	static unsigned Green = 0;
	static unsigned Blue = 0;
	static int State = 0;
	switch (State)
	{
		case 0:{
			Green++;
			if (Green == 255)
				State = 1;
			break;
		}
		case 1:{
			Red--;
			if (Red == 0)
				State = 2;
			break;
		}
		case 2:{
			Blue++;
			if (Blue == 255)
				State = 3;			
			break;
		}
		case 3:{
			Green--;
			if (Green == 0)
				State = 4;
			break;
		}
		case 4:{
			Red++;
			if (Red == 255)
				State = 5;
			break;
		}
		case 5:{
			Blue --;
			if (Blue == 0)
				State = 0;
			break;
		}		
	}
	startFrame();
	transferSPI(brightness | 0xe0); // Maximum brightness
    transferSPI(Blue);
    transferSPI(Green);
    transferSPI(Red);
    
    transferSPI(brightness | 0xe0);
    transferSPI(Blue);
    transferSPI(Green);
    transferSPI(Red);	
    
    transferSPI(brightness | 0xe0);
    transferSPI(Blue);
    transferSPI(Green);
    transferSPI(Red);	
    endFrame();
}
int main()
{	
    initClock();
	configPins(); 
    initSPI();
	while(1)
	{			
		GPIOA_ODR |= BIT4;  // LED on
		delay(40000);
		GPIOA_ODR &= ~BIT4; // LED off
		delay(40000);
        doRainbow(15); // half brightness
	} 
	return 0;
}








