// This is a test program for use with a low energy weather station based
// around the STM32F030 and an NRF905 radio module.  The sensing module is
// a GY-652 (HMC5983 + BMP180) which uses an I2C interface.
// The readings for this sensor will be transmitted periodically over
// the radio link
// The STM32F030 runs at the default speed of 8MHz on its internal oscillator.
// A serial interface is provided for debugging purposes.
// This version (0.1) does not take much heed of power saving - that will come later.
//
// This test program was used to develop the code for the BMP180 pressure/temperature sensor
//
/* Wiring : This needs to change due to io conflicts
 
 STM32F030          NRF905
 PA1                DR
 PA2                Pwr  ---> PF0
 PA3                CE   ---> PF1
 PA4                CSN
 PA5                SPI SCLK
 PA6                SPI MISO
 PA7                SPI MOSI
 PB1                TXEn
 
 UART Interface
 PA9                UART TX  ---> PA2
 PA10               UART RX  ---> PA3
 
 BMP180/HMC5983 I2C interface wiring
 I2C SDA   PA10
 I2C SCL   PA9 
 
 
 */ 
#include "stm32f030xx.h"
#include "serial.h"
#include "i2c.h"

// Uncomment the line below to use test data and calibration values from the BMP180 datasheet
//#define TEST_DATA 1
char Msg[32]="Hello World";
uint8_t Buffer[32];
void delay(int dly)
{
  while( dly--);
}
void configPins()
{
  // Power up PORTA
  RCC_AHBENR |= BIT17;	
}	

void dumpI2C1Registers()
{
// Used for debugging only    
    eputs("\r\nGPIOA_MODER:");
    printHex(GPIOA_MODER);
    drainUART();
    eputs("\r\nGPIOA_AFRH:");
    printHex(GPIOA_AFRH);
    drainUART();    
    eputs("\r\nCR1:");
    printHex(I2C1_CR1);
    drainUART();
    eputs("\r\nCR2:");
    printHex(I2C1_CR2);
    drainUART();
    eputs("\r\nOAR1:");    
    printHex(I2C1_OAR1);
    drainUART();
    eputs("\r\nOAR2:");    
    printHex(I2C1_OAR2);
    drainUART();
    eputs("\r\nTIMINGR:");    
    printHex(I2C1_TIMINGR);
    drainUART();
    eputs("\r\nTIMEOUTR:");    
    printHex(I2C1_TIMEOUTR);
    drainUART();
    eputs("\r\nISR:");    
    printHex(I2C1_ISR);
    drainUART();
    eputs("\r\nICR:");    
    printHex(I2C1_ICR);
    drainUART();
    eputs("\r\nPECR:");    
    printHex(I2C1_PECR);
    drainUART();
    eputs("\r\nRXDR:");    
    printHex(I2C1_RXDR);
    drainUART();
    eputs("\r\nTXDR:");    
    printHex(I2C1_TXDR);
    drainUART();
}
void printByteArray(uint8_t *Ary,int len)
{
    eputs("\r\n");
    while (len--)
    {
        printByte(*Ary++);
        eputs(" ");
    }
}
uint8_t readBMPRegister(uint8_t RegNum)
{    
  I2CTransaction Transaction;    
  Transaction.Mode = 'r';
  Transaction.SlaveAddress = 0x77;
  Transaction.TXCount = 1;
  Transaction.RXCount = 1;
  Transaction.TXData[0] = RegNum;  
  I2CDoTransaction(&Transaction);
  return Transaction.RXData[0];
}
typedef struct {
    int16_t AC1;
    int16_t AC2;
    int16_t AC3;
    uint16_t AC4;
    uint16_t AC5;
    uint16_t AC6;
    int16_t B1;
    int16_t B2;
    int16_t MB;
    int16_t MC;
    int16_t MD;
} CalibrationData_t;
CalibrationData_t CalibrationData;

uint8_t writeBMPRegister(uint8_t RegNum,uint8_t Value)
{    
  I2CTransaction Transaction;    
  Transaction.Mode = 'w';
  Transaction.SlaveAddress = 0x77;
  Transaction.TXCount = 2;
  Transaction.RXCount = 0;
  Transaction.TXData[0] = RegNum;
  Transaction.TXData[1] = Value;    
  return I2CDoTransaction(&Transaction);  
}
void readBMPRegisters(uint8_t * Registers)
{
    Registers[0]=readBMPRegister(0xf8);
    Registers[1]=readBMPRegister(0xf7);
    Registers[2]=readBMPRegister(0xf6);
    Registers[3]=readBMPRegister(0xf4);
    Registers[4]=readBMPRegister(0xe0);
    Registers[5]=readBMPRegister(0xd0);
}
void readBMPCalibrationData()
{
    CalibrationData.AC1 = ((int16_t)readBMPRegister(0xaa)<<8)+(int16_t)readBMPRegister(0xab);
    CalibrationData.AC2 = ((int16_t)readBMPRegister(0xac)<<8)+(int16_t)readBMPRegister(0xad);
    CalibrationData.AC3 = ((int16_t)readBMPRegister(0xae)<<8)+(int16_t)readBMPRegister(0xaf);
    CalibrationData.AC4 = ((uint16_t)readBMPRegister(0xb0)<<8)+(uint16_t)readBMPRegister(0xb1);
    CalibrationData.AC5 = ((uint16_t)readBMPRegister(0xb2)<<8)+(uint16_t)readBMPRegister(0xb3);
    CalibrationData.AC6 = ((uint16_t)readBMPRegister(0xb4)<<8)+(uint16_t)readBMPRegister(0xb5);
    CalibrationData.B1 = ((int16_t)readBMPRegister(0xb6)<<8)+(int16_t)readBMPRegister(0xb7);
    CalibrationData.B2 = ((int16_t)readBMPRegister(0xb8)<<8)+(int16_t)readBMPRegister(0xb9);
    CalibrationData.MB = ((int16_t)readBMPRegister(0xba)<<8)+(int16_t)readBMPRegister(0xbb);
    CalibrationData.MC = ((int16_t)readBMPRegister(0xbc)<<8)+(int16_t)readBMPRegister(0xbd);
    CalibrationData.MD = ((int16_t)readBMPRegister(0xbe)<<8)+(int16_t)readBMPRegister(0xbf);
#ifdef TEST_DATA  
  // Test data from data sheet
    CalibrationData.AC1 = 408;
    CalibrationData.AC2 = -72;
    CalibrationData.AC3 = -14383;
    CalibrationData.AC4 = 32741;
    CalibrationData.AC5 = 32757;
    CalibrationData.AC6 = 23153;
    CalibrationData.B1 = 6190;
    CalibrationData.B2 = 4;
    CalibrationData.MB = -32768;
    CalibrationData.MC = -8711;
    CalibrationData.MD = 2686;
#endif    
}    
int readTemperature()
{
    long RawTemp;
    long Temperature,X1,X2,B5;
    writeBMPRegister(0xf4,0x2e);
    // short delay for conversion
    int TimeOut=0x100;
    while((TimeOut--) && (readBMPRegister(0xf4)&0x20) );    
    
    RawTemp=readBMPRegister(0xf6);
    RawTemp <<= 8;
    RawTemp+=readBMPRegister(0xf7); 
#ifdef TEST_DATA
    RawTemp = 27898;
#endif         
    X1 = (((long)RawTemp-(long)CalibrationData.AC6)*(long)CalibrationData.AC5)>>15;
	X2 = ((long)CalibrationData.MC << 11)/((long)X1 + (long)CalibrationData.MD);
	B5 = X1 + X2;
	Temperature = (B5+8)>>4;    
    return Temperature;    
}
int readPressure()
{
    // Need to read the temperature and then the pressure.
    long RawTemp;
    long Temperature,X1,X2,X3,B3,B5,B6,Pressure;
    unsigned long B4,B7;
    writeBMPRegister(0xf4,0x2e);
    delay(1000); // short delay for conversion
    int TimeOut=0x100;
    while((TimeOut--) && (readBMPRegister(0xf4)&0x20));        
    RawTemp=readBMPRegister(0xf6);
    RawTemp <<= 8;
    RawTemp+=readBMPRegister(0xf7);        
#ifdef TEST_DATA
    RawTemp = 27898;
#endif     
    X1 = (((long)RawTemp-(long)CalibrationData.AC6)*(long)CalibrationData.AC5)>>15;
	X2 = ((long)CalibrationData.MC << 11)/((long)X1 + (long)CalibrationData.MD);
	B5 = X1 + X2;   
    B6 = B5 - 4000;    
    // Now read the raw pressure.  Will use oversampling of 0 to save power    
    writeBMPRegister(0xf4,0x34);
    delay(1000);  // short delay for conversion
    TimeOut=0x100;    
    while((TimeOut--) && (readBMPRegister(0xf4)&0x20) );    
    long RawPressure = ((long)readBMPRegister(0xf6))<<16;    
    RawPressure += ((long)readBMPRegister(0xf7)) << 8;
    RawPressure += ((long)readBMPRegister(0xf8));
    RawPressure >>= 8;
#ifdef TEST_DATA
     RawPressure = 23843; // test data from data sheet
#endif       
    X1 =((long)CalibrationData.B2 * (B6 * B6) >> 12 ) >> 11;        
    X2 = ((long)CalibrationData.AC2 * B6) >> 11;    
    X3 = X1 + X2;    
    B3 = (((long)CalibrationData.AC1*4+X3)+2) >> 2;    
    X1 = ((long)CalibrationData.AC3*B6) >> 13;
    X2 = ((long)CalibrationData.B1*(B6*B6 >> 12))>>16;        
    X3 = ((X1+X2)+2) >> 2;    
    B4 = ((CalibrationData.AC4*(unsigned long)(X3+32768)))>>15;    
    B7 = ((unsigned long)RawPressure-B3)*50000UL;
    if (B7 < 0x80000000)
    {
        Pressure = (B7 * 2)/B4;        
    }
    else
    {
        Pressure = (B7 / B4) << 1;        
    }    
    X1 = (Pressure >> 8) * (Pressure >> 8);    
    X1 = (X1 * 3038)>>16;    
    X2 = (-7357*Pressure) >> 16;    
    Pressure = Pressure + ( (X1+X2+3791)  >> 4 );
    return Pressure;    
    
}
void ShowCalibrationData()
{
    // Used during debugging only - check to see if the 
    // calibration data were read correctly
    int i=0;    
    eputs("\r\n");
    // print the individual 8 bit registers
    uint8_t Addr = 0xaa;
    for (i=0;i<22;i++)
    {
        printByte(readBMPRegister(Addr++));
        eputs(" ");
        drainUART();
    }
    eputs("\r\n");
    // print the 16 bit signed/unsigned co-efficients
    printDecimal(CalibrationData.AC1);
    eputs("\r\n");
    drainUART();
    printDecimal(CalibrationData.AC2);
    eputs("\r\n");
    drainUART();
    printDecimal(CalibrationData.AC3);
    eputs("\r\n");
    drainUART();
    printDecimal(CalibrationData.AC4);
    eputs("\r\n");
    drainUART();
    printDecimal(CalibrationData.AC5);
    eputs("\r\n");
    drainUART();
    printDecimal(CalibrationData.AC6);
    eputs("\r\n");
    drainUART();
    printDecimal(CalibrationData.B1);
    eputs("\r\n");
    drainUART();
    printDecimal(CalibrationData.B2);
    eputs("\r\n");
    drainUART();
    printDecimal(CalibrationData.MB);
    eputs("\r\n");
    drainUART();
    printDecimal(CalibrationData.MC);
    eputs("\r\n");
    drainUART();
    printDecimal(CalibrationData.MD);
    eputs("\r\n");
    drainUART();

}
int main()
{  
  int Count = 0;
  configPins();
  initUART(9600);    
  initI2C();
  enable_interrupts();
  eputs("Starting\r\n");
  readBMPCalibrationData();            
  ShowCalibrationData();
  while(1) {                  
    printDecimal(readTemperature());
    eputs(" ");
    printDecimal(readPressure());
    eputs("\r\n");
    drainUART();
  }
  return 0;
}
