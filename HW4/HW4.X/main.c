// Demonstrates spi by accessing external dac
// PIC is the master, dac is the slave
// Uses microchip 23K256 ram chip (see the data sheet for protocol details)
// SDO4 -> SI (pin F5 -> pin 5)
// SDI4 -> SO (pin F4 -> pin 2)
// SCK4 -> SCK (pin B14 -> pin 6)
// SS4 -> CS (pin B8 -> pin 1)
// Additional SRAM connections
// Vss (Pin 4) -> ground
// Vcc (Pin 8) -> 3.3 V
// Hold (pin 7) -> 3.3 V (we don't use the hold function)
// 
// Only uses the SRAM's sequential mode
//


#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include <math.h>
#define SYS_FREQ 48000000 // system frequency 48MHz
#define CS LATBbits.LATB7       // chip select pin latb7
#define PI 3.1415926
// DEVCFG0
#pragma config DEBUG = OFF // no debugging
#pragma config JTAGEN = OFF // no jtag
#pragma config ICESEL = ICS_PGx1 // use PGED1 and PGEC1
#pragma config PWP = OFF // no write protect
#pragma config BWP = OFF // no boot write protect
#pragma config CP = OFF // no code protect

// DEVCFG1
#pragma config FNOSC = PRIPLL // use primary oscillator with pll
#pragma config FSOSCEN = OFF // turn off secondary oscillator
#pragma config IESO = OFF // no switching clocks
#pragma config POSCMOD = HS // high speed crystal mode
#pragma config OSCIOFNC = OFF // free up secondary osc pins //??????????????????????????
#pragma config FPBDIV = DIV_1 // divide CPU freq by 1 for peripheral bus clock
#pragma config FCKSM = CSDCMD // do not enable clock switch
#pragma config WDTPS = PS1048576 // slowest wdt
#pragma config WINDIS = OFF // no wdt window
#pragma config FWDTEN = OFF // wdt off by default
#pragma config FWDTWINSZ = WINSZ_25 // wdt window at 25%

// DEVCFG2 - get the CPU clock to 48MHz
#pragma config FPLLIDIV = DIV_2 // divide input clock to be in range 4-5MHz
#pragma config FPLLMUL = MUL_24 // multiply clock after FPLLIDIV
#pragma config FPLLODIV = DIV_2 // divide clock after FPLLMUL to get 48MHz
#pragma config UPLLIDIV = DIV_2 // divider for the 8MHz input clock, then multiply by 12 to get 48MHz for USB
#pragma config UPLLEN = ON // USB clock on

// DEVCFG3
#pragma config USERID = 0x0001 // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = OFF // allow multiple reconfigurations
#pragma config IOL1WAY = OFF // allow multiple reconfigurations
#pragma config FUSBIDIO = ON // USB pins controlled by USB module
#pragma config FVBUSONIO = ON // USB BUSON controlled by USB module

// send a byte via spi and return the response
unsigned char spi_io(unsigned char o) {
  SPI1BUF = o;
  while(!SPI1STATbits.SPIRBF) { // wait to receive the byte
    ;
  }
  return SPI1BUF;
}

// initialize spi1 
void initSPI1() {
  // set up the chip select pin as an output
  // when a command is beginning (clear CS to low) and when it
  // is ending (set CS high)
  TRISBbits.TRISB7 = 0; // select b7 pin as output 	
  CS = 1;// output high

  // Master - SPI1, pins are: SDI1(F4), SDO1(F5), SCK1(F13).  
  // we manually control SS4 as a digital output (F12)
  // since the pic is just starting, we know that spi is off. We rely on defaults here
 
  // setup spi1
  RPB8Rbits.RPB8R = 0b0011;	// set RPB8 as SDOut
  SPI1CON = 0;              // turn off the spi module and reset it
  SPI1BUF;                  // clear the rx buffer by reading from it
  SPI1BRG = 0x3;            // baud rate to 10 MHz [SPI4BRG = (80000000/(2*desired))-1]
  SPI1STATbits.SPIROV = 0;  // clear the overflow bit
  SPI1CONbits.CKE = 1;      // data changes when clock goes from hi to lo (since CKP is 0)
  SPI1CONbits.MSTEN = 1;    // master operation
  SPI1CONbits.ON = 1;       // turn on spi 1
  

                            // send a ram set status command.
//  CS = 0;                   // enable the ram
//  spi_io(0x01);             // ram write status
//  spi_io(0x41);             // sequential mode (mode = 0b01), hold disabled (hold = 0)
//  CS = 1;                   // finish the command
}

void setVoltage(char channel, unsigned int voltage) {
		
//	int dig_vol = 0;
//	int data_sent = 0;
//	dig_vol = (2^8 * floor(voltage / 3.3));
	switch(channel) {

	case 'A' :
   		//data_sent = 0x7000 | (dig_vol<<4);
		//data_sent = 0x70 | (voltage>>4);
        CS = 0;
        spi_io(0x70 | (voltage>>4));
        spi_io(voltage<<4);
        CS = 1;
    break; 
	
	case 'B' :
   		//data_sent = 0xF000 | (dig_vol<<4);
    	//data_sent = 0xF000 | (voltage<<4);
        CS = 0;
        spi_io(0xF0 | (voltage>>4));
        spi_io(voltage<<4);
        CS = 1;
    break; 

    }
//	CS = 0;
//	spi_io(data_sent & 0xFF00>> 8);  // the most significant byte
//	spi_io(data_sent & 0x00FF);      // the least significant byte    
//	CS = 1;  
}



int main(void) {
  char channel = 'A';
  float output_a, output_b = 0;
  float t1, t2 = 0;
  unsigned int output_a_dig,output_b_dig = 0;
	  
  
  TRISAbits.TRISA4 = 0;       // make A4(PORT 12) as output
 // NU32_Startup();   // cache on, interrupts on, LED/button init, UART init
  initSPI1(); 
  LATAbits.LATA4 = 1;
  // check the ram status
  
  CS = 0;  
  spi_io(0x70);                                   
  spi_io(0x00);      //write 16 bits into dac                      
  CS = 1;

  channel = 'A';
 
  setVoltage('A', 0x40);
  _CP0_SET_COUNT(0);//initialize the counter, half the speed of CPU
  while(1) {
      
      if( _CP0_GET_COUNT() >= (SYS_FREQ/1000/2)*1 )// count up to 1ms // greater than (SYS_FREQ /2*countingseconds)
            {
                output_a = 255 * (sin(2*PI*10*t1) + 1)/2;//sin(2*pi*frequency*t) for 8-bit resolution
                output_a_dig = (unsigned int) output_a;
                output_a_dig = output_a_dig & 0xFF;
                setVoltage('A', output_a_dig);

                output_b = 255 * t2 /0.2;// triangle wave
                if( t2 >= 0.2)
                    t2 = 0;
                
                output_b_dig = (unsigned int) output_b;
                output_b_dig = output_b_dig & 0xFF;
                setVoltage('B', output_b_dig);

                _CP0_SET_COUNT(0); // reset the counter
                
                t1 = t1 + 0.001;
                t2 = t2 + 0.001;
            }
 
        
  
  }
  return 0;
}
