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
#include "ILI9163C.h"
#define SYS_FREQ 48000000 // system frequency 48MHz
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


void LCD_drawChar(unsigned short x, unsigned short y, char word,unsigned short color) {
    
    int i,j;
    for (i=0;i<5;i++)
    {
        for (j=0;j<8;j++)
        {
            if((ASCII[word - 0x20][i]>>j) % 2 != 0)//ascii && (0x01<<j)
            {
                LCD_drawPixel(x+i, y+j, RED);        
            }
        }
    }
}

void LCD_drawMessage(unsigned short x, unsigned short y, char message[20],unsigned short color) 
{
    
    int i = 0; 
    while(message[i])
    { 
        if(x>123)
        {
            y = y+8;
            x = 0;
        }
        LCD_drawChar(x,y,message[i],color); 
        i++;
        x = x+6;
        
    }
}

int main(void) {
    
    char message[20];
    unsigned short x,y;
    char word;
    TRISAbits.TRISA4 = 0;       // make A4(PORT 12) as output
    LATAbits.LATA4 = 1;
    
    SPI1_init();
    LCD_init();
    LCD_clearScreen(CYAN);//BLACK WHITE BLUE RED GREEN CYAN MAGENTA YELLOW 
    
    word = 'd';
    x = 28;
    y = 32;
    //LCD_drawChar(x, y, word,RED);   
    sprintf(message,"Hello world 1337!");
    LCD_drawMessage( x, y, message,GREEN);
    
    
    return 0;
}
