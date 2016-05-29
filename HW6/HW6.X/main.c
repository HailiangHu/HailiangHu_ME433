#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include"i2c_master_noint.h"

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
#pragma config OSCIOFNC = OFF // free up secondary osc pins
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
#pragma config FUSBIDIO = ON// USB pins controlled by USB module
#pragma config FVBUSONIO = ON // USB BUSON controlled by USB module




void i2c_init(void){
    ANSELBbits.ANSB2 = 0;
    ANSELBbits.ANSB3 = 0;
    i2c_master_setup();
}

void i2c_imu_init(void){
    i2c_master_start();
    i2c_master_send(0xD6); //0b1101011 + 0write 11010110: D6
    i2c_master_send(0x10);  // Reg ADDRESS CTRL1_XL (10h)
    i2c_master_send(0x80); //1000 00 00
    i2c_master_stop();
    //set latch
   
    i2c_master_start();
    i2c_master_send(0xD6); //0b1101011 + 0write 11010110: D6
    i2c_master_send(0x11);  // Reg ADDRESS CTRL2_G (11h)
    i2c_master_send(0x80); //1000 00 00
    i2c_master_stop();
    
    i2c_master_start();
    i2c_master_send(0xD6); //0b1101011 + 0write 11010110: D6
    i2c_master_send(0x12);  // Reg ADDRESS CTRL3_C (12h)
    i2c_master_send(0x04); //0000 0100
    i2c_master_stop();
}




I2C_read_multiple(char address, char register, unsigned char * data, char length)
{


}



int main() {
    unsigned char master_read  = 0x00; 

    __builtin_disable_interrupts();

    // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;

    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;

    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;   
     
     TRISAbits.TRISA4 = 1; //set port A4 (PIN12)as input pin for button input
     TRISBbits.TRISB4 = 0; //set port B4 (PIN11)as output pin for LED
    
     
     i2c_init();
     i2c_imu_init();
     
    __builtin_enable_interrupts();
    
    //WHO_AM_I (0Fh)
    i2c_master_start();
    i2c_master_send(0xD6);
    i2c_master_send(0x0F);
    i2c_master_restart();
    i2c_master_send(0xD7);
    unsigned char input=i2c_master_recv();
    i2c_master_ack(1);
    i2c_master_stop();
    
    
   
    
    
    
    
}