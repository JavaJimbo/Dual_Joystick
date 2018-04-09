/*  XBEE Dual Joystick - For XBEE Dual Joystick board, V1.0
 *  No USB version
 *  C language written for for 18F2520 on XC8 Compiler V1.41 & MPLABX V4.01
 *  Reads four pots on two joysticks, transmits data via XBEE
 * 
 *  4-5-18: Adapted from USB project, runs 4 Wheeler
 *  4-6-18: Moved motor math from joystick to Quad board, so joystick data is transmitted.
 *  4-8-18: Works with Quad Motor 220 steering forward-reverse-right-left on four wheels. 
 */

#include <xc.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include "delay.h"
#pragma config IESO = OFF, OSC = HS, FCMEN = OFF, BOREN = OFF, PWRT = ON, WDT = OFF, CCP2MX = PORTC, PBADEN = OFF, LPT1OSC = OFF, MCLRE = ON, DEBUG = OFF, STVREN = OFF, XINST = OFF, LVP = OFF, CP0 = OFF, CP1 = OFF, CP2 = OFF, CP3 = OFF  // For 18F2520

#define STX 36
#define ETX 13
#define DLE 16


#define leftJoystickY   ADresult[0]
#define leftJoystickX   ADresult[1]
#define rightJoystickY  ADresult[2]
#define rightJoystickX  ADresult[3]

#define false 0
#define true !false
#define TRUE true
#define FALSE false

#define SleepControl 	PORTAbits.RA5
#define TX_OUT 		LATAbits.LATA0
#define PUSHin 		PORTBbits.RB0
#define TRIG_OUT    PORTBbits.RB5

#define ORIENTATION_BIT 0x10
#define MOTION_BIT 0x04
#define INTERRUPT_STATUS ORIENTATION_BIT

#define NUM_AD_CHANNELS 4
unsigned short ADresult[NUM_AD_CHANNELS] = {0, 0, 0, 0};

void init(void);

extern unsigned short CRCcalculate (unsigned char *message, unsigned char nBytes);

#define MAXPACKET 32
unsigned char arrPacket[MAXPACKET];

#define MAXPACKETBYTES 1024
unsigned char arrDataPacket[MAXPACKETBYTES];
#define MAX_DATA_BYTES 64
unsigned char commandBuffer[MAX_DATA_BYTES];   
unsigned short createDataPacket(unsigned char *ptrData, unsigned short numDataBytes, unsigned char *ptrPacket);

#define HIGH_STATE 1
#define LOW_STATE 0

unsigned char insertByte(unsigned char dataByte, unsigned char *ptrBuffer, unsigned char *index);
// unsigned char BuildPacket(unsigned char command, unsigned char subCommand, unsigned char dataLength, unsigned char *ptrData, unsigned char *ptrPacket);
unsigned char BuildPacket(unsigned char dataLength, unsigned char *ptrData, unsigned char *ptrPacket);
void ADsetChannel(unsigned char channel);
short ADreadResult(void);
void putch(unsigned char TxByte);
void readJoySticks(void);

unsigned char PORTBreg;
unsigned char pushFlag = FALSE;
unsigned char Timer2flag = FALSE;

#define ROOMBA 0
#define DRIVEDIRECT 145

void main() {        
short intLeftJoystickY, intLeftJoystickX, intRightJoystickY, intRightJoystickX;
unsigned char arrTransmitData[16];

#define CommandByte      arrTransmitData[0]
#define SubCommandByte   arrTransmitData[1]
#define TransmitLength   arrTransmitData[2]
#define LSBLeftJoystickX arrTransmitData[3]
#define MSBLeftJoystickX arrTransmitData[4]
#define LSBLeftJoystickY arrTransmitData[5]
#define MSBLeftJoystickY arrTransmitData[6]
#define LSBRightJoystickX arrTransmitData[7]
#define MSBRightJoystickX arrTransmitData[8]
#define LSBRightJoystickY arrTransmitData[9]
#define MSBRightJoystickY arrTransmitData[10]
#define LSB_CRCresult     arrTransmitData[11]
#define MSB_CRCresult     arrTransmitData[12]

unsigned char packetLength, i;
unsigned char LEDcounter = 0;
union convertType {
    unsigned char byte[2];
    int integer;
} convert;

    
    init();
    DelayMs(200);
    PORTBbits.RB0 = PORTBbits.RB1 = PORTBbits.RB2 = 1;    

        
    while(1)
    {                       
        if (Timer2flag)
        {
            Timer2flag = FALSE;
            LEDcounter++;
            if (LEDcounter == 4)
            {
                PORTBbits.RB0 = 1;
                PORTBbits.RB1 = 0;
                PORTBbits.RB2 = 0;
            }            
            else if (LEDcounter == 8)
            {
                PORTBbits.RB0 = 0;
                PORTBbits.RB1 = 1;
                PORTBbits.RB2 = 0;
            }            
            else if (LEDcounter >= 12)
            {
                PORTBbits.RB0 = 0;
                PORTBbits.RB1 = 0;
                PORTBbits.RB2 = 1;
                LEDcounter = 0;
            }           
            
            readJoySticks();           
                            
            intLeftJoystickX = ((short) leftJoystickX) - 127;    
            intLeftJoystickY = ((short) leftJoystickY) - 127;
            intRightJoystickX = ((short) rightJoystickX) - 127; 
            intRightJoystickY = ((short) rightJoystickY) - 127;        

            convert.integer = intLeftJoystickX;
            LSBLeftJoystickX = convert.byte[0];
            MSBLeftJoystickX = convert.byte[1];

            convert.integer = intLeftJoystickY;
            LSBLeftJoystickY = convert.byte[0];
            MSBLeftJoystickY = convert.byte[1];
            
            convert.integer = intRightJoystickX;
            LSBRightJoystickX = convert.byte[0];
            MSBRightJoystickX = convert.byte[1];
            
            convert.integer = intRightJoystickY;
            LSBRightJoystickY = convert.byte[0];
            MSBRightJoystickY = convert.byte[1];           
            
            CommandByte = ROOMBA;
            SubCommandByte = DRIVEDIRECT;   
            TransmitLength = 13;
            
            convert.integer = CRCcalculate(arrTransmitData, TransmitLength-2);
            LSB_CRCresult = convert.byte[0];
            MSB_CRCresult = convert.byte[1];            
            packetLength = BuildPacket(TransmitLength, arrTransmitData, arrPacket);        // Construct arrPacket
            if (packetLength < MAXPACKET) for (i = 0; i < packetLength; i++) putch(arrPacket[i]);  // Transmit arrPacket
        }
    }
}

void init(void) {
    INTCONbits.GIE = 0; // Global interrupt disabled

    // Initialize ports
    ADCON0 = 0b00000000; // Turn off A/D for now.
    
    // Set up 18F2520 for four analog inputs, use VCC and VSS for references.        
    ADCON1bits.VCFG0 = 0;
    ADCON1bits.VCFG1 = 0;
    ADCON1bits.PCFG3 = 1;
    ADCON1bits.PCFG2 = 0;
    ADCON1bits.PCFG1 = 1;
    ADCON1bits.PCFG0 = 1;
    
    ADCON2 = 0;             // Clear A/D control register 2
    ADCON2bits.ADFM = 1;    // Right justified A/D result
    
    ADCON2bits.ACQT2 = 1;   // Acquisition time max
    ADCON2bits.ACQT1 = 1;
    ADCON2bits.ACQT0 = 1;
    
    ADCON2bits.ADCS2 = 1; // FRC is conversion clock 
    ADCON2bits.ADCS1 = 1;
    ADCON2bits.ADCS0 = 1;    
    TRISA = 0b11011111; // Set PORT A to all input except for SLEEP pin RA5
    SleepControl = 1;


    TRISB = 0b11111000; // RB0,RB1,RB2 are LED outputs, RB7,RB6,RB5 are pushbutton inputs
    INTCON2bits.RBPU = 0; // Enable pullups
    TRISC = 0b10111111; // RC6 is TX output
        
    // Set up the UART         
    TXSTAbits.BRGH = 1; // High speed baud rate
    SPBRG = 19;         // Baudrate = 57600 @ 18.432 Mhz clock
    TXSTAbits.SYNC = 0; // asynchronous
    RCSTAbits.SPEN = 1; // enable serial port pins
    RCSTAbits.CREN = 1; // enable reception
    RCSTAbits.SREN = 0; // no effect
    PIE1bits.TXIE = 0; // disable tx interrupts 
    PIE1bits.RCIE = 0; // disable rx interrupts 
    TXSTAbits.TX9 = 0; // 8-bit transmission
    RCSTAbits.RX9 = 0; // 8-bit reception
    TXSTAbits.TXEN = 1; // Enable transmitter
    BAUDCONbits.TXCKP = 0; // Do not invert transmit and receive data
    BAUDCONbits.RXDTP = 0;

    // Set up Timer 2 to roll over every 1 millisecond:
    PR2 = 72;              // For 1 ms rolloever
    T2CON = 0x00;
    T2CONbits.T2CKPS1 = 0; // 1:4 prescaler  was 1:1 for 1 ms rolloever
    T2CONbits.T2CKPS0 = 1;
    T2CONbits.T2OUTPS3 = 1; // 1:16 postscaler
    T2CONbits.T2OUTPS2 = 1;
    T2CONbits.T2OUTPS1 = 1;
    T2CONbits.T2OUTPS0 = 1;        
    T2CONbits.TMR2ON = 1; // Timer 2 ON 

    // Set up interrupts. 
    INTCON = 0x00;          // Clear all interrupts
    INTCONbits.INT0IE = 0;  // Disable pushbutton interrupts
    INTCONbits.RBIE = 0;    // Disable PORT B change interrrupts
    PIE1bits.TMR2IE = 1;    // Enable Timer 2 interrupts
    INTCONbits.PEIE = 1;    // Enable peripheral interrupts for Timer 2

    INTCON2 = 0x00;
    INTCON2bits.RBPU = 0;       // Enable Port B pullups 
    INTCON2bits.INTEDG0 = 0;    // Interrupt on falling edge of RB0 pushbutton wakes up PIC from sleep mode.
    INTCONbits.GIE = 1;         // Enable global interrupts           
}



static void interrupt isr(void) {
    static int Timer2Counter = 0;
    
    if (INTCONbits.INT0IF) 
    {
        INTCONbits.INT0IF = 0;
        pushFlag = TRUE;
    }
    
    if (INTCONbits.RBIF) 
    {
        PORTBreg = PORTB;
        INTCONbits.RBIF = 0;
    }
    
    if (TMR2IF)
    {
        TMR2IF = 0;
        Timer2Counter++;
        if (Timer2Counter >= 100)
        {
            Timer2Counter = 0;
            Timer2flag = TRUE;
        }
    }
}

void putch(unsigned char TxByte) {
    while (!PIR1bits.TXIF); // set when register is empty 
    TXREG = TxByte;
    return;
}



void ADsetChannel(unsigned char channel) {
    ADCON0 = (channel << 2) + 0x01; // enable ADC, RC osc.
}

short ADconvertAndRead(void) {
    unsigned short highByte;
    short ADvalue;
    ADCON0bits.GO_DONE = 1;
    while (ADCON0bits.GO_DONE);
    highByte = (unsigned short) ADRESH;
    highByte = (highByte << 8) & 0b001100000000;
    ADvalue = (short) (highByte | ADRESL);
    return (ADvalue);
}

void readJoySticks(void) {
    static unsigned char ADchannel = 0;
    const unsigned short ADoffset[] = {450, 400, 430, 400};
    const unsigned short ADspan[] = {150, 200, 200, 200};
    unsigned short joyStickReading, offset, span;      
    short ADrawReading;
        
        ADrawReading = ADconvertAndRead();
    
        if (ADchannel == 0 || ADchannel == 2) joyStickReading = 1023 - ADrawReading;
        else joyStickReading = ADrawReading;

        offset = ADoffset[ADchannel];
        span = ADspan[ADchannel];
        if (joyStickReading < offset) joyStickReading = 0;
        else joyStickReading = joyStickReading - offset;
        if (joyStickReading > span) joyStickReading = span;

        ADresult[ADchannel] = (joyStickReading * 255) / span;
        if (ADresult[ADchannel] > 255) ADresult[ADchannel] = 255;
        
        ADchannel++;
        if (ADchannel >= NUM_AD_CHANNELS) ADchannel = 0;
        ADsetChannel(ADchannel);
}


unsigned char insertByte(unsigned char dataByte, unsigned char *ptrBuffer, unsigned char *index) 
{
    if (*index >= MAXPACKET) return (FALSE);
    if (dataByte == STX || dataByte == DLE || dataByte == ETX) {
        ptrBuffer[*index] = DLE;
        *index = *index + 1;
    }
    if (*index >= MAXPACKET) return (FALSE);
    ptrBuffer[*index] = dataByte;
    *index = *index + 1;
    return (TRUE);
}


unsigned char BuildPacket(unsigned char dataLength, unsigned char *ptrCommandData, unsigned char *ptrPacket) 
{
    unsigned char packetIndex = 0, i;

    if (dataLength <= MAXPACKET) {
        ptrPacket[packetIndex++] = STX;        
        for (i = 0; i < dataLength; i++)
            if (!insertByte(ptrCommandData[i], ptrPacket, &packetIndex)) return(0);        
        ptrPacket[packetIndex++] = ETX;
        return (packetIndex);
    } else return (0);
}

