/*
 * File:   whack_a_mole.c
 * Author: 18468242
 *
 * Created on 03 December 2020, 15:02
 */

#pragma config FOSC = XT        // Oscillator Selection bits (XT oscillator)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable bit (BOR disabled)
#pragma config LVP = OFF        // Low-Voltage (Single-Supply) In-Circuit Serial Programming Enable bit (RB3 is digital I/O, HV on MCLR must be used for programming)
#pragma config CPD = OFF        // Data EEPROM Memory Code Protection bit (Data EEPROM code protection off)
#pragma config WRT = OFF        // Flash Program Memory Write Enable bits (Write protection off; all program memory may be written to by EECON control)
#pragma config CP = OFF         // Flash Program Memory Code Protection bit (Code protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdio.h>      // Include Standard I/O header file
#include "I2C_EE302.h"
#include "ee302lcd.h"   // Include LCD header file. This file must be in same
                        // directory as source main source file.

#ifndef _XTAL_FREQ

 // Unless already defined assume 4MHz system frequency
 // This definition is required to calibrate the delay functions, __delay_us() and __delay_ms()
 #define _XTAL_FREQ 4000000
#endif 

#define CLOSED 0    //switch closed
#define OPEN 1      //switch open
#define POT RA0     //Potentionometer
#define SW1 RB0     //assign INC to RB0
#define SW2 RB1
#define SW3 RB2
#define SW4 RB3
#define LED1 RD0    //led 1
#define LED2 RD1    //led2
#define ON  1       //turn led on
#define OFF 0       //turn led off

char gOutString[16];   //store output string
//global values needed for timer interrupt
long gSpeed = 0;
unsigned int gFreqDivider = 1;
unsigned int gNum = 5;
unsigned int gUpdate = 1;
unsigned int gCheck = 0;
unsigned int gPlaying = 0;
unsigned int gStart = 0;
unsigned int gLoopCount = 0;
//global values for eeprom addresses
unsigned int EEPROM_ADDRESS_R = 0xA0;
unsigned int EEPROM_ADDRESS_W = 0xA0;

void setup(void);                   //setup
void superloop(void);                   //main loop
void lcdTitle(void);                //show title at startup
//USART functions
void USART_write(char); //write to putty
char USART_read(void);      //read input from putty
void USART_sequential_read(int len, char *);
void print_string(char*);   //write string to putty
//Game functions
int playGame(int length);
void print_lcd_score(int score);
void print_lcd_state(void);
void write_record(int score);
//i2c functions
void sequential_write(char * text, int);
void sequential_read(char * text, int);
//miscellaneous
int char_to_int(char array[], int len);
void clearOutString(void);          //clears gOutString
int rand(void);

void main(void) {
    
    setup();                        //setup
    i2c_init();
    Lcd8_Init();
    lcdTitle();                     //print title screen
    sequential_write("00", 2);    //preload eeprom with 0000 because read is called first
    
    for(;;){                        //superloop with small delay for stability
        superloop();
        __delay_ms(200);
    }
    return;
}

void setup(){                       //setup lcd and TRISB inputs
    
    TRISB = 0b00001111;             //SW1 input
    PORTB = 0xff;
    ADCON0 = 0b01000001;            //POT, selected CH0, conversion not im progress, power on
    ADCON1 = 0b00000111;            //LCD
    //USART setup
    TRISC = 0xD8; //RC6 and RC7 must be set to inputs for USART. RC3 and RC4 for I2C
    TRISD = 0x00; //SET TRIST D as output
    TXSTA = 0x24; //Set TXEN bit to enable transmit.
    //Set BRGH bit for Baud Rate table selection.
    RCSTA = 0b10110000; //Set CREN bit for continuous read.
    //Set SPEN bit to enable serial port.
    SPBRG = 0x19; //Set Baud Rate to 9600
    
    T1CON = 0b00110001; // Timer0 with external freq and 8 as prescalar, start timer1
    INTCON = 0b11000000;    //set GIE and PEIE
    TMR1 = 0;      // Load the time value(0x85EE) for 250ms delay (500ms for full cycle = 2Hz)
    TMR1IE=1;       //Enable timer interrupt bit in PIE1 register
    
    
    PORTD = 0xff; //SET port D as output
    
    
}

void lcdTitle(){                    //print title screen
    Lcd8_Write_String("EE302 Project");
    Lcd8_Set_Cursor(2, 0);
    Lcd8_Write_String("Whack-A-Mole");
}

void superloop(){    //calls main function
    static int length;  //stores length of game
    static int score = 0;   //stores score from game
    
    print_string("\r\nDial Potentionometer to desired speed and type in the duration of the game");
    //read 2 usart characters, convert to int
    
    __delay_ms(50);
    clearOutString();
    USART_sequential_read(2,gOutString);
    length = char_to_int(gOutString, 2);
    
    __delay_ms(50);
    //read potentionometer value and cast to 0-20000(a good value for speed)
    GO_nDONE = 1;   //set go and done
    while(GO_nDONE){    //wait
        continue;
    }
    gSpeed = ((ADRESH)/255.0)*(30000);
    
    __delay_ms(50);
    //print stuff
    clearOutString();
    sprintf(gOutString, "\r\nSpeed is: %ld, length is: %d", gSpeed, length);
    print_string(gOutString);
    print_string("\r\nclick button1 to start\r\n");
    
    while(SW1){                     //wait for SW1 to be pressed to start game
        continue;
    }
    __delay_ms(300);
    gLoopCount = 0;
    score = playGame(length);                        //play game and store score
    //print score
    clearOutString();
    sprintf(gOutString, "\r\nScore is: %d", score);
    print_string(gOutString);
    //find record with i2c and print to usart
    write_record(score);    
    return;
}
int playGame(int length){
    int btn = 4;    //button default value
    int score = 0; //track score
    gPlaying = 1;   //start timer interrupt
    gUpdate = 1;    //start lcd
    while(gLoopCount <= length){
        if(gUpdate){    //print lcd screen when update is ready
            print_lcd_state();  //if gNum == 5 do not light anything
            print_lcd_score(score);
            gUpdate = 0;
        }
        //assignt btn value from sw inputs
        if(!SW4){
            btn =0;
        }else if(!SW3){
            btn =1;
        } else if(!SW2){
            btn =2;
        }else if(!SW1){
            btn =3;
        } else {
            btn = 4;
        }
        __delay_ms(50);
        //if btn matches gNum, increment score. use gCheck to only execute once per gNum
        if(gNum == btn && gCheck == 0){
            gCheck = 1;
            score++;
        }
    }
    //update score at the end
    print_lcd_score(score);
    //reset game booleans
    gStart = 0;
    gPlaying = 0;
    gNum = 5; //default gNum value
    return score;
}
void write_record(int score){   //stores record in eeprom and checks score against it
    //store record value from eeprom and output array
    int record;
    
    clearOutString();
    sequential_read(gOutString, 2);  //write record to array
    __delay_ms(50);
    record = char_to_int(gOutString, 2); //convert char array to int
    __delay_ms(50);
    if(score > record){             //if score is the new record: update record
        record = score;  
        clearOutString();
        if(score<10){
            sprintf(gOutString, "0%d", score);    //convert int to string
        } else {
            sprintf(gOutString, "%d", score);    //convert int to string
        }
        sequential_write(gOutString, 2);
        __delay_ms(50);
    } else {
        EEPROM_ADDRESS_R -=2;           //if score isn't the new record, reset read address
    }
    //print record
    clearOutString();
    sprintf(gOutString, "\r\nRecord is: %d", record);
    print_string(gOutString);
}

int char_to_int(char array[], int len){ //convert char array to int
    int res = 0;
    for(int i = 0; i < len; i++){
        res = res*10+(array[i]-48);
    }
    return res;
}
void print_lcd_score(int score){    // write score to lcd
    Lcd8_Set_Cursor(1, 0);                      //move cursor
    clearOutString();                           //clear string
    sprintf(gOutString, "Score: %d", score);        //write score
    Lcd8_Write_String(gOutString);              //print
}
void print_lcd_state(){             // write '0' in a random place on lcd
    if(gNum<4){
        Lcd8_Set_Cursor(2, (5*gNum));                      //move cursor to random location
        Lcd8_Write_String("O");              //print
    } else{
        Lcd8_Clear();
    }
    __delay_ms(100);                            //delay 
}

void clearOutString(void) {                     //clear string of length 16
    for (int i = 0; i < 16; i++)
    {
        gOutString[i] = 0x00;
    }
}

//USART functions
void USART_write(char data){
    while(!TXIF){
        continue;
    }// set when transmit register (TXREG) is empty
    TXREG = data;
        // Once TXREG is loaded the byte is automatically
        // transmitted by the USART peripheral
    
}
char USART_read(){
  while(!RCIF);     //set while empty
  return RCREG;     //return character
}

void print_string(char *data){//print string
    for(int i = 0; data[i]!='\0'; i++){//iterate over string until reaches end
        USART_write(data[i]);   //print each byte individually
        
    }
}
void USART_sequential_read(int len, char array[]){    //read messge of from usart sequentially
    for(int i = 0; i< len; i++){
        array[i]=USART_read();
        __delay_ms(50);
    }
}
//I2C functions
void sequential_write(char * text, int len){
    for(int i = 0; i < len;i++){
        write_ext_eeprom(EEPROM_ADDRESS_W, (unsigned char)(EEPROM_ADDRESS_W>>8), text[i]);//bitshift adress by a byte instead of having 2 different values
        EEPROM_ADDRESS_W++;//increment address
    }
}
void sequential_read(char * text, int len){
    for(int i = 0; i < len; i++){
        text[i] = read_ext_eeprom(EEPROM_ADDRESS_R,((unsigned char)(EEPROM_ADDRESS_R>>8)));//bitshift adress by a byte instead of having 2 different values
        EEPROM_ADDRESS_R++;//increment address
    }
}
void __interrupt() //interrupt
timer_isr()
{  
    if(TMR1IF==1)   //when timer1 overflows
    {
        if(gPlaying == 1){  //if playing game
            if(gFreqDivider==0 && gStart == 1){ 
                //reduce speed of program with gFreqDivider because timer1 min speed is too fast
                //use gStart to add delay after clicking SW1
                if(gNum<4){     //add a gap between printing '0' to lcd
                    gNum = 5;       //default gNum value
                    gUpdate = 1;    //indicate lcd update
                }else {         
                    gNum = rand() % 4;//set gNum to random value
                    gCheck = 0;       //indicate that a new value was set
                    gUpdate = 1;      //indicate lcd update
                    gLoopCount++;     //increment loop count
                }
                gFreqDivider = 1;     //reset freq divider
            } else {                  //decrement freq divider and set start
                gFreqDivider--;
                gStart = 1;
            }
        }
        TMR1=gSpeed;    //update timer1 with timer
        TMR1IF=0;       // Clear timer interrupt flag
    }
}
