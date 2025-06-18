// LCD Initialization and Display Message...
#include <xc.h>
#include <stdint.h>
#include <stdio.h>

// CONFIGURATION BITS
#pragma config FOSC = HS, WDTE = OFF, PWRTE = ON, BOREN = ON, LVP = OFF, CPD = OFF, WRT = OFF, CP = OFF

#define _XTAL_FREQ 20000000

// LCD Pins
#define RS RD0
#define RW RD1
#define EN RD2
#define LCD_DATA PORTB
#define LCD_DIR TRISB
// DHT11
#define  DHT11_Pin PORTDbits.RD5
#define DHT11_Pin_Dir TRISDbits.TRISD5

// FAN
#define FAN PORTCbits.RC0


// RGB LEDs on PORTD pins
#define RED_LED   PORTDbits.RD3
#define GREEN_LED PORTDbits.RD4
#define BLUE_LED  PORTDbits.RD6   // Optional
// UART functions
void UART_Init() {
    TRISC6 = 0; // TX output
    TRISC7 = 1; // RX input
    SPBRG = 129; // For 9600 baud at 20MHz
    BRGH = 1;
    SYNC = 0;
    SPEN = 1;
    TXEN = 1;
    CREN = 1;
}
void UART_Write(char data) {
    while (!TXIF);
    TXREG = data;
}

void UART_Write_Text(const char* text) {
    while (*text) UART_Write(*text++);
}

char UART_Read() {
    while (!RCIF);
    return RCREG;
}


// LCD Functions
void lcd_cmd(unsigned char cmd) {
    RS = 0; RW = 0;
    LCD_DATA = cmd;
    EN = 1; __delay_ms(2); EN = 0;
}

void lcd_data(unsigned char data) {
    RS = 1; RW = 0;
    LCD_DATA = data;
    EN = 1; __delay_ms(2); EN = 0;
}

void lcd_puts(const char* str) {
    while (*str) lcd_data(*str++);
}

void lcd_init() {
    TRISD0 = TRISD1 = TRISD2 = 0;
    LCD_DIR = 0x00;
    __delay_ms(20);
    lcd_cmd(0x38); // 8-bit, 2 line, 5x8 font
    lcd_cmd(0x0C); // Display ON, cursor OFF
    lcd_cmd(0x06); // Entry mode, auto increment cursor
    lcd_cmd(0x01); // Clear display
    __delay_ms(2);
}
//dh11 sensor functions
unsigned char RH_int, RH_dec, Temp_int, Temp_dec, checksum;
char Check;
void dht11_start(){
 DHT11_Pin_Dir = 0;//Con pin is output
DHT11_Pin=0;//pin is low start signal
__delay_ms(18);
DHT11_Pin=1;//pin is high to relese the line
__delay_us(30);
DHT11_Pin_Dir = 1;//Con pin is input (to read response)
}
void dht11_response(){
Check=0;//reset  
__delay_us(40);
if (DHT11_Pin==0){   //cheks if pin is low it recived start signal 
        __delay_us(80);     
        
        if (DHT11_Pin == 1) //check if pin is high to send data
            Check = 1;       
        
        __delay_us(50);    
    }
}
char dht11_read_byte() {
    char data = 0;
    for (char i = 0; i < 8; i++) {
        while (!DHT11_Pin); // wait for pin to go high
        __delay_us(30);
        if (DHT11_Pin == 0)
            data &= ~(1 << (7 - i));
        else {
            data |= (1 << (7 - i));
            while (DHT11_Pin); // wait for pin to go low
        }
    }
    return data;
}
    






void main() {
    // Pin directions
    TRISC7 = 1; // UART RX input

    // Initialize LCD
    lcd_init();
    
    // Display message
    lcd_puts("Temperature Measuring");
    __delay_ms(2000);  // Wait for 2 seconds
    
    lcd_cmd(0x01);     // Clear display
}
