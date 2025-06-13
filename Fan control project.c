// LCD Initialization and Display Message
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
