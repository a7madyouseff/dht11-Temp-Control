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
// Globals
char mode = 0; // 0 = AUTO, 1 = MANUAL
char fan_state = 0;  // 0 = OFF, 1 = ON

// Update LCD with temperature, humidity and mode
void update_lcd() {
    lcd_cmd(0x80);
    char buffer[16];
    if (RH_int == 0xFF && Temp_int == 0xFF) {
        lcd_puts("Sensor Error   ");
    } else {
        sprintf(buffer, "T:%2dC H:%2d%%", Temp_int, RH_int);
        lcd_puts(buffer);
    }

    lcd_cmd(0xC0);
    lcd_puts("Mode: ");
    if (mode == 0) lcd_puts("AUTO   ");
    else          lcd_puts("MANUAL ");
    
}
void update_rgb_leds() {
    if (Temp_int <= 26) {
        GREEN_LED = 1;
        RED_LED = 0;
    } else {
        GREEN_LED = 0;
        RED_LED = 1;
    }
    BLUE_LED = 0;  // off 
}

void main() {
    // Pin directions
    TRISC7 = 1; // UART RX input
    TRISC0 = 0;   // FAN output
    TRISD3 = 0;   // RED LED output
    TRISD4 = 0;   // GREEN LED output
    TRISD6 = 0;   // BLUE LED output

    FAN = 0;
    RED_LED = 0;
    GREEN_LED = 0;
    BLUE_LED = 0;


    // Initialize LCD
    lcd_init();
    UART_Init();// Initialize UART
    // Display message
    lcd_puts("Temperature Measuring");
    __delay_ms(2000);  // Wait for 2 seconds
    
    lcd_cmd(0x01);     // Clear display
    while(1) {
          // UART Command Processing with error clearing
        if (RCIF) {
            // Clear UART errors
            if (OERR) {
                CREN = 0;
                CREN = 1;
            }
            if (FERR) {
                char dummy = RCREG;  // Clear framing error by reading buffer
            }

            char command = UART_Read();

            if (command == 'a' || command == 'A') {
                mode = 0;
                UART_Write_Text("Switched to AUTO\r\n");
            }
            else if (command == 'm' || command == 'M') {
                mode = 1;
                UART_Write_Text("Switched to MANUAL\r\n");
            }
            else if (command == '1') {
                if (mode == 1) {
                    if (!fan_state) {
                        FAN = 1;
                        fan_state = 1;
                        UART_Write_Text("Fan is ON\r\n");
                    }
                } else {
                    UART_Write_Text("Cannot control fan manually in AUTO mode\r\n");
                }
            }
            else if (command == '0') {
                if (mode == 1) {
                    if (fan_state) {
                        FAN = 0;
                        fan_state = 0;
                        UART_Write_Text("Fan is OFF\r\n");
                    }
                } else {
                    UART_Write_Text("Cannot control fan manually in AUTO mode\r\n");
                }
            }
            else {
                UART_Write_Text("Unknown command\r\n");
            }
        }
          // DHT11 Sensor Reading
        dht11_start();
        dht11_response();
        if (Check == 1) {
            RH_int = dht11_read_byte();
            RH_dec = dht11_read_byte();
            Temp_int = dht11_read_byte();
            Temp_dec = dht11_read_byte();
            checksum = dht11_read_byte();

            // Simple checksum validation
            if (checksum == ((RH_int + RH_dec + Temp_int + Temp_dec) & 0xFF)) {
                // Auto mode fan control with terminal message on change
                if (mode == 0) {
                    char new_fan_state = (Temp_int > 26) ? 1 : 0;
                    if (new_fan_state != fan_state) {
                        fan_state = new_fan_state;
                        FAN = fan_state;
                        if (fan_state)
                            UART_Write_Text("Fan is ON\r\n");
                        else
                            UART_Write_Text("Fan is OFF\r\n");
                    }
                }
            update_rgb_leds();
                update_lcd();
            } else {
                UART_Write_Text("Checksum error\r\n");
                // Set error values so LCD shows sensor error
                RH_int = 0xFF;
                Temp_int = 0xFF;
                update_lcd();
            }
        } else {
            UART_Write_Text("No response from sensor\r\n");
            RH_int = 0xFF;
            Temp_int = 0xFF;
            update_lcd();
        }

        __delay_ms(1500);
    }
}
