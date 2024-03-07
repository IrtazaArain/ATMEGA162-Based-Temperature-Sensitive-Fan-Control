/*
 * FanControl.c
 *
 * Created: 7/11/2023 4:00:00 PM
 * Author : Irtaza
 */

#define F_CPU 1000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// LCD pin configuration
#define LCD_DATA_PORT PORTA
#define LCD_CONTROL_PORT PORTC
#define LCD_RS_PIN PC0
#define LCD_RW_PIN PC1
#define LCD_EN_PIN PC2

// Keypad pin configuration
#define KEY_PRT PORTD
#define KEY_PIN PIND
#define KEY_DDR DDRD


// Function prototypes
char KEYPAD_getKey();
void LCD_init();
void LCD_command(unsigned char cmd);
void LCD_data(unsigned char data);
void LCD_string(const char* str);
void delay_ms(uint16_t ms);
void motorSpeed(uint8_t speed);

char key1;
static char count = '0';

int main(void) {
	
	DDRB = 0b00000001;
	TCCR1B |= (1 << CS10) | (1 << CS12) | (1 << WGM10);
	// Initialize peripherals
	LCD_init();

	// Display "Welcome" message
	LCD_command(0x80); // Set cursor to first line
	LCD_string("Welcome");

	// Delay for a certain period
	delay_ms(2000); // Delay for 2 seconds

	// Clear the display
	LCD_command(0x01); // Clear display

	// Display "Fan Status: ON" message
	LCD_command(0x80); // Set cursor to first line
	LCD_string("Fan Status: ON");

	// Delay for a certain period
	delay_ms(2000); // Delay for 2 seconds

	// Clear the display
	LCD_command(0x01); // Clear display

	LCD_string("Press a key");

	KEY_DDR = 0x0F;
	KEY_PRT = 0xF0;
	char key;

	// Main program loop
	while (1) {
		KEY_PRT=0xF0;
		if (KEY_PIN!=0xF0){
			key=KEYPAD_getKey();
			count='0';
		}
		// Read keypad input
		// Process keypad input
		if (key == '1') {
			if (count == '0') {
				LCD_command(0x80); // Set cursor to first line
				LCD_string("Fan Status: Slow");
				TCNT1 = 0;
				count = '1';
			}
			motorSpeed(1);
		}
		else if (key == '2') {
			if (count == '0') {
				LCD_command(0x80); // Set cursor to first line
				LCD_string("Fan Status: Medium");
				TCNT1 = 0;
				count = '1';
			}
			motorSpeed(2);
		}
		else if (key =='3'){
			if (count == '0') {
				LCD_command(0x80);
				LCD_string("Fan Status: Fast");
				TCNT1 = 0;
				count = '1';
			}
			motorSpeed(3);
		}
		else if (key=='0'){
			PORTB=0;
			// Display "Fan Status: OFF" message
			if (count=='0'){
				LCD_command(0x80); // Set cursor to first line
				LCD_string("Fan Status: OFF");
				// Delay for a certain period
				delay_ms(2000); // Delay for 2 seconds
				// Clear the display
				LCD_command(0x01); // Clear display
				count='1';
			}
		}
	}

	return 0;
}

// Function to read a key from the keypad
char KEYPAD_getKey() {
	PORTD=0b11111011;
	if ((PIND & (1<<PIND4))==0){
		_delay_ms(125);
		return '1';
	}
	else if((PIND & (1<<PIND5))==0){
		_delay_ms(125);
		return '2';
	}
	else if((PIND & (1<<PIND6))==0){
		_delay_ms(125);
		return '3';
	}
	PORTD=0b11110111;
	if ((PIND & (1<<PIND5))==0){
		_delay_ms(125);
		return '0';
	}
}
// Function to initialize the LCD
void LCD_init() {
	
	// Configure LCD control pins as outputs
	DDRC |= (1 << LCD_RS_PIN) | (1 << LCD_RW_PIN) | (1 << LCD_EN_PIN);
	
	// Configure LCD data port as outputs
	DDRA = 0xFF;
	
	// Initialize LCD in 4-bit mode
	_delay_ms(15);               // Wait for LCD to power up
	LCD_command(0x02);           // Return Home
	LCD_command(0x28);           // 4-bit, 2-line, 5x8 font
	LCD_command(0x0C);           // Display ON, Cursor OFF
	LCD_command(0x06);           // Entry mode - Auto-increment cursor
	LCD_command(0x01);           // Clear display
	_delay_ms(2);                // Wait for Clear display command to complete
}

// Function to send a command to the LCD
void LCD_command(unsigned char cmd) {
	// Send upper nibble
	LCD_DATA_PORT = (LCD_DATA_PORT & 0x0F) | (cmd & 0xF0);
	LCD_CONTROL_PORT &= ~(1 << LCD_RS_PIN);  // RS = 0 for command
	LCD_CONTROL_PORT &= ~(1 << LCD_RW_PIN);  // RW = 0 for write
	LCD_CONTROL_PORT |= (1 << LCD_EN_PIN);   // EN = 1 to enable
	_delay_us(1);
	LCD_CONTROL_PORT &= ~(1 << LCD_EN_PIN);  // EN = 0 to latch data
	
	_delay_us(100);
	
	// Send lower nibble
	LCD_DATA_PORT = (LCD_DATA_PORT & 0x0F) | (cmd << 4);
	LCD_CONTROL_PORT |= (1 << LCD_EN_PIN);   // EN = 1 to enable
	_delay_us(1);
	LCD_CONTROL_PORT &= ~(1 << LCD_EN_PIN);  // EN = 0 to latch data
	
	_delay_ms(2);
}

// Function to send data to the LCD
void LCD_data(unsigned char data) {
	// Send upper nibble
	LCD_DATA_PORT = (LCD_DATA_PORT & 0x0F) | (data & 0xF0);
	LCD_CONTROL_PORT |= (1 << LCD_RS_PIN);  // RS = 1 for data
	LCD_CONTROL_PORT &= ~(1 << LCD_RW_PIN);  // RW = 0 for write
	LCD_CONTROL_PORT |= (1 << LCD_EN_PIN);   // EN = 1 to enable
	_delay_us(1);
	LCD_CONTROL_PORT &= ~(1 << LCD_EN_PIN);  // EN = 0 to latch data

	_delay_us(100);

	// Send lower nibble
	LCD_DATA_PORT = (LCD_DATA_PORT & 0x0F) | (data << 4);
	LCD_CONTROL_PORT |= (1 << LCD_EN_PIN);   // EN = 1 to enable
	_delay_us(1);
	LCD_CONTROL_PORT &= ~(1 << LCD_EN_PIN);  // EN = 0 to latch data

	_delay_ms(2);
}
// Function to send a string to the LCD
void LCD_string(const char* str) {
	while (*str) {
		LCD_data(*str++);
	}
}
// Function to delay for a given number of milliseconds
void delay_ms(uint16_t ms) {
	for (uint16_t i = 0; i < ms; i++) {
		_delay_ms(1);
	}
}
// Function to control the motor speed
void motorSpeed(uint8_t speed) {
	
	if (TCNT1==5){
		PORTB=0;
	}
	if(speed==1){
		if(TCNT1==20){
			PORTB=1;
			TCNT1=0;
		}
	}
	if (speed==2){
		if(TCNT1==10){
			PORTB=1;
			TCNT1=0;
		}
		
	}
	if (speed==3){
		if(TCNT1==5){
			PORTB=1;
			TCNT1=0;
		}
		
	}
	
}