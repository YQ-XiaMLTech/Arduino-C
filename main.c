// Define LCD pins
#define RS eS_PORTB4
#define EN eS_PORTB5
#define D0 eS_PORTB3
#define D1 eS_PORTD3
#define D2 eS_PORTD4
#define D3 eS_PORTD5
#define D4 eS_PORTD6
#define D5 eS_PORTD7
#define D6 eS_PORTB0  
#define D7 eS_PORTB1

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "lcd.h"
#define F_CPU 16000000UL  // Define the clock frequency as 16MHz
#define BAUDRATE 9600        // Define baud rate 9600
#define BAUD_PRESCALLER (((F_CPU / (BAUDRATE * 16UL))) - 1)
#define Servo_PIN PB2
#define INTERRUPT_PIN PD2

// Initialize USART for communication
void USART_init(void){
	UBRR0H = (uint8_t)(BAUD_PRESCALLER>>8); // Set baud rate
	UBRR0L = (uint8_t)(BAUD_PRESCALLER);
	UCSR0B = (1<<RXEN0)|(1<<TXEN0); // Enable transmission
	UCSR0C = (3<<UCSZ00); // Set to 8-bit data format (default)
}

// Function to send data through USART
void USART_send(unsigned char data){
	while(!(UCSR0A & (1<<UDRE0))); // Wait until the buffer is empty
	UDR0 = data;
}

/*unsigned char USART_receive(void) {
	while(!(UCSR0A & (1<<RXC0))); // Wait for data to be received
	return UDR0; // Fetch the received byte from the USART data register
}*/

void USART_putstr(const char *str) {
	while(*str != '\0') { // Loop through the string until null terminator
		USART_send(*str); // Send each character
		str++; // Move to the next character
	}
}

// Initialize ADC (Analog to Digital Converter)
void ADC_Init() {
	// REFS0 = 1 sets the ADC reference voltage to AVcc
	ADMUX = (1 << REFS0); 
	// ADPS2:0 = 111 sets ADC prescaler to 128
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

// parameter 'channel' is the ADC channel number to read from
uint16_t Read_ADC(uint8_t channel) {
	// Configure ADMUX to select the ADC channel
	ADMUX = (ADMUX & 0xF8) | (channel & 0x05);

	ADCSRA |= (1 << ADSC);
	// ADSC bit will be cleared automatically once conversion is complete
	while (ADCSRA & (1 << ADSC));
	// Return the ADC conversion result
	return ADC;
}

// Initialize the servo motor
void initServo() {
	DDRB |= (1 << Servo_PIN); 

	// Set Timer 1 for Fast PWM mode, non-inverted
	TCCR1A |= (1 << WGM11) | (1 << COM1A1)|(1<<COM1B1);
	TCCR1B |= (1 << WGM13) | (1 << WGM12)| (1<<CS11);

	ICR1 = 40000; // Set the TOP value to 4999 for a 20ms PWM period

	// Set prescaler to 64
	//TCCR1B |= (1 << CS11) | (1 << CS10);

	OCR1B = 2000; // Initialize to 0-degree position (1ms pulse width)
}
ISR(INT0_vect){
	//Lcd8_Init();
	//Turn the servo to 90 degrees
	OCR1B = 4000;
	Lcd8_Clear();
	char* string = "Check mode";//lcd displays manual check mode
	Lcd8_Write_String(string);
	_delay_ms(3000);//Delay 3 seconds
}



int main() {
	USART_init();// Initialize USART for serial communication
	ADC_Init();// Initialize ADC for analog to digital conversion
	initServo();// Initialize the servo motor
	
	// Buffer arrays for serial and LCD communication
	char buffer[40];
	char lcd_write[40];
	// Set PORTD and PORTB as output
	DDRD = 0xFF;
	DDRB = 0xFF;
	
	// Set the interrupt pin as input and enable internal pull-up resistor
	DDRD &= ~(1 << INTERRUPT_PIN); // Set PD2 as input
	PORTD |= (1 << INTERRUPT_PIN); // Enable internal pull-up resistor on PD2

	// Enable External Interrupt Request 0
	EIMSK |= (1 << INT0); // Enable external interrupt 0
	// Set interrupt trigger mode (here set to trigger on any logical change)
	EICRA |= (1 << ISC00);// Any logical change on INT0 generates an interrupt request
	EICRA &= ~(1 << ISC01);// Clear ISC01 to configure for any logical change

	// Enable global interrupts
	sei();
	
	// Initialize 8-bit LCD
	Lcd8_Init();

	while (1) {
		// Read the temperature from ADC A0
		uint16_t tempReading = Read_ADC(0); 
		uint16_t voltage = (uint32_t)tempReading * 5000 / 1024; // Convert ADC reading to voltage in millivolts
		int16_t temperatureC = (voltage - 500) / 10;// Convert voltage to temperature in Celsius
		 // Format the temperature data into a string
		snprintf(buffer, sizeof(buffer), "Temperature: %d C\r\n", temperatureC);

		// Send formatted string over USART
		for (int i = 0; buffer[i] != '\0'; i++) {
			USART_send(buffer[i]);
		}
		
		 // Control logic based on temperature
		if (temperatureC>=25)
		{
			// If temperature is high, display alarm message and adjust servo
			snprintf(lcd_write,sizeof(lcd_write),"%d:high alarm",temperatureC);
			OCR1B=3000;
			
		}
		else
		{
			// If temperature is low, display message and adjust servo
			snprintf(lcd_write,sizeof(lcd_write),"%d:low temp",temperatureC);
			OCR1B=2000;
		}
		
		// Display the message on LCD
		Lcd8_Set_Cursor(1,1);
		Lcd8_Write_String(lcd_write);
		// Check if a character is received via USART
		if (UCSR0A & (1<<RXC0))
		{
			unsigned char ReceivedChar = UDR0;
			// If 'c' is received, close incubator)
			if(ReceivedChar == 'c'){
				OCR1B=2000;
				Lcd8_Set_Cursor(1,1);
				Lcd8_Write_String("input:close");
				USART_putstr("close incubator\n");
				_delay_ms(5000);
			}
			// If 'o' is received, open something incubator)
			else if(ReceivedChar == 'o'){
				OCR1B=4000;
				Lcd8_Set_Cursor(1,1);
				Lcd8_Write_String("input:open");
				USART_putstr("open incubator\n");
				_delay_ms(5000);
			}
		} 
		
		_delay_ms(1000); // Wait for 1 second
	}
}