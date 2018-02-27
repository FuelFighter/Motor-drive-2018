/*
 * uart.c
 *
 * Created: 18/03/17 13:13:47
 *  Author: Sondre
 */ 

#include "uart.h"
#include <avr/io.h>
#include <stdint.h>

void uart_init()
{
	const uint32_t baud_rate = 9600;
	UBRR0 = (F_CPU / 16) / baud_rate - 1;
	UCSR0C = (3 << UCSZ0);	// 8 bit transfer
	UCSR0B |= (1 << TXEN0); // Enable transmit only
}

void uart_tx_char(char c)
{
	while (!(UCSR0A & (1 << UDRE0))) { }; // Wait until data register is empty
	UDR0 = c;
}

void uart_tx_str(char* str)
{
	for(char* ptr = str; *ptr; ptr++)
	uart_tx_char(*ptr);
}


/////////////////////////USART/////////////////////////

void USART0_Init (unsigned int baud)
{
/* Set baud rate */
UBRR0H = (unsigned char) (baud>>8);
UBRR0L = (unsigned char) baud;
/* Set frame format: 8data, no parity & 2 stop bits */
UCSR0C = (0<<UMSEL0) | (0<<UPM0) | (1<<USBS0) | (3<<UCSZ0);
/* Enable receiver and transmitter */
UCSR0B = (1<<RXEN0) | (1<<TXEN0);
}

void USART0_Transmit (unsigned int data)
{
/* Wait for empty transmit buffer */
while ( !( UCSR0A & (1<<UDRE0)));
/* Copy 9th bit to TXB8 */
UCSR0B &= ~(1<<TXB80);
if ( data & 0x0100 )
UCSR0B |= (1<<TXB80);
/* Put data into buffer, sends the data */
UDR0 = data;
}
