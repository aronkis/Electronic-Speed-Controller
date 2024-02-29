#pragma once

#define F_CPU 16000000UL

#include <stdint.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define RX_BUFFER_SIZE 128

void uart_init(uint32_t baud,uint8_t high_speed);
void uart_send_byte(const char c);
void uart_send_array(const char *c,uint16_t len);
void uart_send_string(const char *c);
uint16_t uart_read_count(void);
char uart_read(void);
void debug_print(unsigned int number, const char *text);

