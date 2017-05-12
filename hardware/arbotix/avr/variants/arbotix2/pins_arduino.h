/*
  pins_arduino.h - Pin definition functions for ArbotiX2
  Part of Arduino - http://www.arduino.cc/

  Copyright (c) 2012 Michael Ferguson

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General
  Public License along with this library; if not, write to the
  Free Software Foundation, Inc., 59 Temple Place, Suite 330,
  Boston, MA  02111-1307  USA
*/

#ifndef Pins_Arduino_h
#define Pins_Arduino_h

#define ARBOTIX2

#include <avr/pgmspace.h>

#define NUM_DIGITAL_PINS            32
#define NUM_ANALOG_INPUTS           8
#define analogInputToDigitalPin(p)  ((p < 8) ? 24 + (p) : -1)

#define digitalPinHasPWM(p)         ((p) == 3 || (p) == 4 || (p) == 12 || (p) == 13 || (p) == 14 || (p) == 15)

static const uint8_t SS   = 4;
static const uint8_t MOSI = 5;
static const uint8_t MISO = 6;
static const uint8_t SCK  = 7;

static const uint8_t SDA = 17;
static const uint8_t SCL = 16;
static const uint8_t LED_BUILTIN = 0;

static const uint8_t A0 = 24;
static const uint8_t A1 = 25;
static const uint8_t A2 = 26;
static const uint8_t A3 = 27;
static const uint8_t A4 = 28;
static const uint8_t A5 = 29;
static const uint8_t A6 = 30;
static const uint8_t A7 = 31;

#define digitalPinToPCICR(p)    (((p) >= 0 && (p) <= 31) ? (&PCICR) : ((uint8_t *)0))
#define digitalPinToPCICRbit(p) (((p) <= 7) ? 1 : \
                                (((p) <= 15) ? 3 : \
                                (((p) <= 23) ? 2 : \
                                  0)))
#define digitalPinToPCMSK(p)    (((p) <= 7) ? (&PCMSK1) : \
                                (((p) <= 15) ? (&PCMSK3) : \
                                (((p) <= 23) ? (&PCMSK2) : \
                                (((p) <= 31) ? (&PCMSK0) : ((uint8_t *)0)))))
#define digitalPinToPCMSKbit(p) (((p) <= 7) ? (p) : \
                                (((p) <= 15) ? ((p) - 8) : \
                                (((p) <= 23) ? ((p) - 16) : \
                                 ((p) - 24))))

#ifdef ARDUINO_MAIN

// On the Arduino board, digital pins are also used
// for the analog output (software PWM).  Analog input
// pins are a separate set.

// ATMEL ATMEGA644P
//
//                   +---\/---+
//  INT0 (D 0) PB0  1|        |40  PA0 (AI 0 / D24)
//  INT1 (D 1) PB1  2|        |39  PA1 (AI 1 / D25)
//  INT2 (D 2) PB2  3|        |38  PA2 (AI 2 / D26)
//   PWM (D 3) PB3  4|        |37  PA3 (AI 3 / D27)
//   PWM (D 4) PB4  5|        |36  PA4 (AI 4 / D28)
//  MOSI (D 5) PB5  6|        |35  PA5 (AI 5 / D29)
//  MISO (D 6) PB6  7|        |34  PA6 (AI 6 / D30)
//   SCK (D 7) PB7  8|        |33  PA7 (AI 7 / D31)
//             RST  9|        |32  AREF
//             VCC 10|        |31  GND 
//             GND 11|        |30  AVCC
//           XTAL2 12|        |29  PC7 (D 23)
//           XTAL1 13|        |28  PC6 (D 22)
//  RX0 (D 8)  PD0 14|        |27  PC5 (D 21) TDI
//  TX0 (D 9)  PD1 15|        |26  PC4 (D 20) TDO
//  RX1 (D 10) PD2 16|        |25  PC3 (D 19) TMS
//  TX1 (D 11) PD3 17|        |24  PC2 (D 18) TCK
//  PWM (D 12) PD4 18|        |23  PC1 (D 17) SDA
//  PWM (D 13) PD5 19|        |22  PC0 (D 16) SCL
//  PWM (D 14) PD6 20|        |21  PD7 (D 15) PWM
//                   +--------+
//

// these arrays map port names (e.g. port B) to the
// appropriate addresses for various functions (e.g. reading
// and writing)
const uint16_t PROGMEM port_to_mode_PGM[] = {
	NOT_A_PORT,
	(uint16_t) &DDRA,
	(uint16_t) &DDRB,
	(uint16_t) &DDRC,
	(uint16_t) &DDRD,
};

const uint16_t PROGMEM port_to_output_PGM[] = {
	NOT_A_PORT,
	(uint16_t) &PORTA,
	(uint16_t) &PORTB,
	(uint16_t) &PORTC,
	(uint16_t) &PORTD,
};

const uint16_t PROGMEM port_to_input_PGM[] = {
	NOT_A_PORT,
	(uint16_t) &PINA,
	(uint16_t) &PINB,
	(uint16_t) &PINC,
	(uint16_t) &PIND,
};

const uint8_t PROGMEM digital_pin_to_port_PGM[] = {
	PB, /* 0 */
	PB,
	PB,
	PB,
	PB,
	PB,
	PB,
	PB,
	PD, /* 8 */
	PD,
	PD,
	PD,
	PD,
	PD,
	PD,
	PD,
	PC, /* 16 */
	PC,
	PC,
	PC,
	PC,
	PC,
   	PC,
	PC,
	PA, /* 24 */
	PA,
	PA,
	PA,
	PA,
	PA,
	PA,
	PA  /* 31 */
};

const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[] = {
	_BV(0), /* 0, port B */
	_BV(1),
	_BV(2),
	_BV(3),
	_BV(4),
	_BV(5),
	_BV(6),
	_BV(7),
	_BV(0), /* 8, port D */
	_BV(1),
	_BV(2),
	_BV(3),
	_BV(4),
	_BV(5),
	_BV(6),
	_BV(7),
	_BV(0), /* 16, port C */
	_BV(1),
	_BV(2),
	_BV(3),
	_BV(4),
	_BV(5),
	_BV(6),
	_BV(7),
	_BV(0), /* 24, port A */
	_BV(1),
	_BV(2),
	_BV(3),
	_BV(4),
	_BV(5),
	_BV(6),
	_BV(7)
};

const uint8_t PROGMEM digital_pin_to_timer_PGM[] = {
	NOT_ON_TIMER, 	/* 0  - PB0 */
	NOT_ON_TIMER, 	/* 1  - PB1 */
	NOT_ON_TIMER, 	/* 2  - PB2 */
	TIMER0A,     	/* 3  - PB3 */
	TIMER0B, 		/* 4  - PB4 */
	NOT_ON_TIMER, 	/* 5  - PB5 */
	NOT_ON_TIMER, 	/* 6  - PB6 */
	NOT_ON_TIMER,	/* 7  - PB7 */
	NOT_ON_TIMER, 	/* 8  - PD0 */
	NOT_ON_TIMER, 	/* 9  - PD1 */
	NOT_ON_TIMER, 	/* 10 - PD2 */
	NOT_ON_TIMER, 	/* 11 - PD3 */
	TIMER1B,     	/* 12 - PD4 */
	TIMER1A,     	/* 13 - PD5 */
	TIMER2B,     	/* 14 - PD6 */
	TIMER2A,     	/* 15 - PD7 */
	NOT_ON_TIMER, 	/* 16 - PC0 */
	NOT_ON_TIMER,   /* 17 - PC1 */
	NOT_ON_TIMER,   /* 18 - PC2 */
	NOT_ON_TIMER,   /* 19 - PC3 */
	NOT_ON_TIMER,   /* 20 - PC4 */
	NOT_ON_TIMER,   /* 21 - PC5 */
	NOT_ON_TIMER,   /* 22 - PC6 */
	NOT_ON_TIMER,   /* 23 - PC7 */
	NOT_ON_TIMER,   /* 24 - PA0 */
	NOT_ON_TIMER,   /* 25 - PA1 */
	NOT_ON_TIMER,   /* 26 - PA2 */
	NOT_ON_TIMER,   /* 27 - PA3 */
	NOT_ON_TIMER,   /* 28 - PA4 */
	NOT_ON_TIMER,   /* 29 - PA5 */
	NOT_ON_TIMER,   /* 30 - PA6 */
	NOT_ON_TIMER    /* 31 - PA7 */
};

#endif

#endif
