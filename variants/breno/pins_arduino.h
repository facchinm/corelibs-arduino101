/*
  pins_arduino.h - Pin definition functions for Arduino
  Part of Arduino - http://www.arduino.cc/

  Copyright (c) 2007 David A. Mellis

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

  $Id: wiring.h 249 2007-02-03 16:52:51Z mellis $
*/

#ifndef Pins_Arduino_h
#define Pins_Arduino_h

#define NUM_DIGITAL_PINS            32
#define NUM_ANALOG_INPUTS           7
#define NUM_PWM						4
#define NUM_UARTS					1
#define NUM_SPI						1
#define NUM_I2C						2

#define analogInputToDigitalPin(p)  ((p < 7) ? (p) + 15 : -1)

#define digitalPinHasPWM(p)         ((p) == 2 || (p) == 3 || (p) == 4 || (p) == 5)

static const uint8_t SS   = 6;
static const uint8_t MOSI = 8;
static const uint8_t MISO = 10;
static const uint8_t SCK  = 9;

static const uint8_t SDA = 11;
static const uint8_t SCL = 12;
static const uint8_t LED_BUILTIN = 25;

static const uint8_t A0 = 15;
static const uint8_t A1 = 16;
static const uint8_t A2 = 17;
static const uint8_t A3 = 18;
static const uint8_t A4 = 19;
static const uint8_t A5 = 20;
static const uint8_t A6 = 21;

#endif
