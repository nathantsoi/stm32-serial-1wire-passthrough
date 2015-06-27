// From https://github.com/4712/BLHeliSuite/blob/master/Interfaces/Arduino1Wire/Source/Arduino1Wire_C/Arduino1Wire.c
/*
    This file is part of the BlHeliSuite interface solutions for AVR
    Copyright (C) 2014  by 4712

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

    File: Arduino1Wire.c
    Author: 4712
*/
/*
 * Arduino1Wire.c //Method I in C
 *
 * Rev 1: Initial realease
 * Rev 1.1: Set ow pull up at startup
*/

#include <avr/io.h>

//1-Wire Pin:
//Arduino Nano only -> PD3=D3
//MEGA boards -> PB2=D51=MOSI
//all other -> PB3=D11=MOSI

#if defined OW_PIND3
#define OW_DDR DDRD
#define OW_PORT PORTD
#define OW_PIN PIND
#define OW PIND3
#elif defined OW_PINB3
#define OW_DDR DDRB
#define OW_PORT PORTB
#define OW_PIN PINB
#define OW PINB3
#elif defined OW_PINB2
#define OW_DDR DDRB
#define OW_PORT PORTB
#define OW_PIN PINB
#define OW PINB2
#endif


#if (defined (__AVR_ATmega2560__) || defined (__AVR_ATmega1280__))
#define IN_DDR DDRE
#define IN_PORT PORTE
#define IN_PIN PINE

#define IN_RX PINE0
#define IN_TX PINE1
#else

#define IN_DDR DDRD    // Data direction register         -- 0 == in      1 == out
#define IN_PORT PORTD  // Port output register            -- 0 == low     1 == high
                       // In input mode, manages pull up  -- 0 == no pull 1 == pull up
#define IN_PIN PIND    // Port input register             -- 0 == low     1 == high

#define IN_RX PIND0    // receive from serial adapter
#define IN_TX PIND1    // transmist to serial adapter
#endif

#define SetOWout (OW_DDR|= (1<< OW))       // 1wire pin OUTPUT mode
#define SetOWin (OW_DDR &= ~(1<< OW))      // 1wire pin INPUT mode
#define SetOWlow (OW_PORT &= ~(1<< OW))    // output mode: 1wire pin LOW
                                           // input mode:  1wire pin NO PULL
#define SetOWhigh (OW_PORT |= (1<< OW))    // output mode: 1wire pin HI
                                           // input mode:  1wire pin PULL UP
#define IsOWhigh (OW_PIN & (1<< OW))       // 1 when 1wire pin is HI in input mode
#define IsOWlow (!(OW_PIN & (1<< OW)))     // 1 when 1wire pin is LO in input mode

#define SetTXlow (IN_PORT &= ~(1<< IN_TX)) // tx set pin LO
#define SetTXhigh (IN_PORT |= (1<< IN_TX)) // tx set pin HI
#define IsRXhigh (IN_PIN & (1<< IN_RX))    // 1 when rx pin is HI
#define IsRXlow (!(IN_PIN & (1<< IN_RX)))  // 1 when  rx pin is low

int main(void)
{

	//Very important disable hardware UART
	UCSR0B =0;
	IN_DDR |=  (1<<IN_TX ); // set TX pin to output mode, sending data to serial adapter on computer
	IN_DDR &= ~(0<< IN_RX); // set RX pin to input mode
	IN_PORT|= (1<<IN_TX )|(1<< IN_RX); //pullup RX ;TX high
	//SetOWin;//Set OW as input / default
	SetOWhigh; //pull up on

	while(IsRXhigh); //wait for RX go Low = 1 Incoming data
	while(1)
	{
		//RX low = 1 -> set oneWire to output
		SetOWout;
		SetOWlow; //Set oneWire low =1
		SetTXlow;  //Echo low to TX

		//Wait RX go high = 0 again
		while(IsRXlow);

		// input // RX is high (or high again) = 0 end of bit or idle - so listen to data from TRX
		SetOWhigh;  //Set OneWire High = 0 for sharper edge //can be skipped?
		//and Echo low to TX
		SetOWin;
		SetOWhigh;  //Set OneWire pull up

		while (IsRXhigh) //while RX high (no incoming bytes)
		{
			if  (IsOWhigh) SetTXhigh; //Set TX High = 0
			else SetTXlow; //Set TX Low = 1
		}
	}
}
