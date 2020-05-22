/*
 * ps2bs.h - a library to interface with ps2 devices. See comments in
 * ps2bs.cpp.
 * Written by Chris J. Kiick, January 2008.
 * adapted with digitalwritefast (optimized digital functions for AVR microcontrollers): BS 2019
 * Release into public domain.
 */

#ifndef ps2bs_h
#define ps2bs_h

#include "Arduino.h"

class PS2
{
	public:
		PS2(int clk, int data);
		void write(unsigned char data);
		unsigned char read(void);
	private:
		int _ps2clk;
		int _ps2data;
		void golo(int pin);
		void gohi(int pin);
};

#endif /* ps2bs_h */
