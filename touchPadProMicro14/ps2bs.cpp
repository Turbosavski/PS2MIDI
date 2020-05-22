/*
 * ps2bs.cpp - an interface library for ps2 devices.  Common devices are
 * mice, keyboard, barcode scanners etc.  See the examples for mouse and
 * keyboard interfacing.
 * limitations:
 *      we do not handle parity errors.
 *      The timing constants are hard coded from the spec. Data rate is not impressive.
 *      probably lots of room for optimization.
 * BS: use digitalwritefast library
 */

#include "ps2bs.h"
#include "digitalWriteFast.h"

/*
 * the clock and data pins can be wired directly to the clk and data pins
 * of the PS2 connector.  No external parts are needed.
 */
PS2::PS2(int clk, int data)
{
	_ps2clk = clk;
	_ps2data = data;
	gohi(_ps2clk);
	gohi(_ps2data);
}

/*
 * according to some code I saw, these functions will
 * correctly set the clock and data pins for
 * various conditions.  It's done this way so you don't need
 * pullup resistors.
 */
void
PS2::gohi(int pin)
{
	pinModeFast(pin, INPUT);
	digitalWriteFast(pin, HIGH);// switch on pullup resistor
}

void
PS2::golo(int pin)
{
  digitalWriteFast(pin, LOW);
	pinModeFast(pin, OUTPUT);
}

/* write a byte to the PS2 device */
void
PS2::write(unsigned char data)
{
	unsigned char i;
	unsigned char parity = 1;
	
	// Set the clock and data pins high
	gohi(_ps2clk); // switch as input HIGH - pulled up
	gohi(_ps2data); // switch as input HIGH - pulled up
	delayMicroseconds(300); // or 150
	golo(_ps2clk); // switch as output LOW
	delayMicroseconds(300); // or 150
	golo(_ps2data); // switch as output LOW
	delayMicroseconds(10);
	gohi(_ps2clk);	// start bit  // switch as input HIGH
	
	/* wait for device to take control of clock */
	// this is blocking the system until pin senses low
	while (digitalReadFast(_ps2clk) == HIGH){;} // this loop intentionally left blank

	// clear to send data
	for (i=0; i < 8; i++)
	{
		if (data & 0x01)
		{
			gohi(_ps2data); // switch as input
		} else {
			golo(_ps2data); // switch as output
		}
		while (digitalReadFast(_ps2clk) == LOW){;}
		while (digitalReadFast(_ps2clk) == HIGH){;}
		parity = parity ^ (data & 0x01);
		data = data >> 1;
	}

	// parity bit
	if (parity)
	{
		gohi(_ps2data); // switch as input
	} else {
		golo(_ps2data);  // switch as output
	}
	// clock cycle - like ack.
	while (digitalReadFast(_ps2clk) == LOW){;}
	while (digitalReadFast(_ps2clk) == HIGH){;}
	// stop bit
	gohi(_ps2data); // switch as input
	delayMicroseconds(50);
	while (digitalReadFast(_ps2clk) == HIGH){;}
	// mode switch
	while ((digitalReadFast(_ps2clk) == LOW) || (digitalReadFast(_ps2data) == LOW)){;}
	// hold up incoming data
	golo(_ps2clk); // switch as output LOW
} 


/*
 * read a byte of data from the ps2 device.  Ignores parity.
 */
unsigned char
PS2::read(void)
{
	unsigned char data = 0x00;
	unsigned char i;
	unsigned char bit = 0x01;

	// start clock
	gohi(_ps2clk); // switch as input HIGH
	gohi(_ps2data); // switch as input HIGH
	//delayMicroseconds(50); // 150
	while (digitalReadFast(_ps2clk) == HIGH){;}
	//delayMicroseconds(5);	// not sure why.
	while (digitalReadFast(_ps2clk) == LOW){;} // eat start bit
	for (i=0; i < 8; i++)
	{
		while (digitalReadFast(_ps2clk) == HIGH){;}
		if (digitalReadFast(_ps2data) == HIGH)
		{
			data = data | bit;
		}
		while (digitalReadFast(_ps2clk) == LOW){;}
		bit = bit << 1;
	}

	// eat parity bit, ignore it.
	while (digitalReadFast(_ps2clk) == HIGH){;}
	while (digitalReadFast(_ps2clk) == LOW){;}
	// eat stop bit
	while (digitalReadFast(_ps2clk) == HIGH){;}
	while (digitalReadFast(_ps2clk) == LOW){;}
	golo(_ps2clk);	// hold incoming data -  // switch as output LOW
	delayMicroseconds(100); // BS
	
	return data;
}
