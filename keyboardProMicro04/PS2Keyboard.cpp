/*
  PS2Keyboard.cpp - PS2Keyboard library
  Copyright (c) 2007 Free Software Foundation.  All right reserved.
  Written by Christian Weichel <info@32leaves.net>

  ** Mostly rewritten Paul Stoffregen <paul@pjrc.com> 2010, 2011
  ** Modified for use beginning with Arduino 13 by L. Abraham Smith, <n3bah@microcompdesign.com> * 
  ** Modified for easy interrup pin assignement on method begin(datapin,irq_pin). Cuningan <cuninganreset@gmail.com> **

  for more information you can read the original wiki in arduino.cc
  at http://www.arduino.cc/playground/Main/PS2Keyboard
  or http://www.pjrc.com/teensy/td_libs_PS2Keyboard.html

  Version 2.4 (March 2013)
  - Support Teensy 3.0, Arduino Due, Arduino Leonardo & other boards
  - French keyboard layout, David Chochoi, tchoyyfr at yahoo dot fr

  Version 2.3 (October 2011)
  - Minor bugs fixed

  Version 2.2 (August 2011)
  - Support non-US keyboards - thanks to Rainer Bruch for a German keyboard :)

  Version 2.1 (May 2011)
  - timeout to recover from misaligned input
  - compatibility with Arduino "new-extension" branch
  - TODO: send function, proposed by Scott Penrose, scooterda at me dot com

  Version 2.0 (June 2010)
  - Buffering added, many scan codes can be captured without data loss
    if your sketch is busy doing other work
  - Shift keys supported, completely rewritten scan code to ascii
  - Slow linear search replaced with fast indexed table lookups
  - Support for Teensy, Arduino Mega, and Sanguino added

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "PS2Keyboard.h"

// BS
// Private function declarations
uint8_t PS2_led_lock = 0;     // LED and Lock status
int16_t send_next( void );
void set_lock( );
// end BS

#define BUFFER_SIZE 45
static volatile uint8_t buffer[BUFFER_SIZE];
static volatile uint8_t head, tail;
static uint8_t DataPin;
static uint8_t CharBuffer=0;
static bool CharBufferRelease=false;
static uint8_t UTF8next=0;
static const PS2Keymap_t *keymap=NULL;

// The ISR for the external interrupt
void ps2interrupt(void) {
  static uint8_t bitcount=0;
  static uint8_t incoming=0;
  static uint32_t prev_ms=0;
  uint32_t now_ms;
  uint8_t n, val;

  val = digitalRead(DataPin);
  now_ms = millis();
  if (now_ms - prev_ms > 250) {
    bitcount = 0;
    incoming = 0;
  }
  prev_ms = now_ms;
  n = bitcount - 1;
  if (n <= 7) {
    incoming |= (val << n);
  }
  bitcount++;
  if (bitcount == 11) {
    uint8_t i = head + 1;
    if (i >= BUFFER_SIZE) i = 0;
    if (i != tail) {
      buffer[i] = incoming;
      head = i;
    }
    bitcount = 0;
    incoming = 0;
  }
}

static inline uint8_t get_scan_code(void){
  uint8_t c, i;

  i = tail;
  if (i == head) return 0;
  i++;
  if (i >= BUFFER_SIZE) i = 0;
  c = buffer[i];

  tail = i;

  //Serial.print(" buffer ");
  //Serial.print(i);
  //Serial.print(" ");
  //Serial.println(c);

  // for each button push there is trio:
  //        i
  // buffer 0 51
  // buffer 1 240
  // buffer 2 51
  // repeating button:
  // buffer 3 51
  // buffer 4 240
  // buffer 5 51

  // if there is no 240 between numbers in buffer code 
  // and next number is the same as previous in the buffer
  // we should return nothing
  // this is against rafal of numbers / notes
  //if(buffer[i]!=240 && buffer[i-1]!=240) return 0;
  if(buffer[i]==buffer[i-1] && buffer[i-2]!=240) return 0;

  return c;
}

// http://www.quadibloc.com/comp/scan.htm
// http://www.computer-engineering.org/ps2keyboard/scancodes2.html

// These arrays provide a simple key map, to turn scan codes into ISO8859
// output.  If a non-US keyboard is used, these may need to be modified
// for the desired output.

// BS: some SI, DE letters are not defined in ISO8859! In such occasion return number
// MIDI keymap returns mostly numbers

// original
const PROGMEM PS2Keymap_t PS2Keymap_US = {
  // without shift
  {0, PS2_F9, 0, PS2_F5, PS2_F3, PS2_F1, PS2_F2, PS2_F12,
  0, PS2_F10, PS2_F8, PS2_F6, PS2_F4, PS2_TAB, '`', 0,
  0, 0 /*Lalt*/, 0 /*Lshift*/, 0, 0 /*Lctrl*/, 'q', '1', 0,
  0, 0, 'z', 's', 'a', 'w', '2', 0,
  0, 'c', 'x', 'd', 'e', '4', '3', 0,
  0, ' ', 'v', 'f', 't', 'r', '5', 0,
  0, 'n', 'b', 'h', 'g', 'y', '6', 0,
  0, 0, 'm', 'j', 'u', '7', '8', 0,
  0, ',', 'k', 'i', 'o', '0', '9', 0,
  0, '.', '/', 'l', ';', 'p', '-', 0,
  0, 0, '\'', 0, '[', '=', 0, 0,
  0 /*CapsLock*/, 0 /*Rshift*/, PS2_ENTER /*Enter*/, ']', 0, '\\', 0, 0,
  0, 0, 0, 0, 0, 0, PS2_BACKSPACE, 0,
  0, '1', 0, '4', '7', 0, 0, 0,
  '0', '.', '2', '5', '6', '8', PS2_ESC, 0 /*NumLock*/,
  PS2_F11, '+', '3', '-', '*', '9', PS2_SCROLL, 0,
  0, 0, 0, PS2_F7 },
  // with shift
  {0, PS2_F9, 0, PS2_F5, PS2_F3, PS2_F1, PS2_F2, PS2_F12,
  0, PS2_F10, PS2_F8, PS2_F6, PS2_F4, PS2_TAB, '~', 0,
  0, 0 /*Lalt*/, 0 /*Lshift*/, 0, 0 /*Lctrl*/, 'Q', '!', 0,
  0, 0, 'Z', 'S', 'A', 'W', '@', 0,
  0, 'C', 'X', 'D', 'E', '$', '#', 0,
  0, ' ', 'V', 'F', 'T', 'R', '%', 0,
  0, 'N', 'B', 'H', 'G', 'Y', '^', 0,
  0, 0, 'M', 'J', 'U', '&', '*', 0,
  0, '<', 'K', 'I', 'O', ')', '(', 0,
  0, '>', '?', 'L', ':', 'P', '_', 0,
  0, 0, '"', 0, '{', '+', 0, 0,
  0 /*CapsLock*/, 0 /*Rshift*/, PS2_ENTER /*Enter*/, '}', 0, '|', 0, 0,
  0, 0, 0, 0, 0, 0, PS2_BACKSPACE, 0,
  0, '1', 0, '4', '7', 0, 0, 0,
  '0', '.', '2', '5', '6', '8', PS2_ESC, 0 /*NumLock*/,
  PS2_F11, '+', '3', '-', '*', '9', PS2_SCROLL, 0,
  0, 0, 0, PS2_F7 },
  0
};

// below: C12=12, C2=24, C3=36, C4=48 , C5=60
// our reference is 12 - do not go below
// int is number, char is keyword
const PROGMEM PS2Keymap_t PS2Keymap_MIDI = {
  // without shift
  {0, PS2_F9, 0, PS2_F5, PS2_F3, PS2_F1, PS2_F2, PS2_F12,
  0, PS2_F10, PS2_F8, PS2_F6, PS2_F4, PS2_TAB, '`', 0,
  0, 0, 0, 0, 0, 0, '1', 0,
  0, 0, M_TRANSPOSE_M, 14, 12, 13, '2', 0,
  0, M_VOLUME_M, M_TRANSPOSE_P, 16, 15, '4', '3', 0,
  0, ' ', M_VOLUME_P, 17, 18, 0, '5', 0,
  0, 'n', 'b', 21, 19, 20, '6', 0,
  0, 0, 'm', 23, 22, '7', '8', 0,
  0, ',', 24, 0, 25, '0', '9', 0,
  0, '.', '/', 26, ';', 27, '-', 0,
  0, 0, '\'', 0, '[', '=', 0, 0,
  0, 0, PS2_ENTER, ']', 0, '\\', 0, 0,
  0, 0, 0, 0, 0, 0, PS2_BACKSPACE, 0,
  0, '1', 0, '4', '7', 0, 0, 0,
  '0', '.', '2', '5', '6', '8', PS2_ESC, 0,
  PS2_F11, '+', '3', '-', '*', '9', PS2_SCROLL, 0,
  0, 0, 0, PS2_F7 },
  // with shift
  {0, PS2_F9, 0, PS2_F5, PS2_F3, PS2_F1, PS2_F2, PS2_F12,
  0, PS2_F10, PS2_F8, PS2_F6, PS2_F4, PS2_TAB, '~', 0,
  0, 0, 0, 0, 0, 'Q', '!', 0,
  0, 0, 'Z', 'S', 'A', 'W', '@', 0,
  0, 'C', 'X', 'D', 'E', '$', '#', 0,
  0, ' ', 'V', 'F', 'T', 'R', '%', 0,
  0, 'N', 'B', 'H', 'G', 'Y', '^', 0,
  0, 0, 'M', 'J', 'U', '&', '*', 0,
  0, '<', 'K', 'I', 'O', ')', '(', 0,
  0, '>', '?', 'L', ':', 'P', '_', 0,
  0, 0, '"', 0, '{', '+', 0, 0,
  0, 0, PS2_ENTER, '}', 0, '|', 0, 0,
  0, 0, 0, 0, 0, 0, PS2_BACKSPACE, 0,
  0, '1', 0, '4', '7', 0, 0, 0,
  '0', '.', '2', '5', '6', '8', PS2_ESC, 0,
  PS2_F11, '+', '3', '-', '*', '9', PS2_SCROLL, 0,
  0, 0, 0, PS2_F7 },
  0
};


#define BREAK     0x01
#define MODIFIER  0x02
#define SHIFT_L   0x04
#define SHIFT_R   0x08
#define ALTGR     0x10

static char get_iso8859_code(bool &iskeyrelease, bool detectkeyreleases){
  static uint8_t state=0;
  uint8_t s;
  char c; 
  iskeyrelease = false;
  while (1) {
    s = get_scan_code();
    if (!s) return 0;
   
    if (s == 0xF0) {
      state |= BREAK;
    } else if (s == 0xE0) {
      state |= MODIFIER;
    } else {
      if (state & BREAK) {
        if (s == 0x12) {
          state &= ~SHIFT_L;
        } else if (s == 0x59) {
          state &= ~SHIFT_R;
        } else if (s == 0x11 && (state & MODIFIER)) {
          state &= ~ALTGR;
        } else {
          iskeyrelease = true;
        }
        // CTRL, ALT & WIN keys could be added
        // but is that really worth the overhead?
        state &= ~(BREAK | MODIFIER);
        if(!iskeyrelease || !detectkeyreleases) {
          continue;        
        }
      }
      if (s == 0x12) {
        state |= SHIFT_L;
        continue;
      } else if (s == 0x59) {
        state |= SHIFT_R;
        continue;
      } else if (s == 0x11 && (state & MODIFIER)) {
        state |= ALTGR;
      }
      
      c = 0;
      if (state & MODIFIER) {
        switch (s) {
          case 0x70: c = PS2_INSERT;      break;
          case 0x6C: c = PS2_HOME;        break;
          case 0x7D: c = PS2_PAGEUP;      break;
          case 0x71: c = PS2_DELETE;      break;
          case 0x69: c = PS2_END;         break;
          case 0x7A: c = PS2_PAGEDOWN;    break;
          case 0x75: c = PS2_UPARROW;     break;
          case 0x6B: c = PS2_LEFTARROW;   break;
          case 0x72: c = PS2_DOWNARROW;   break;
          case 0x74: c = PS2_RIGHTARROW;  break;
          case 0x4A: c = '/';             break;
          case 0x5A: c = PS2_ENTER;       break;
          default: break;
        }
        
      } else if ((state & ALTGR) && keymap->uses_altgr) {
        if (s < PS2_KEYMAP_SIZE)
          c = pgm_read_byte(keymap->altgr + s);
      } else if (state & (SHIFT_L | SHIFT_R)) {
        if (s < PS2_KEYMAP_SIZE)
          c = pgm_read_byte(keymap->shift + s);
      } else {
        if (s < PS2_KEYMAP_SIZE)
          c = pgm_read_byte(keymap->noshift + s);
      }
      state &= ~(BREAK | MODIFIER);

      // BS: from PS2KeyAdvanced lib
      // Now update PS2_led_lock status to match
      if( PS2_led_lock & s ) {
          PS2_led_lock &= ~s;
          //PS2_keystatus |= _BREAK;     // send as break
      } else
          PS2_led_lock |= s;
      set_lock( );
      // end BS
      
      if (c) return c;
    }
  }
}

bool PS2Keyboard::availablerel() {   
  if (CharBuffer || UTF8next) return true;  
  CharBuffer = get_iso8859_code(CharBufferRelease,true);
  if (CharBuffer) return true;
  return false;
}

bool PS2Keyboard::available() {  // wrapper to maintain backwards compatibility
  if (CharBuffer || UTF8next) return true;  
  CharBuffer = get_iso8859_code(CharBufferRelease,false);
  if (CharBuffer) return true;
  return false;
}

int PS2Keyboard::readrel_internal(bool &iskeyrelease, bool detectkeyreleases) { 
  uint8_t result;
  result = UTF8next;
  if (result) {
    UTF8next = 0;
  } else {
    result = CharBuffer;
    iskeyrelease = CharBufferRelease;
    if (result) {
      CharBuffer = 0;
      CharBufferRelease = false;
    } else {
      // we don't get here
      result = get_iso8859_code(iskeyrelease,true);
    }
    if (result >= 128) {
      UTF8next = (result & 0x3F) | 0x80;
      result = ((result >> 6) & 0x1F) | 0xC0;
    }
  }

  if (!result) return -1;
  return result;
}

int PS2Keyboard::readrel(bool &iskeyrelease) { 
  uint8_t result;
  result = readrel_internal(iskeyrelease, true);
  // this returns a number - or two numbers
  //Serial.println(result);
  return result;
}

// always prints twice
int PS2Keyboard::read() {  // wrapper to maintain backwards compatibility
  uint8_t result;
  bool iskeyrelease;
  result = readrel_internal(iskeyrelease, false);
  // this returns a number - or two numbers
  //Serial.println(result);
  return result;
}

PS2Keyboard::PS2Keyboard() {
  // nothing to do here, begin() does it all
}

void PS2Keyboard::begin(uint8_t data_pin, uint8_t irq_pin, const PS2Keymap_t &map) {
  
  uint8_t irq_num=255;

  DataPin = data_pin;
  keymap = &map;

  // initialize the pins
  #ifdef INPUT_PULLUP
    pinMode(irq_pin, INPUT_PULLUP);
    pinMode(data_pin, INPUT_PULLUP);
  #else
    pinMode(irq_pin, INPUT);
    digitalWrite(irq_pin, HIGH);
    pinMode(data_pin, INPUT);
    digitalWrite(data_pin, HIGH);
  #endif
  
  #ifdef CORE_INT_EVERY_PIN
    irq_num = irq_pin;
  
  #else
    switch(irq_pin) {
      #ifdef CORE_INT0_PIN
      case CORE_INT0_PIN:
        irq_num = 0;
        break;
      #endif
      #ifdef CORE_INT1_PIN
      case CORE_INT1_PIN:
        irq_num = 1;
        break;
      #endif
      #ifdef CORE_INT2_PIN
      case CORE_INT2_PIN:
        irq_num = 2;
        break;
      #endif
      #ifdef CORE_INT3_PIN
      case CORE_INT3_PIN:
        irq_num = 3;
        break;
      #endif
      #ifdef CORE_INT4_PIN
      case CORE_INT4_PIN:
        irq_num = 4;
        break;
      #endif
      #ifdef CORE_INT5_PIN
      case CORE_INT5_PIN:
        irq_num = 5;
        break;
      #endif
      #ifdef CORE_INT6_PIN
      case CORE_INT6_PIN:
        irq_num = 6;
        break;
      #endif
      #ifdef CORE_INT7_PIN
      case CORE_INT7_PIN:
        irq_num = 7;
        break;
      #endif
    }
  #endif

  head = 0;
  tail = 0;
  if (irq_num < 255) {
    attachInterrupt(irq_num, ps2interrupt, FALLING);
  }
}

/* From PS2KeyAdvanced: */

/*  Send a byte to the TX buffer
    Value in buffer of PS2_KEY_IGNORE signifies wait for response,
    use one for each byte expected

    Returns -4 - if buffer full (buffer overrun not written)
    Returns 1 byte written when done 
*/
int send_byte( uint8_t val ) {
  uint8_t ret;

  ret = head + 1;
  if( ret >= BUFFER_SIZE )
    ret = 0;
  if( ret != tail ) {
    buffer[ ret ] = val;
    head = ret;
    return 1;
  }
  return -4;
}


/* Build command to send lock status
    Assumes data is within range */
void set_lock( ){
  send_byte( PS2_KC_LOCK );        // send command
  send_byte( PS2_KEY_IGNORE );     // wait ACK
  send_byte( PS2_led_lock );       // send data from internal variable
  //if( ( send_byte( PS2_KEY_IGNORE ) ) ) // wait ACK
  //  send_next();              // if idle start transmission
}

/* Returns the current status of Locks */
uint8_t PS2Keyboard::getLock(){ 
  return( PS2_led_lock );
}

/* Sets the current status of Locks and LEDs */
void PS2Keyboard::setLock( uint8_t code ){
  code &= 0xF;                // To allow for rare keyboards with extra LED
  PS2_led_lock = code;        // update our lock copy
  //PS2_keystatus &= ~_CAPS;    // Update copy of _CAPS lock as well
  //PS2_keystatus |= ( code & PS2_LOCK_CAPS ) ? _CAPS : 0;
  set_lock( );
}
