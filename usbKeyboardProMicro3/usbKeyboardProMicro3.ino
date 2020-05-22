/* Keyboard matrix, PSU (I2C) and touchpad PS2 - to usb controller
 * Sparkfun Arduino Pro Micro 16MHz 5V
 * 
 * // pins used - not used: 5 
   // pins 2 and 3: SDA and SCL - to eeprom
   #define WP_PIN 4 // eeprom WP pin always low to enable writing

 //Make sure you have data and clock wires connected correctly. 
 //Clock wire MUST be connected to an interrupt pin!

 * touch2usb
 * reads ps/2 data stream and outputs it to usb 
 The ps/2 code uses the USB PJRC Mouse functions at www.pjrc.com/teensy/td_mouse.html 
 The ps/2 code has a watchdog timer so the code can't hang if a clock edge is missed.
 In the Arduino IDE, select Tools, Teensy 3.2. Also under Tools, select Keyboard+Mouse+Joystick
 * Copyright 2019 Frank Adams
   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at
       http://www.apache.org/licenses/LICENSE-2.0
   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
 // This code has been tested on the following touchpads:
//
// Dell D630 Touchpad. I used the wires from the touchpad connector as follows:
// Pin 2 = 5V
// Pin 1 = Gnd
// Pin 7 = Clock
// Pin 6 = Data
// This touchpad has resistive pullups so no additional pullups were required. 
//
// HP Pavilion DV9000 Touchpad part number 920-000702-04 Rev A
// The test points on the touchpad were wired as follows:
// T22 = 3.3V  (The touchpad also works with 5V)
// T23 = Gnd   (I soldered to the ground plane)
// T10 = Clock 
// T11 = Data 
// This touchpad has active pullups so no additional pullups were required.
// 
// Dell 1545 touchpad. I soldered wires to the 4 pin connector as follows:
// Pin 1 = 5V
// Pin 4 = Gnd
// Pin 3 = Clock
// Pin 2 = Data
// This touchpad has active pullups so no additional pullups were required. 

**/


/**
 * These core libraries allow the 32u4 and SAMD based boards (Leonardo, Esplora, Zero, Due and MKR Family) to appear as 
 * a native Mouse and/or Keyboard to a connected computer. A word of caution on using the Mouse and Keyboard libraries: 
 * if the Mouse or Keyboard library is constantly running, it will be difficult to program your board. 
 * Functions such as Mouse.move() and Keyboard.print() will move your cursor or send keystrokes to a connected computer 
 * and should only be called when you are ready to handle them. It is recommended to use a control system to turn this 
 * functionality on, like a physical switch or only responding to specific input you can control. 
 * Refer to the Mouse and Keyboard examples for some ways to handle this.

  When using the Mouse or Keyboard library, it may be best to test your output first using Serial.print(). 
  This way, you can be sure you know what values are being reported.
 */


// define to use promicro as ps/2 to usb controller
#define USE_TOUCH 1
// define to use promicro as i2c psu controller - needs Wire.h
#define USE_PSU 1
// define to also use i2c eeprom
//#define USE_EEPROM 1

// touchpad ps/2 data
#ifdef USE_TOUCH

// ps/2CLK and DATA are bidirectional open-collector signals; they are normally held high 
// by pull-up resistor on the host (arduino)
// data/clock connection - 6, 7 or 7, 8
#define TP_CLK 7 // touchpad ps/2 clock pin // MUST be interrupt pin! Pin 7 = INT4
#define TP_DATA 8 // touchpad ps/2 data pin // or D6

#include <Mouse.h>
// locally included
//#include "Mouse.h"

// touchpad functions that use above pin declarations
// touch2usb.h // Copyright 2019 Frank Adams
#include "touch2usb.h"

unsigned long touchtimestamp = millis();
unsigned long touchtime = 30; // 30ms

#endif

// I2C PSU bus - communicate with battery charger and PSU components 
#ifdef USE_PSU

// I2C support - for PSU and EEPROM
#include <Wire.h>

// on Pro Micro sda is pin 5 (D2), scl is pin 6 (D3)
// PSU has int line on pin A2, otg line on pin A3 - these are rows = change to cols
// PSU has int line on pin D1, otg line on pin D0

// otg pin -> D0
// int pin -> D1
// sda pin -> D2
// scl pin -> D3
#define PSU_OTG 0
#define PSU_INT 1
#define PSU_SDA 2
#define PSU_SCL 3

// source communicates with i2c via open drain fets - so on arduino host we need input pins pulled up
// use regular checks from promicro host to devices for i2c data
// to sense the i2c data - rows (outputs) should be held low, relevent cols (inputs) pulled up (still as inputs)
// then keys pressed will not interfere 

unsigned long psutimestamp = millis();
unsigned long psutime = 1000; // 1s

#endif

/************************************************************************/
// keyboard matrix - basic use of this program
/************************************************************************/
 
// rows are active send pins, cols are input pins for reading the byte 
// Rows: A3/D21(20), A2/D20(19), A1/D19(18), A0/D18(17), D15(16), D14(15), D16(14)
// Cols: D10(13), D2(5), D3(6), D4(7), D5(8), D6(9), D7(10), D8(11), D9(12), D1(1), D0(2) 

#define NUMKEYS 77  // 7x11=77, 8x10=80, 9x9=81

// counter for rows
uint8_t row = 0;
// row pins
uint8_t rowpins[7]={21, 20, 19, 18, 15, 14, 16}; // active pins = outputs
uint8_t maxrow = 7;

// counter for cols
uint8_t col = 0;
// cols pins
uint8_t colpins[11]={10, 2, 3, 4, 5, 6, 7, 8, 9, 1, 0}; // passive pins = inputs
uint8_t maxcol = 11;

// force scan highs on rows and check each time the cols situation via 11-bit byte number
// catch the change to determine what was pushed then send the change via usb serial
// on rows we may need some prescaler change to raise the scan frequency
// rows are on port PF (PF4, PF5, PF6, PF7) and PB (PB1, PB3, PB2) - remaining PB port also as cols (PB4, PB5, PB6)
// maybe not needed - default clock frequency is 500 or 1000 Hz, so each of 7 rows is read 70 times per second
//#define CPU_PRESCALE(n) (CLKPR = 0x80, CLKPR = (n))
//uint8_t number_keys[11]={KEY_0,KEY_1,KEY_2,KEY_3,KEY_4,KEY_5,KEY_6,KEY_7,KEY_8,KEY_9,KEY_10};

// this is SI keyboard
// matrix cols:      1  2    3     4     5     6    7   8      9    10  11
//___________________________________________________________________________
// row 1 has keys: ESC, ¨, TAB, CAPS,SHIFT, CTRL,  F7,  7,     u,    j,  n  (plus all shifted keys)
// row 2 has keys:  F1, 1,   q,    a,    <,  ALT,  F8,  8,     i,    k,  m  (plus all shifted keys)
// row 3 has keys:  F2, 2,   w,    s,    y,  GUI,  F9,  9,     o,    l,  ,  (plus all shifted keys) GUI=WIN=APPLE
// row 4 has keys:  F3, 3,   e,    d,    x,SPACE, F10,  0,     p,    č,  .
// row 5 has keys:  F4, 4,   r,    f,    c, LEFT, F11,  ',     š,    ć,  -  
// row 6 has keys:  F5, 5,   t,    g,    v, DOWN, F12,  +,     đ,    ž, UP
// row 7 has keys:  F6, 6,   z,    h,    b,RIGHT, F13,DEL,BSPACE,ENTER,SHIFT

// reading of 11 bit byte on row 5 (cols pulled down)
/* 00000000000 = no key pressed // do not send anything - we don't check this */
// 10000000000 = F4
// 01000000000 = 4
// 00100000000 = r
// 00010000000 = f
// 00001000000 = c
// 00000100000 = LEFT
// 00000010000 = F11
// 00000001000 = ' -
// 00000000100 = š {
// 00000000010 = ć '
// 00000000001 = - ?/

// usb_keyboard.h and keyboard.h has the below keys definitions 
// both are HID

// globally included 
//#include <Keyboard.h>
// locally included 
#include "Keyboard.h"

// 7bit = 64 + 32 + 16 + 8 + 4 + 2 + 1 = 127
// 11-bit = 1024 + 512 + 256 + 128 + 64 + 32 + 16 + 8 + 4 + 2 + 1 = 2047
// lookup table for cols in each row
// no need to have combinations - singular translation table
// single quote, double quote??? 
// less than, more than > <  ????
// not converted: <> should be single key -> KEY_PAGE_DOWN
// KEY_SCROLL_LOCK, KEY_NUM_LOCK, KEY_NUMBER, KEY_PAUSE, KEY_INSERT, KEY_HOME, KEY_END, KEY_PAGE_DOWN, KEY_PAGE_UP   not used
// and žŽ (now KEY_BACKSLASH)
// F13 is KEY_PRINTSCREEN

// put in PROGMEM if needed
uint8_t keycodes[7][11] = {
  {KEY_ESC, KEY_TILDE, KEY_TAB, KEY_CAPS_LOCK, KEY_SHIFT, KEY_CTRL, KEY_F7, KEY_7, KEY_U, KEY_J, KEY_N},
  {KEY_F1, KEY_1, KEY_Q, KEY_A, KEY_PAGE_DOWN, KEY_ALT, KEY_F8, KEY_8, KEY_I, KEY_K, KEY_M},
  {KEY_F2, KEY_2, KEY_W, KEY_S, KEY_Y, KEY_GUI, KEY_F9, KEY_9, KEY_O, KEY_L, KEY_COMMA },
  {KEY_F3, KEY_3, KEY_E, KEY_D, KEY_X, KEY_SPACE, KEY_F10, KEY_0, KEY_P, KEY_SEMICOLON, KEY_PERIOD},
  {KEY_F4, KEY_4, KEY_R, KEY_F, KEY_C, KEY_LEFT, KEY_F11, KEY_MINUS, KEY_LEFT_BRACE, KEY_QUOTE, KEY_SLASH},
  {KEY_F5, KEY_5, KEY_T, KEY_G, KEY_V, KEY_DOWN, KEY_F12, KEY_EQUAL, KEY_RIGHT_BRACE, KEY_BACKSLASH, KEY_UP},
  {KEY_F6, KEY_6, KEY_Z, KEY_H, KEY_B, KEY_RIGHT, KEY_PRINTSCREEN, KEY_DELETE, KEY_BACKSPACE, KEY_ENTER, KEY_SHIFT},
};

uint16_t idle_count=0;

bool touched[NUMKEYS];
// remember the last key decimal value 
uint8_t last_value=0;


void setup(){

  // set for 16 MHz clock
  //CPU_PRESCALE(0);

  // Configure all row pins as outputs
  for (int i=0;i<sizeof(rowpins);i++){
    pinMode(rowpins[i], OUTPUT);
    // or use rows in inverted mode - default levels high, active scan being low (pulling down the pulled up col pin)
    // would make it more compatible with shared i2c and ps/2 pins
    // but if client data forcing via it's open-collector outputs - then that would be interference
    // digitalWrite(row_pins[i], HIGH);
  }
  // Configure all col keys pins as inputs
  // there are external pull-downs 1M for high impedance keys (conductive rubber material...)
  // or use rows in inverted mode - default levels high, scan active  
  for (int i=0;i<sizeof(colpins);i++){
    pinMode(colpins[i], INPUT);  
    // can be pulled up via 20K internal resistor - rows should be act inverted then
    //pinMode(col_pins[i], INPUT_PULLUP);
  }

  #ifdef USE_PSU
  // I2C support - for PSU and EEPROM
  // join i2c bus (address optional for master)
  // i2c bus has PSU and optional eeprom
  // pin pullups done in library-  triggered probably here - col pins config below should remove pull-ups
  Wire.begin();
  // i2c communication should be done occasionally -> on keyboard scanning -> Wire.end()
  #endif
  
  /**
  // i2c EEPROM option - not implemented yet - could save some keyboard presets, power up options, etc.
  // arduino pin 4 switches the write protect pin of eeprom
  pinMode(WP_PIN, OUTPUT); // power up - and status ok
  digitalWrite(WP_PIN, LOW);
  // eeprom
  #ifdef USE_EEPROM
    // Start the I2C interface for i2c EEPROM
    startEEPROM();
  #endif
  **/

  // start Serial on usb for testing  output
  Serial.begin(9600);
  // initialize mouse control:
  Mouse.begin();
  
  // initialize keyboard control:
  Keyboard.begin();

  #ifdef USE_TOUCH

    // trigger setup in touch2usb.h
    touch_setup(); 
  
  #endif


  // Initialize the USB, and then wait for the host to set configuration. If the board is powered without a PC connected to the USB port,
  // this will wait forever.

  // Wait an extra second for the PC operating system to load drivers
  // and do whatever it does to actually be ready for input
  _delay_ms(1000);


}

/**
// read from TI battery charger chip
void i2c_test_loop() {
  Wire.beginTransmission(0x6B);
  Wire.write(0x0D);
  Wire.endTransmission();
  //are you using the correct address? The address given in the DS is 0x6A.
  Wire.requestFrom(0x6B, 1);    // request 6 bytes from slave device #8

  while (Wire.available()) { // slave may send less than requested
    int c = Wire.read(); // receive a byte as character
    Serial.println(c);         // print the character
  }

  delay(500);
  Wire.endTransmission();
}
**/


/**
// This interrupt routine is run approx 61 times per second. A very simple inactivity timeout is implemented, where we will send a space character.
ISR(TIMER0_OVF_vect)
{
  idle_count++;
  if (idle_count > 61 * 8) {
    idle_count = 0;
    usb_keyboard_press(KEY_SPACE, 0);
  }
}
**/

/************************************************************************************/
// main loop
/************************************************************************************/
void loop() {
  
  //uint8_t b, d, mask, i, reset_idle;
  //uint8_t b_prev=0xFF, d_prev=0xFF;

#ifdef USE_TOUCH
  // check touchpad every 30ms - ps/2 bus 
  if(millis()-touchtimestamp > touchtime ) {
    touchtimestamp = millis();
    touch_loop();
  }  
#endif

#ifdef USE_PSU
  // check i2c - this is PSU data
  if(millis()-psutimestamp > psutime ) {
    psutimestamp = millis();

    Wire.beginTransmission(0x6B);
    Wire.write(0x0D);
    Wire.endTransmission();
    //are you using the correct address? The address given in the DS is 0x6A.
    Wire.requestFrom(0x6B, 1);    // request 6 bytes from slave device #8    
    //Wire.requestFrom(2, 6);    // request 6 bytes from slave device #2
    while(Wire.available()) {
      // slave may send less than requested
      char c = Wire.read(); // receive a byte as character
      //Serial.print(c);    // print the character
    }
    Wire.endTransmission();
    
  }
#endif

  // state machine?
  // if letter key pressed - ok, print it
  // if letter key kept pressed - repeate printing it in some intervals
  // there are also function keys - they do not have print output
  // if letter key pressed and function key pressed - stop interval printing
  // if letter key pressed and another letter key pressed - print the last pressed
  // if function key pressed - ok
  // if function key pressed and letter key pressed - do the function on letter key
  // if function key pressed and letter key pressed - do the function on letter key
  // if function key pressed and function key pressed - ok
  // CTL ALT DEL - three gunction keys - prepare to stop the computer
  // but all these are functions of computer
  // we just send data



  /******************************************************************/ 
  // usb_keyboard.h code
  /******************************************************************/ 
  /**
  // Configure timer 0 to generate a timer overflow interrupt every
  // 256*1024 clock cycles, or approx 61 Hz when using 16 MHz clock
  // This demonstrates how to use interrupts to implement a simple
  // inactivity timeout.
  TCCR0A = 0x00;
  TCCR0B = 0x05;
  TIMSK0 = (1<<TOIE0);
  **/

  /**  
  while (1) {

    // read all port B and port D pins
    b = PINB;
    d = PIND;
    // check if any pins are low, but were high previously
    mask = 1;
    reset_idle = 0;
    for (i=0; i<8; i++) {
      if (((b & mask) == 0) && (b_prev & mask) != 0) {
        usb_keyboard_press(KEY_B, KEY_SHIFT);
        usb_keyboard_press(number_keys[i], 0);
        reset_idle = 1;
      }
      if (((d & mask) == 0) && (d_prev & mask) != 0) {
        usb_keyboard_press(KEY_D, KEY_SHIFT);
        usb_keyboard_press(number_keys[i], 0);
        reset_idle = 1;
      }
      mask = mask << 1;
    }
    // if any keypresses were detected, reset the idle counter
    if (reset_idle) {
      // variables shared with interrupt routines must be
      // accessed carefully so the interrupt routine doesn't
      // try to use the variable in the middle of our access
      cli();
      idle_count = 0;
      sei();
    }
    // now the current pins will be the previous, and wait a short delay so we're not highly sensitive to mechanical bounce
    b_prev = b;
    d_prev = d;
    _delay_ms(2);
  }
  **/

  bool touch;
  bool changed = false;

  // first: set current row pin to output high
  pinMode(rowpins[row], HIGH);
  // scan over all col pins to get 11-bit byte
  uint8_t value = 0; // all open = pulled down -> this is default 00000000000 all zeroes
  for (uint8_t i=0; i<maxcol;i++){
    touch = (digitalRead(colpins[i]) == 1); // pressed means row 1 managed to get through = key pressed -> result is true else false;   
    // check current reading against global from before
    if (touch != touched[i]) { // if changed
      touched[i] = touch; // update global with new value
      changed = true; // a change of key detected
    }
    // create 11-bit byte and determine raw number for key pressed -> convert to keyboard key value
    if (touched[i]) value += 1 << i; // add a 1 to the byte ->      00010000001
  }

  // get key pressed from the byte via lookup table - 1 to 1 -> no intervals
  // lookup table per each row 
  // do not send anything if value=0 - but reset it
  if(value>0) {
    for (uint8_t i = 0; i < sizeof(keycodes[row]); i++) {
      // here we search for unique values
      if(value == keycodes[row][i]) {   // ok, we found it
          value = keycodes[row][i];
          break; // do not check any further
      }
    }
  } else {
     value = 0;  // reset it - send only if this value not zero
  }

  // remember byte dec value as global
  last_value = value;

  // last: set current row pin back to output low
  pinMode(rowpins[row], LOW);

  //increment index
  row++;
  if(row >= maxrow){
      row=0; // reset if over
  }
}
