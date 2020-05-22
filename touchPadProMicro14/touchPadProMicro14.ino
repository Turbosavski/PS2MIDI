/**
* Sparkfun Arduino Pro Micro 16MHz 5V
* 
*   Version >=13 is an upgrade to versions <13 -> state machine
*   
* usb midi and DIN midi out
* to use internally the DreamBlaster Synth S2 - just connect its MIDI IN to Tx pin 
* have audio stereo output
* have psu jack (9V) or just use usb micro for 5V psu 
* internally - 9V battery connector
* 
* very large latency and slow on MIDI OUT when used with usb midi
* when used with usb midi - still freezes after some time 
* solution: do not use both at the same time: switch between DIN and usb
* 
*/

/* T1004 touchpad capacitive sensing ASIC designed by Synaptics
 * two modes: mouse-compatible relative mode and absolute mode with pressure sensing
 * http://sparktronics.blogspot.com/2008/05/synaptics-t1004-based-touchpad-to-ps2.html
 * Majority of touchpads out there operate with PS/2 standard. A PS2 data bus has 4 pins:
   - Data
   - Clock
   - Ground
   - Vcc (+5V)
*/
/*
// to generate midinote using some random and statistics
// save last notes in a sequence and decide based on that
// some tonality options
// melodic line is the key moment here
// stop note = note off - left button
// chord option - right button
// polyphonic would be when one note playing - being met with another note - so that both play
// - for this sending to a second midi channel would be needed - and alternating channels
// additional button: SI (standard intelligence) or AI (advanced intelligence)
// some leds: green (ON), red or yellow (note out activity - can be with pwm to show pressure) 
// etc
*/

const bool test = false; //false true;
const bool testtone = false; //false true;

// switch between the two serial outputs
// since they have large latency together

// true to send usb midi -> Serial
bool usbmidi = false; //false true;

// true: send additional (classic=ttl) midi on hw -> Serial1
bool midibaudrate = true; //true or false

//uint8_t midiCH = 0; // midi channel
char midiCH = 0; // midi channel

// comment out if not used
#define AI 1

// maybe have a pot to set this?
// finger area and xy movement check - affects scale slide
unsigned int sensibility = 150; // 50 minimum, 200 optimum, 500 seems stable

// state machine
// no note is playing
#define NOTE_OFF 1
// a transition from below to above the threshold value detected - wait to see how fast the breath velocity is increasing
#define RISE_WAIT 2
// note is playing
#define NOTE_ON 3

// The state of our state machine
uint8_t state = 1; // NOTE_OFF

#define TOUCH_THRESH 10
// we wait for 3-10 milliseconds of continuous touch above THRESHOLD before we turn on the note, to de-glitch
#define RISE_TIME 3 //3, 5, 10

// The time that we noticed the touch off -> on transition
unsigned long touch_on_time = millis(); // = 0L;
// The touch value at the time we observed the transition - this remembers the attack value
uint8_t initial_touch_value;

// end state machine

uint8_t touch_midi_value = 0;
uint8_t prev_touch_midi_value = 0; // check if breath is changing
uint8_t prev_note_value; // previous note value - to stop the note

// send aftertouch, volume, modwheel, portamento, pitchbend data no more than every  milliseconds
#define MIDIMETA_INTERVAL 50
// the last time we sent volume value
unsigned long midivolumeTime = millis(); // 0L;
// the last time we sent modulation, aftertouch value
unsigned long midimetaTime = millis(); // 0L;

// pins used - not used: 5 
// pins 2 and 3: SDA and SCL - to eeprom
#define WP_PIN 4 // eeprom WP pin always low to enable writing

// on any synptics board - testpoints:
// T28: ground
// T22: 5V
// T11: data, 
// T10 - clock
// T7 - left
// T6 - right
// additional buttons
// T4 - up
// T5 - down

// CLK and DATA are bidirectional “open-collector” signals; they are normally held at 
// high (+5V) level by a 5–10K pull-up resistor on the host (arduino)
// check correct data/clock connection - 6, 7 - on pcb 5 and 6 - can be reversed
#define PS2_CLK 6 // 5, touchpad ps/2 clock pin T11
#define PS2_DATA 5 // 6, touchpad ps/2 data pin T10

#define LEDPIN_RED 10 // red  - for midi out traffic
#define LEDPIN_ORANGE 16 // orange to show AI selected
#define LEDPIN_GREEN 14 // green for device powered and status ok
#define MODEPIN 15 // switch live between SI and AI
#define MIDIPIN 18 // A0 // switch live between DIN and USB midi out

// globals
// global to hold touchpad z,x,y sensor data
unsigned int data[3];
// global to hold touchpad previous data
unsigned int prevdata[3];
// global to hold touchpad rendered midi data
byte midinote[3]; // holds note, velocity and timing 
byte prevmidinote[3]; // holds note, velocity and timing 
// timing should measure how long the finger is held on touchpad!
// to basic note we can attach additional notes - to make a chord
byte midichord[2]; // holds a number of added note values
byte prevmidichord[2]; // holds a number of added note values

byte tonality = 1; // 1: C-minor, 2: C-major

// remember touch as firsttouch true when starting tone
// and put to false of finger pressure drops below threshold
bool firsttouch = false;
bool lasttouch = false;
bool noteoff = true; // always use it on next nete triggered - force it sooner with left button 
bool leftbutt = false;
bool rightbutt = false;
bool chord = false; // make a chord or not
bool slide = false; // sense that we have a slide movement
unsigned long notetime = millis();
byte preveventtime = notetime/1000; // remember as relative

// when registers fill up - start from zero or pop the first and add the last
// or change keys for all elements? Too much work?
// or remember current index and use this to find what is in the past:
// smaller indexes = near past <- current index -> higher indexes far past

// how many notes to remember
// fifo or some global counter/pointer - overwriting the current value in arrays
// shifting array values to the left?
const unsigned int numnotes = 127;
// register to remember sequence of events:
// separate lists with corresponding indexes:
uint8_t register_notes[numnotes];
uint8_t register_volumes[numnotes];
uint8_t register_times[numnotes]; 
// 1 here means 0.1 second, 127 means 12 seconds
// counter of events - fills indexes up to numnotes - then resets
unsigned int events=0;

unsigned long lastchangetime = millis();
unsigned long lastchangeinterval = 10000; // in ms = 10s

// this keeps synaptics touchpad and midi under control
// seems that midiusb crashes often
#define INTERVAL_LENGTH_US 75000UL  // microsecs = 50ms, 75ms or 100ms

unsigned long previousMicros;
//unsigned long previousMicros = INTERVAL_LENGTH_US;

// all these are local files

#include "midimusicality.h"
//instantiate an object in order to call its member functions
MidiMusicality Music;

// global to be changed
bool ai = false;
#ifdef AI
  #include "midistatistics.h"
#endif

// this is for synaptics ps2 touchpad
// my version faster pinmode, read and write functions
#include "ps2bs.h"

/**    
 * Synaptics touchpad
 * sequence to set the absolute touchpad sensor mode:
 * if a Set Sample Rate 20 ($F3, $14) command is preceded by four Set Resolution 
 * commands encoding an 8-bit argument, the 8-bit argument is stored as the new value for 
 * the TouchPad mode byte
 * to set the mode byte to 0xC1 (absolute mode, high packet rate, Wmode enabled) use sequence: 
 * 0xf0 0xe8 0x03 0xe8 0x00 0xe8 0x00 0xe8 0x01 0xF3 0x14
 * we use (absolute mode, high packet rate, Wmode enabled) sequence: 
 * 0xf0 0xe8 0x03 0xE8 0x00 0xE8 0x01 0xE8 0x00 0xF3 0x14
 * it is important to ensure that the device is disabled (0xF5) before sending this command sequence 
 * to receive Absolute mode packets, follow this sequence with an Enable (0xF4) command.
 * 
 * Data packets are sent in response to Read Data (0xEB) commands. If Stream mode is
 * selected and data reporting is enabled, data packets are also sent unsolicited whenever
 * finger motion and/or button state changes occur. Synaptics recommends using Stream
 * mode instead of Read Data commands to obtain data packets
**/

PS2 touchpad(PS2_CLK, PS2_DATA);      
// self-adjustment of touchpad boundaries
unsigned int maxx,minx,maxy,miny,maxz,minz;

// midi usb stuff - standard library
#include <MIDIUSB.h>

byte mididata[] = {
  0, // 5th byte
  0, // 6th byte - channel index byte - channels 1-16 - or some other adress for effects
  0, // 7th byte - instrument group byte - index - or some other adress
  0  // 8th byte - instrument byte - index - or some other adress or control value
};

//select instrument index from the list below
byte instrument = 0;
// find instrument data by instrument index
const int NUMBER_OF_VOICES = 46;
const byte instruments[NUMBER_OF_VOICES][2] PROGMEM = {
  {0,0},  // Grand Piano
  {0,2},  // Bright Piano
  {0,3}, // El. Grand Piano
  {0,4}, // Honky-tonk Piano
  {0,5}, // El. Piano 1 
  {0,6}, // El. Piano 2 
  {0,7}, // Harpsichord
  {0,8}, // Clavi
  {0,9}, // Celesta
  {0,10}, // Glockenspiel
  {0,11}, // Music Box
  {0,12}, // Vibraphone
  {0,13}, // Marimba
  {0,14}, // Xylophone
  {0,15}, // Tubular Bells
  {0,16}, // Santur
  {0,17}, // Drawbar Organ
  {0,25}, // Nylon Guitar
  {0,26}, // Steel Guitar
  {0,27}, // Jazz Guitar
  {0,32}, // Guitar Harmonics
  {0,33}, // Acoustic Bass
  {0,34}, // Finger Bass
  {0,35},  //Pick Bass
  {0,36},  //Fretless Bass
  {0,37}, // Slap Bass 1
  {0,41}, // "Violine"               
  {0,44}, // "Contrabass"            
  {0,46}, // "Pizzicato Strings"     
  {0,47}, // "Orchestral Harp"       
  {0,48}, // "Timpani"               
  {0,53}, // "Choir Aahs"            
  {0,54}, // "Voice Oohs"            
  {0,57}, // "Trumpet"               
  {0,58}, // "Trombone"              
  {0,59}, // "Tuba"                  
  {0,66}, // "Tenor Sax"             
  {0,71}, // "Basoon"                
  {0,72}, // "Clarinet"              
  {0,105}, // "Sitar"                 
  {0,107}, // "Shamisen"              
  {0,108}, // "Koto"                  
  {0,109}, // "Kalimba"               
  {0,116}, // "Woodblock"             
  {0,117}, // "Taiko Drum"            
  {0,119} // "Synth Drum"            
};
// get it like this:
//pgm_read_byte(&instruments[0][i])

void touchpad_init(){
  touchpad.write(0xff);  // reset
  touchpad.read();  // ack byte
  touchpad.read();  // blank 
  touchpad.read();  // blank 

  //sequence start
  touchpad.write(0xf0);  // remote mode
  touchpad.read();  // ack
  delayMicroseconds(100);
  
  touchpad.write(0xe8); // set resolution
  touchpad.read();  // ack byte
  touchpad.write(0x03); // x1  ( x1 * 64  +  x2 * 16  +  x3 * 4  +  x4   == modebyte )
  touchpad.read();  // ack byte

  touchpad.write(0xe8); // set resolution
  touchpad.read();  // ack byte
  touchpad.write(0x00); // x2
  touchpad.read();  // ack byte

  touchpad.write(0xe8); // set resolution
  touchpad.read();  // ack byte
  touchpad.write(0x01); // x3
  touchpad.read();  // ack byte
  
  touchpad.write(0xe8); // set resolution
  touchpad.read();  // ack byte
  touchpad.write(0x00); // x4
  touchpad.read();  // ack byte
  
  touchpad.write(0xf3); // set samplerate 20 (this stores the mode) 
  touchpad.read();  // ack byte
  touchpad.write(0x14);
  touchpad.read();  // ack byte

  //sequence stop
  delayMicroseconds(100); 
}

void noteOn(uint8_t channel, uint8_t pitch, uint8_t velocity) {
  midiEventPacket_t noteOn = {0x09, 0x90 | channel, pitch, velocity};
  if(usbmidi) MidiUSB.sendMIDI(noteOn);
}
void noteOff(uint8_t channel, uint8_t pitch) {
  midiEventPacket_t noteOff = {0x08, 0x80 | channel, pitch, 0};
  if(usbmidi) MidiUSB.sendMIDI(noteOff);
}

void controlChange(uint8_t channel, uint8_t control, uint8_t value) {
  midiEventPacket_t event = {0x0B, 0xB0 | channel, control, value};
  if(usbmidi) MidiUSB.sendMIDI(event);
}


// Simple pseudo-random number generator
// Multiply-with-carry method invented by George Marsaglia.
// two initializers (not null) - or we always get the same
unsigned long m_w = 1;
unsigned long m_z = 2;

// pseudo random generator
unsigned long getRandom(){
    m_z = 36969L * (m_z & 65535L) + (m_z >> 16);
    m_w = 18000L * (m_w & 65535L) + (m_w >> 16);
    return (m_z << 16) + m_w;  // 32-bit result
}

void setup(){

  // start measuring micros
  unsigned long startup = micros();
  // if A3 left unconnected
  unsigned int randnoise = analogRead(A3); // A3 should not be connected anywhere

  // arduino pin 4 switches the write protect pin of eeprom
  pinMode(WP_PIN, OUTPUT); // power up - and status ok
  digitalWrite(WP_PIN, LOW);
  
  pinMode(LEDPIN_GREEN, OUTPUT); // power up - and status ok
  digitalWrite(LEDPIN_GREEN, LOW);

  pinMode(LEDPIN_ORANGE, OUTPUT); // show AI selected
  digitalWrite(LEDPIN_ORANGE, LOW);

  pinMode(LEDPIN_RED, OUTPUT); // midi traffic
  digitalWrite(LEDPIN_RED, LOW);

  pinMode(MODEPIN, INPUT_PULLUP); // switch live between SI and AI
  pinMode(MIDIPIN, INPUT_PULLUP); // switch live between DIN/internal or USB midi
  pinMode(PS2_CLK, INPUT_PULLUP); // held at high
  pinMode(PS2_DATA, INPUT_PULLUP); // held at high

  // true: send additional (classic=ttl) midi on hw -> Serial1
  // if we use the Tx pin with MIDI OUT we use MIDI baud rate 
  // might be that we need to .begin or .end this serial on the fly 
  Serial1.begin(31250);  //  Set MIDI baud rate

  //MIDIUSB library needs activation here - to later function normally!!!
  // should not block Serial1 - if midiusb not connected
  byte index = 0;
  for(byte i=0;i<5;i++){
    index = i+50;
    noteOn(midiCH, index, 100);
    //noteOff(midiCH, index);
    // Flush the output
    if(usbmidi) MidiUSB.flush();
    digitalWrite(LEDPIN_RED, HIGH);
    delay(250);
    digitalWrite(LEDPIN_RED, LOW);
    delay(250);
  }
  midinote[0]=index;
  midinote[1]=100;

  if(test){
    // internal usb serial port is Serial
    // this replaces the above midiUSB
    Serial.begin(115200);
    // wait for serial port to connect. Needed for native USB serial port only
    while (!Serial) { ; }
    Serial.println("Usb serial connected");
  }
  // eeprom  used with AI mode
  #ifdef USEEEPROM
    // Start the I2C interface for i2c EEPROM
    startEEPROM();
  #endif

  if (test) {
    Serial.print("Detecting touchpad... ");
  }

  // ps2 lib version
  touchpad_init();

  if (test) {
    Serial.println("Touchpad initialization OK ");
  }
  maxx = 0;
  minx = 32000;
  maxy = 0;
  miny = 32000;
  maxz = 0;
  minz = 32000;

  //setup the defined register_data size
  for(uint8_t i=0; i<numnotes; i++){
    register_notes[i]=64;
    register_volumes[i]=64;
    register_times[i]=64; // translate this (0-127) to milliseconds *100=12700ms=12.7s 
  }
  // end of setup

  // status ok
  digitalWrite(LEDPIN_GREEN, HIGH);
  delay(250);

  // mode is SI = LOW - otherwise HIGH for AI
  digitalWrite(LEDPIN_ORANGE, LOW);
  #ifdef AI
    digitalWrite(LEDPIN_ORANGE, HIGH);
    delay(250);
  #endif

  // select instrument - // we have instrument as index
  // 1. MAIN SOUNDS - GENERAL MIDI on all channels but 10
  // Program change 1:piano, 2: etc -> PC
  // 2. MT-32 SOUND VARIATION #127 all channels but 10 -> To select variation: send CTRL 0 = 127, then PC
  // + DRUM SET TABLE (MIDI CHANNEL 10)
  
  // maybe first reset the S2 chip
  // instrument variable is index of instruments list 
  //instrument = 0;
  //mididata[0] = 8; // 5th byte value
  //mididata[1] = midiCH;  // default for most
  // get from flash
  //mididata[2] = pgm_read_byte(&instruments[instrument][0]); // 7th byte value
  mididata[3] = pgm_read_byte(&instruments[instrument][1]); // 8th byte value

  //FFh: Reset S2 synth to power-up condition -> Note on: 9nh kk vv 
  Serial1.write(0xFF);
  
  // set new GM instrument
  programChangeSend(0xC0, (char)mididata[3]);

  // set new MT-32 sound variation 
  // send CTRL 0 = 127, then PC
  //controlSend(0xB0, 0x00, 0x7F);
  //programChangeSend(0xC0, (char)mididata[3]);

  unsigned long intertime = micros()-startup;
  // use Marsaglia pseudo random generator 
  unsigned long randnum = getRandom()*randnoise*startup; // 32-bit result
  randomSeed(randnum); //25440
  if(test){
    Serial.print("Random seed: "); //1192609344,418614272, etc
    Serial.println(randnum);
  }

  // initialize state machine
  state = NOTE_OFF;

  // short delay
  delay(50);
}

// HW MIDI - simple = raw = no library
// -> internal or MIDI OUT - in parallel or single

//NOTE ON message is structured as follows:
// Status byte : 1001 CCCC
// Data byte 1 : 0PPP PPPP
// Data byte 2 : 0VVV VVVV
//where:
//"CCCC" is the MIDI channel (from 0 to 15)
//"PPP PPPP" is the pitch value (from 0 to 127)
//"VVV VVVV" is the velocity value (from 0 to 127)
// noteOnSend(0x90, (char)midinote[0], (char)midinote[1]);
void noteOnSend(char cmd, char data1, char data2) {
  cmd = cmd | char(midiCH);  // merge channel number
  //to send a single byte - use Serial1.write()
  Serial1.write(cmd);
  Serial1.write(data1);
  Serial1.write(data2);
}

//NOTE OFF message is structured as follows:
//  Status byte : 1000 CCCC   where CCCC and PPPPPPP have the same meaning as above. 
//  Data byte 1 : 0PPP PPPP
//  Data byte 2 : 0VVV VVVV   The VVVVVVV is the release velocity, which is very rarely used. By default, set it to zero.
// noteOffSend(0x80, (char)midichord[0]);
void noteOffSend(char cmd, char data1) {
  cmd = cmd | char(midiCH);  // merge channel number
  //to send a single byte - use Serial1.write()
  Serial1.write(cmd);
  Serial1.write(data1);
  Serial1.write(0);
}

//The CC message is constructed as follows:
//  Status byte : 1011 CCCC
//  Data byte 1 : 0NNN NNNN
//  Data byte 2 : 0VVV VVVV
// where CCCC is the MIDI channel, NNNNNNN is the controller number (from 0 to 127) 
//and VVVVVVV is the value assigned to the controller (also from 0 to 127).
// controlSend(0xB0, (char)midichord[0]);
void controlSend(char cmd, char data1, char data2) {
  cmd = cmd | char(midiCH);  // merge channel number
  //to send a single byte - use Serial.write()
  Serial1.write(cmd);
  Serial1.write(data1);
  Serial1.write(data2);
}

// "program change" message. It has one STATUS byte and one DATA byte :
//  Status byte : 1100 CCCC   where CCCC is the MIDI channel (0 to 15) 
//  Data byte 1 : 0XXX XXXX   and XXXXXXX is the instrument number from 0 to 127. 
// Similarly to MIDI channels, you will often see that the instrument numbers in synthesizers and in GM lists, 
// are numbered from 1 to 128 so you also need to add or subtract 1 for the conversion.
// programChangeSend(0xC0, (char)data1);
void programChangeSend(char cmd, char data1) {
  cmd = cmd | char(midiCH);  // merge channel number
  //to send a single byte - use Serial.write()
  Serial1.write(cmd);
  Serial1.write(data1);
}


void loop() {
  
  // check some switches
  // check MODEPIN -> low: AI   high/default: SI
  if(digitalRead(MODEPIN)>0){
    ai = false;
  } else {
    ai = true;
  }
  
  // check MIDIPIN -> low: DIN+internal   high/default: USBMIDI
  // switch live between DIN and USB midi out
  if(digitalRead(MIDIPIN)>0){
    usbmidi = false;
    midibaudrate = true;
  } else {
    usbmidi = true;
    midibaudrate = false;
  }
 
  // have some rest for midi and synaptics
  unsigned long currentMicros = micros();
  if ((currentMicros - previousMicros) >= INTERVAL_LENGTH_US){
    previousMicros += INTERVAL_LENGTH_US;
    // Initialize the packages
    
    // ps2 library is ok when reading of data done in the loop!
    touchpad.write(0xeb); // demand data
    // we get up to here with T1004BE - ok with T1004
    // library problem? Waits indefinitely
    
    // Absolute packet format - each motion report consists of six bytes
    touchpad.read(); // byte 1 is ACK byte - ignore
    // read next byte - this one also holds the button down
    byte mstat1 = touchpad.read();
    // read next byte
    byte mxy = touchpad.read();
    // read next byte
    byte cz = touchpad.read();
    // read next byte
    byte mstat2 = touchpad.read();
    // read next byte
    byte mx = touchpad.read();
    // read next byte
    byte my = touchpad.read();
    delayMicroseconds(100);

    // collect the bits for x and y
    unsigned int cx = (((mstat2 & 0x10) << 8) | ((mxy & 0x0F) << 8) | mx );
    unsigned int cy = (((mstat2 & 0x20) << 7) | ((mxy & 0xF0) << 4) | my );
    // below are just bits 0 or 1 - if trackpad buttons pushed
    bool left = mstat1 & 0x01;
    bool right = (mstat1 & 0x02) >> 1;

    // find some option for multitouch detection - at least for two fingers
    // check if xy position changed in time of same touch

    /**
    // now we use state machine
    // for single tone
    if(cz<10){ //it drops below threshold - finger lifted
      firsttouch = false;
      lasttouch = true;
      slide = false; // reset
    }
    **/
    
    // if left button clicked - trigger noteoff for current note
    // force it off in any case - even if note not triggered
    if(left>0)      leftbutt = true;
    else   leftbutt = false;

    // if right button clicked 
    // can be some more complex function: either a chord or tonal change
    if(right>0)     rightbutt = true;
    else      rightbutt = false;
 
    // if both - change tonality explicitly
    if(rightbutt && leftbutt){
      // random change of the tonality = scale
      tonality = Music.changeTonalityRandom(tonality); // maybe somewhere else
      // no chord 
      chord = false; //true;
      //no note off
      noteoff=false; 
    } else if(rightbutt){
      // do a chord - only if button held down!
      chord = true;
      slide = false; // in case slide detected - drop it
    } else {
      chord = false; // reset to no chord
    }
    
    if(leftbutt && noteoff){
      noteoff=false; // remember that we used it already
      leftbutt = false;
      if(midinote[1]>0){
        noteOff(midiCH, midinote[0]);
        // if midichord not empty
        if(midichord[0]>0) noteOff(midiCH, midichord[0]);
        if(midichord[1]>0) noteOff(midiCH, midichord[1]);
        // Flush the output collected above
        if(usbmidi) MidiUSB.flush();
      }
      // also on hw serial
      if(midibaudrate && midinote[1]>0){
        noteOffSend(0x80, (char)midinote[0]);

        if(midichord[0]>0){
          noteOffSend(0x80, (char)midichord[0]);
        }
        if(midichord[1]>0){
          noteOffSend(0x80, (char)midichord[1]);
        }
        Serial1.flush(); //clears the buffer once all outgoing characters have been sent
      }
      //reset
      midichord[0]=0;
      midichord[1]=0;
      chord = false; // reset to no chord
    }



    /**
    // a bit less sensitivity to pressure here
    // check if xy distance big enough to mean additional finger added
    // or finger sliding over touchpad
    //if(cz>20 && cz<100 && millis()-notetime>250 && millis()-notetime<1000){ 
    // here we get double same notes - not good
    // 50ms is here the minimum timing resolution - can be more
    if(cz > 10 && !chord && millis()-notetime>50){ 
      // prevdata[0] x: low value is around 1400, high value around 5000
      // prevdata[1] y: low value is around 1200, high value around 4000
      // prevdata[2] z 
      // this works ok when going up the scale but not down 
      //prevdata should be referential 
      // maybe have some option to set the sensibility
      // sliding finger - sensibility = amount of change cz, cx, cy: 50-500 
      // checking abs(prevdata[2]-cz)>sensibility && blocks a lot!
      //if(abs(prevdata[2]-cz)>sensibility && (abs(prevdata[0]-cx)>sensibility || abs(prevdata[1]-cy)>sensibility) ){
      // but we should check it to block the double notes when finger down
      if(abs(prevdata[0]-cx)>sensibility || abs(prevdata[1]-cy)>sensibility ){
        firsttouch = false; // means that we shouold trigger as a new touch 
        lasttouch = true;
        slide = true; // remember not to repeat each tone
      }
    }
    **/

    // state machine
    // intialize
    uint8_t note_value = 0; 
    if (state == NOTE_OFF) {
      if (cz > TOUCH_THRESH && cz < 128) {
         // value has risen above threshold -> move to the RISE_TIME state 
         // record time and initial breath value
         touch_on_time = millis();
         initial_touch_value = cz;
         state = RISE_WAIT;  // Go to next state
         return;
      }
      // else remain in NOTE_OFF
      
    } else if (state == RISE_WAIT) {
      if (cz > TOUCH_THRESH && cz < 128) {
        
        // has enough time passed for us to collect our second sample?
        if (millis() - touch_on_time > RISE_TIME) {
          
          // ok, calculate midi note, then send a note_on event
          note_value = getTouchNote(cx,cy,cz,left,right);
          
          // if no valid data - ignore
          if (note_value == 0) {
            return;
          }
  
          state = NOTE_ON; // changed state to be played NOW
          
        } else {
          // else remain in RISE_WAIT state
          return;      
        }
        
      } else {
        // touch value fell below threshold before RISE_TIME passed
        // return to NOTE_OFF state - ignoring a short blip of touch
        state = NOTE_OFF;
        return;  // do not trigger note off event below!
      }
      
    } else if (state == NOTE_ON) {
  
      if (cz > TOUCH_THRESH && cz < 128) {
  
        // did touch value changed significantly enough to generate a volume midi message
        // is it time to send more aftertouch data?
        if (millis() - midivolumeTime > MIDIMETA_INTERVAL) {
          // send the volume controller sooner - do not trigger note
          // use some redundancy here -> 1 or 2 or 4 or 8
          if (abs(touch_midi_value - prev_touch_midi_value) > 4) {
            prev_touch_midi_value = touch_midi_value;
            // send volume control
            if (usbmidi && !test) controlChange(midiCH, 0x07, 0x00 | touch_midi_value);
            //if (midibluetooth && !test) controlSend(midiCH, 0x07, 0x00 | touch_midi_value);
            if(midibaudrate)  controlSend(midiCH, 0x07, 0x00 | touch_midi_value);

          }
          midivolumeTime = millis();
        }
        
        // get the current key note
        // do something on the basis of data above and create a midinote
        note_value = getTouchNote(cx,cy,cz,left,right);
        
        // if no valid data - ignore
        if (note_value == 0) {
          return;
        }
  
        // a new fingering was detected while still blowing
        if (note_value != prev_note_value) {
          // send note off on previous and note on on current
          // the current state is still NOTE_ON
          // we will play the new note
        } else {
          // while touching - we got the same note
          // we must go via NOTE_OFF state to play the same note again
          // so jump out of this NOW
          return;
        }
        
      } else {
        // note is being played but breath value has fallen below threshold - trigger note_off
        state = NOTE_OFF;
      }
  
    } // end state machine




    // play the note
    // 128,0,0 and 128,4069,4069 is invalid!
    //if (cz > 10 && cz < 128  && !firsttouch) {
    //if (cz > 20 && cz < 128  && !firsttouch) {
    
    if(note_value > 0 && ( state == NOTE_ON || state == NOTE_OFF ) ) { 
      
      // remember before changed
      prevmidinote[0] = midinote[0]; // note
      prevmidinote[1] = midinote[1]; // velocity
      prevmidinote[2] = midinote[2]; // timing - if needed

      // send volume control with every new note - this should reflect some attack!
      uint8_t attack_midi_value = touch_midi_value + 10;
      if(attack_midi_value>127) attack_midi_value = 127;
      if (usbmidi && !test) controlChange(midiCH, 0x07, 0x00 | attack_midi_value);
      //if (midibluetooth && !test) controlSend(midiCH, 0x07, 0x00 | attack_midi_value);
      if (midibaudrate && !test) controlSend(midiCH, 0x07, 0x00 | attack_midi_value);

      // we registered current note already -> in midinote

      // send  midinote off
      // for multiple channels polyphony this will be different
      if (state == NOTE_OFF){
        if(midinote[1]>0){
          noteOff(midiCH, midinote[0]);
          // if midichord not empty
          if(midichord[0]>0){
            noteOff(midiCH, midichord[0]);
          }
          if(midichord[1]>0){
            noteOff(midiCH, midichord[1]);
          }
        }  
      }

      // this is mostly for Serial1
      prevmidichord[0]=midichord[0];
      prevmidichord[1]=midichord[1];
      
      midichord[0]=0; // reset
      midichord[1]=0; // reset

      if (state == NOTE_ON){
      // send  midinote - this collects note off above and note on below!
        //MIDIUSB library note on
        if(chord){
          // in case of chord - make velocity a bit higher!
          if(midinote[1]<75) midinote[1]=75;
          // make a sequence of notes from midinote - midichord variable
          generateChord();
          // use just once per touch
          // basic note
          noteOn(midiCH, midinote[0], midinote[1]);
          // chord notes - only if above 0! Same velocity as midi note -> midinote[1]
          if(midichord[0]>0) noteOn(midiCH, midichord[0], midinote[1]);
          if(midichord[1]>0) noteOn(midiCH, midichord[1], midinote[1]);
          // to test - send also tonality as CC20
          controlChange(midiCH, 20, tonality);
  
        } else {
          noteOn(midiCH, midinote[0], midinote[1]);
          // to test - send also tonality as CC20
          controlChange(midiCH, 20, tonality);
        }
        
        if (note_value != prev_note_value) {
          noteOff(midiCH, prev_note_value);
          noteOn(midiCH, midinote[0], midinote[1]);
          prev_note_value = note_value;
         
        } else { 
          // else here we have the same note to be played 
          // note off was triggered during NOTE_OFF state
          noteOn(midiCH, midinote[0], midinote[1]);
          prev_note_value = note_value;
        }      

        // Flush the combined midiusb output
        if(usbmidi) MidiUSB.flush();
        
      }
      
      /**      
      // check if slide - do not have double note
      if(slide && midinote[0]==prevmidinote[0]){
        slide = false;
        return; // do not continue - go a new loop
      }
      **/
      

      // duplicate on hw MIDIOUT
      if(midibaudrate){
        // send raw data
        //0xB0, note[i][bank], 0x60 - controllers
        // send  midinote off
        noteOffSend(0x80, (char)prevmidinote[0]);

        // check the prev chord
        if(prevmidichord[0]>0){
          noteOffSend(0x80, (char)prevmidichord[0]);
        }
        if(prevmidichord[1]>0){
          noteOffSend(0x80, (char)prevmidichord[1]);
        }
        Serial1.flush(); //clears the buffer once all outgoing characters have been sent
        // reset old
        prevmidichord[0]=0;
        prevmidichord[1]=0;

       if(chord){
          // in case of chord make velocity a bit higher!
          if(midinote[1]<75) midinote[1]=75;
          //send note on
          noteOnSend(0x90, (char)midinote[0], (char)midinote[1]);
          
          // add chord notes - same velocity as basic note
          if(midichord[0]>0){
            noteOnSend(0x90, (char)midichord[0], (char)midinote[1]);
          }
          if(midichord[1]>0){
            noteOnSend(0x90, (char)midichord[1], (char)midinote[1]);
          }

       } else {
          //send note on
          noteOnSend(0x90, (char)midinote[0], (char)midinote[1]);
       }
       Serial1.flush(); //clears the buffer once all outgoing characters have been sent
      }
      
      if(chord){
        chord = false; // reset
      }

      // register note event  
      // counter of events - fills indexes up to numnotes - then resets
      byte prevtime = preveventtime;
      if(events==0){ // in case this is first event
        prevtime = 0; // set this as reference
      } else if(events>numnotes){ // in case the buffer is full - turn over
        events=0; // reset index/counter
        prevtime = register_times[numnotes];
      }
      // this should be relative time = from the previous event
      register_times[events]=(byte)(millis()/1000 - prevtime); 
      // 1 here means 0.1 second, 127 means 12 seconds
      // remember current time as global for the next time around 
      preveventtime = notetime/1000; // remember as relative

      register_notes[events]=midinote[0];
      register_volumes[events]=midinote[1];
      // now update index by 1
      events++;
      
      // tell us that we had first touch already
      firsttouch = true;
      noteoff = true; // reset and be prepared to catch noteoff

      // new timestamp
      notetime = millis();
      // midi activity led
      digitalWrite(LEDPIN_RED, HIGH);
    }
    
    chord = false; // reset to no chord
    slide = false; // reset

    // seems usbmidi goes to sleep after some time
    // send noteoff every ten secs
    if(millis()-notetime>10000){
      notetime = millis();
      noteOff(midiCH, midinote[0]);
      // Flush the output
      if(usbmidi) MidiUSB.flush();
    }
        
  }
  
  if(millis()-notetime>50){ // 50-100ms
    digitalWrite(LEDPIN_RED, LOW);
  }
}

// adapts globals
byte getTouchNote(unsigned int cx, unsigned int cy, unsigned int cz, bool left, bool right ){
  if(test){
    Serial.print(cz); //z = pressure
    Serial.print(" ");
    Serial.print(cx); //x
    Serial.print(" ");
    Serial.print(cy); //y
    Serial.print(" ");
    Serial.print(left); //left
    Serial.print(" ");
    Serial.print(right); //right
    Serial.println(" ");
  }
  
  // remember what has been registered as active to check for changed finger position
  prevdata[0] = cx; //x = horizontal position
  prevdata[1] = cy; //y = vertical position
  prevdata[2] = cz; //z = pressure

  // autocalibrate
  // this means that we have to click at every piano start between mins and maxs
  // in some way we could remember setting in eeprom
  if (cx < minx) { minx = cx; }
  if (cx > maxx) { maxx = cx; }
  if (cy < miny) { miny = cy; }
  if (cy > maxy) { maxy = cy; }
  cz += 45;
  if (cz > 127) cz = 127; 
  
  // determine XY midi note range
  //data[0] = map(cx,0,127,24,96); //x
  data[0] = map(cx,minx,maxx,24,96); //x
  //data[1] = map(cy,0,127,24,96); //y
  data[1] = map(cy,miny,maxy,24,96); //y
  //data[2] = map(cz,minz,maxz,0,127); //z=pressure
  data[2] = cz; //z=pressure

  // generate note - either from xy position or ai
  generateNote();

  return midinote[0];
}

// generate a midi note from data sent
// use midimusicality.h functions -> checkScale(), changeTonality(), generateChord()
void generateNote(){
  // we have data global from xy touchpad
  byte newnote = (data[0]+data[1])/2; // note: now here x y combined
  // change tonality - selects the current scale 
  Music.changeTonality(tonality);
  // adapt to nearest in the selected scale
  newnote = Music.checkScale(newnote, tonality);
  
  // check here statistics etc and other AI stuff and adapt again newnote
  // define some timecheck etc - to do this
  if(ai && timeToChange()){
    tonality = Music.changeTonalityAssoc(newnote, tonality);
  }
  
  // generate here midinote trio: note, velocity, time
  midinote[0] = (byte)newnote; // note
  midinote[1] = (byte)data[2]; // velocity = now here pressure (z)
  midinote[2] = 64; // measure of timestamp = times 100
  
}

void generateChord(){
  // change tonality in class - selects the current scale 
  Music.changeTonality(tonality);
  Music.generateChordRandom(midinote[0], tonality);
  // transfer back midichord from class to globals  
  midichord[0]=Music.midichord[0];
  midichord[1]=Music.midichord[1];
}
