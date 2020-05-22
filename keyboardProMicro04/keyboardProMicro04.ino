/**
* Sparkfun Arduino Pro Micro 16MHz 5V
* PS2 KEYBOARD MIDI SYNTH
* usb midi and DIN midi out
* to use internally the DreamBlaster Synth S2 - just connect its MIDI IN to Tx pin 
* Dreamblaster S1 : http://www.serdashop.com/waveblaster
* have audio stereo output
* have psu jack (9V) or just use usb micro for 5V psu 
* internally - 9V battery connector
* 
* very large latency and slow on MIDI OUT when used with usb midi
* when used with usb midi - still freezes after some time 
* solution: do not use both at the same time: switch between DIN and usb
* 
 * The Pro Micro has five external interrupts, which allow you to instantly trigger a function when a pin 
 * goes either high or low (or both). If you attach an interrupt to an interrupt-enabled pin, you’ll need 
 * to know the specific interrupt that pin triggers: pin 3 maps to interrupt 0, pin 2 is interrupt 1, pin 
 * 0 is interrupt 2, pin 1 is interrupt 3, and pin 7 is interrupt 4.
**/


const bool test = true; //false true;
const bool testtone = false; //false true;

unsigned long testime = millis();

// switch between the two serial outputs
// since they have large latency together

// true to send usb midi -> Serial
bool usbmidi = false; //false true;

// true: send additional (classic=ttl) midi on hw -> Serial1
bool midibaudrate = true; //true or false

//uint8_t midiCH = 0; // midi channel
char midiCH = 0; // midi channel

// maybe have a pot to set this?
// finger area and xy movement check - affects scale slide
//unsigned int sensibility = 150; // 50 minimum, 200 optimum, 500 seems stable

// comment out if not used
//#define AI 1

// pins used - not used: 5 
// pins 2 and 3: SDA and SCL - to eeprom
#define WP_PIN 4 // eeprom WP pin always low to enable writing

//Make sure you have data and clock wires connected correctly. 
//Clock wire MUST be connected to an interrupt pin!

// include local ps2 keyboard library - adapted by BS
#include "PS2Keyboard.h"
// CLK and DATA are bidirectional “open-collector” signals; they are normally held at 
// high (+5V) level by a 5–10K pull-up resistor on the host (arduino)
// check correct data/clock connection - 6, 7 - on pcb 5 and 6 - can be reversed

#define PS2_DATA 6 // 6, touchpad ps/2 data pin
#define PS2_CLK 7 // 5, touchpad ps/2 clock pin // MUST be connected to interrupt pin! Pin 7 = INT4

PS2Keyboard keyboard;

// use this or not
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

// transposition 4 here means: 4 * 12 + note / first note is C1=12 -> C5=60
// this will be changed with some plus/ minus buttons
// do not go below 0!
byte transposition = 3;
// this will be changed with some plus/ minus buttons
// do not go below 48!
byte velocity = 96;

// tonality will simplify playing - reduce/ replace notes that are not used with those used
byte tonality = 0; // 1: C-minor, 2: C-major

// have notes remember button pushed (0->1) / button released (1->0)
byte button[32]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

bool iskeyrelease = false;

// this is from touchpadPiano

// remember touch as firsttouch true when starting tone
// and put to false of finger pressure drops below threshold
bool firsttouch = false;
bool lasttouch = false;
bool noteoff = true; // always use it on next note triggered - force it sooner with left button 
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

// midi usb stuff - standard library
// allow to switch between the internal synth and usbmidi
#include <MIDIUSB.h>

byte mididata[] = {
  0, // 5th byte
  0, // 6th byte - channel index byte - channels 1-16 - or some other adress for effects
  0, // 7th byte - instrument group byte - index - or some other adress
  0  // 8th byte - instrument byte - index - or some other address or control value
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


// local lib - optional
//#include "Button.h"

int key; // char
int prevkey; //char

long calctempodelay = 0;
long tempodelay = 500;
long curtimestamp = 0;
long prevtimestamp = 0;
long lasttickstamp =0;
long wheelfactor = 6; // adjust this to modify the pulseperiod to tempo ratio (wheel radius), higher = faster
int  tickflag = 0; // indicate if time tick has passed

const int dreamblaster_enable_pin = 6;
int buttonState = 0; 
int prev_buttonState = 0; 

int enabledrumming = 0;
int enablebassline = 0;
int enablesynthline = 0;

long seq_poscnt = 0;
byte resonantchannel= 1;
byte channel1_prog = 54;
byte channel2_prog = 81;

byte bassvolume = 0x50;
byte drumvolume = 0x50;
byte synthvolume = 0x50;
byte leadvolume  = 0x70;

/**
#define _BASSDRUM_NOTE 0x24
#define _SNAREDRUM_NOTE 0x26
#define _CLOSEDHIHAT_NOTE 0x2A
#define _PEDALHIHAT_NOTE 0x2C
#define _OPENHIHAT_NOTE 0x2E
#define _CYMBAL_NOTE 0x31
#define _DRUMPRESS_STATUSCODE 0x99
#define _DRUMRELEASE_STATUSCODE 0x89
#define _SYNTHPRESS_STATUSCODE 0x90
#define _SYNTHRELEASE_STATUSCODE 0x80

void midiwrite(int cmd, int pitch, int velocity) {
  Serial.write(cmd);
  Serial.write(pitch);
  Serial.write(velocity);
}

void midiprogchange(int cmd, int prog) {
  Serial.write(cmd);
  Serial.write(prog);
}
**/

/**
void midisetup_sam2195_nrpn_send(int channel, int control1,int control2, int value){
   Serial.write((byte)(0xB0+channel));
   Serial.write(0x63);
   Serial.write((byte)control1);
   Serial.write(0x62);
   Serial.write((byte)control2);
   Serial.write(0x06);
   Serial.write((byte)value);
}
void midisetup_sam2195_sysexcfg_send(int channelprefix,int channel, int control, int value){
   Serial.write(0xF0);
   Serial.write(0x41);
   Serial.write(0x00);
   Serial.write(0x42);
   Serial.write(0x12);
   Serial.write(0x40);
   Serial.write((byte)(channelprefix*16+channel));   
   Serial.write((byte)control);
   Serial.write((byte)value);   
   Serial.write(0x00);   
   Serial.write(0xF7);   
}
void midisetup_sam2195_gsreset(void){
  midisetup_sam2195_sysexcfg_send(0,0,0x7F,0);
}
void setupvoices(void)
{
    midiprogchange(0xC0,channel1_prog);
    midiprogchange(0xC1,channel2_prog);
}
void stopallnotes(byte selection = 0){  
  if (selection == 0 || selection ==2){
      midiwrite(0x90, 52, 0x00); // switch off possible synth notes
      midiwrite(0x90, 40, 0x00); // switch off possible synth notes     
  }
  if (selection == 0 || selection ==1){
      midiwrite(0x91, 28, 0x00); // switch off possible bass notes
      midiwrite(0x91, 40, 0x00); // switch off possible bass notes           
      midiwrite(0x91, 52, 0x00); // switch off possible bass notes
      midiwrite(0x91, 64, 0x00); // switch off possible bass notes
  }
}

**/

/**
void timercallback()
{
  lasttickstamp = millis();
  Timer1.setPeriod(tempodelay * 1000);     
  tickflag = 1;
}
**/

/**
void SomeButtonPressHandler(Button& butt){  
 if(enabledrumming)
  {
    if(butt == button3) {
      enablebassline = enablebassline?0:1;
    }

  } else {
    
    if(butt == button3) {  
      midiwrite(0x99, _SNAREDRUM_NOTE, 0x65);   
    }
    if(butt == button4) {  
      midiwrite(0x99, _BASSDRUM_NOTE, 0x65);   
    }
  }
}

void SomeButtonReleaseHandler(Button& butt){
  if(enabledrumming) {
    //..
  } else {
   // ..
  }
}

void SomeButtonHoldHandler(Button& butt){
  if(butt==button4) {
     stopdrum(); 
  }
}

void PotControl(int potnr, int ctrlVal){
  if(potnr == 1) {
     //Serial.println(ctrlVal);
      midisetup_sam2195_nrpn_send(resonantchannel,0x01,0x20,(byte)ctrlVal); 
      //midiwrite(0xB0, 0x01,(byte) ctrlVal);  // modulation wheel
  }
   if(potnr == 2) {
     midisetup_sam2195_nrpn_send(resonantchannel,0x01,0x21,(byte)ctrlVal);  // set resonance for channel resonantchannel
  }
}

void ADC_handle(void){
  static int prevsensorVal1 = -1000;  
  static int prevsensorVal2 = -1000;
  if(pot1setup) {
    int sensorValue = analogRead(A4);
    int ctrlValue;
    if(abs(sensorValue-prevsensorVal1)>3) {
        // filter noise from adc input
        prevsensorVal1 = sensorValue;
        ctrlValue = sensorValue/8;
        PotControl(1,ctrlValue);
    }
  }
  if(pot2setup) {
    int sensorValue = analogRead(A2);
    int ctrlValue;
    if(abs(sensorValue-prevsensorVal2)>3) {
        // filter noise from adc input
        prevsensorVal2 = sensorValue;
        ctrlValue = sensorValue/8;
        PotControl(2,ctrlValue);
    }
  }
}

**/

/**
void OLDsetup() {
  Serial.begin(31250);
  analogReference(DEFAULT); 
  
  pinMode(dreamblaster_enable_pin ,OUTPUT);   // enable the dreamblaster module by pulling high /reset
  digitalWrite(dreamblaster_enable_pin, HIGH);
  delay(300);  // allow 500ms for the dreamblaster to boot
  midisetup_sam2195_gsreset();
  delay(500);  // allow 250 for gs reset

  keyboard.begin(DataPin, IRQpin);  // ps2 keyboard
  
  //midisetup_sam2195_sysexcfg_send(0x02,0x00,0x01,0x50);  // set mod wheel cutoff
  //midisetup_sam2195_sysexcfg_send(0x02,0x00,0x04,0x00);  // disable mod wheel pitch variation
  //midisetup_sam2195_sysexcfg_send(0x02,0x00,0x05,0x7F);  // set mod wheel tvf depth
  midisetup_sam2195_nrpn_send(resonantchannel,0x01,0x21,(byte)0x7F);  // set resonance for channel resonantchannel

  Timer1.attachInterrupt(timercallback);
  Timer1.initialize(tempodelay*1000);   
  
  button3.pressHandler(SomeButtonPressHandler);
  button3.releaseHandler(SomeButtonReleaseHandler);  
  button4.pressHandler(SomeButtonPressHandler);
  button4.releaseHandler(SomeButtonReleaseHandler);
  button4.holdHandler(SomeButtonHoldHandler,1000);
  
  setupvoices();
}
**/

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

  // internal usb serial port is Serial
  Serial.begin(115200);
  //while (!Serial) { ; } // this blocks when no usb serial communication established!

  if(test){
    // wait for serial port to connect. Needed for native USB serial port only
    Serial.println("Usb serial connected");
  }
  // eeprom  used with AI mode
  #ifdef USEEEPROM
    // Start the I2C interface for i2c EEPROM
    startEEPROM();
  #endif

  if (test) {
    Serial.print("Detecting keyboard... ");
  }

  // pin pullups done in library
  // ps2 keyboard lib -> keyboard.begin(DataPin, IRQpin, keymap_data) 
  keyboard.begin(PS2_DATA, PS2_CLK, PS2Keymap_MIDI); // ps2 keyboard clock must be interrupt pin (use pin7) / int4
  
  if (test) {
    Serial.println("Keyboard initialization OK ");
  }

  //setup the defined data register size
  // this will allow to record the sequence played
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

  // midibaudrate -> true: send classic (=ttl) midi on hw -> Serial1
  // if we use the Tx pin with MIDI OUT we use MIDI baud rate 
  // might be that we need to .begin or .end this serial on the fly 
  Serial1.begin(31250);  //  Set MIDI baud rate for S2 synth

  // allow 500ms for the dreamblaster to boot -> above is enough
  // select instrument - // we have instrument as index
  // 1. MAIN SOUNDS - GENERAL MIDI on all channels but 10
  // Program change 1:piano, 2: etc -> PC
  // 2. MT-32 SOUND VARIATION #127 all channels but 10 -> To select variation: send CTRL 0 = 127, then PC
  // + DRUM SET TABLE (MIDI CHANNEL 10)
  
  // first reset the S2 chip - done in hw / we might want to use pin
  // instrument variable is index of instruments list 
  //instrument = 0;
  //mididata[0] = 8; // 5th byte value
  //mididata[1] = midiCH;  // default for most
  // get from flash
  //mididata[2] = pgm_read_byte(&instruments[instrument][0]); // 7th byte value
  mididata[3] = pgm_read_byte(&instruments[instrument][1]); // 8th byte value

  //FFh: Reset S2 synth to power-up condition -> Note on: 9nh kk vv 
  Serial1.write(0xFF);
  delay(500);  // allow 250 for gs reset
  
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
// NOTE_ON 0x90
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
// NOTE_OFF 0x80
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
// Control Change 0xB0
// controlSend(0xB0, (char)midichord[0]);
void controlSend(char cmd, char data1, char data2) {
  cmd = cmd | char(midiCH);  // merge channel number
  //to send a single byte - use Serial.write()
  Serial1.write(cmd);
  Serial1.write(data1);
  Serial1.write(data2);
}

//Control Change 0xB0
// Modulation wheel effect //(0xB0 + channel, 0x01, value)
void midiModWheel(char cmd, char data2) {
  cmd = cmd | char(midiCH);  // merge channel number
  char data1 = 0x01; 
  Serial1.write(cmd);
  Serial1.write(data1);
  Serial1.write(data2);
}

// "program change" message. It has one STATUS byte and one DATA byte :
//  Status byte : 1100 CCCC   where CCCC is the MIDI channel (0 to 15) 
//  Data byte 1 : 0XXX XXXX   and XXXXXXX is the instrument number from 0 to 127. 
// Similarly to MIDI channels, you will often see that the instrument numbers in synthesizers and in GM lists, 
// are numbered from 1 to 128 so you also need to add or subtract 1 for the conversion.
// PROGRAM_CHANGE 0xC0
// programChangeSend(0xC0, (char)data1);
void programChangeSend(char cmd, char data1) {
  cmd = cmd | char(midiCH);  // merge channel number
  //to send a single byte - use Serial.write()
  Serial1.write(cmd);
  Serial1.write(data1);
}

// optional:
// KEY_PRESSURE 0xA0
// CHANNEL_PRESSURE 0xD0
// PITCH_BEND 0xE0

void loop() {

  //if (keyboard.availablerel()) {
  if (keyboard.available()) {
    // read the next key
    //iskeyrelease = false; //false
    // this sends 
    //key = keyboard.readrel(iskeyrelease);   // this returns a note or some code
    // no need for key release?
    key = keyboard.read();   // this returns a note number or some code
    // here we get a rafal of numbers from the library - if button pushed longer 
    
    // if button not pushed - declare that this is a push
    // and do not allow any additional notes to be played - until button released
    if(test && key>0){
      Serial.print(" pressed ");   
      Serial.print(key);   
      Serial.print(" which is note ");   
      Serial.print(midinote[0]);   
      //Serial.print(" pushed or released ");   
      //Serial.print(button[key]);   
      Serial.println(" ");   
      testime = millis();
    }  
    
    //if(button[key]==0) button[key]=1; // was pushed - block this note until next time -> released
    //else  button[key]=0; // was released -> back to normal

    // do what we want from this key
    //keyboardHandler(key, iskeyrelease);

    // transposition 4 here means: 4 * 12 + note / C1=12 -> C5=60
    midinote[0]= transposition*12 + key;
    midinote[1]=velocity;

    // play the note when key below 28 - or above 11
    if(key<28 && key>11){  // && button[key]==0
      // play it
      // do not trigger additional notes - if same note held
      //if(button[key]==1) button[key]=0;

      //if(firsttouch && midinote[0]==prevmidinote[0]){
      //  noteoff = true; // we had the first touch already
      //}

      /**
      if(noteoff) {
        // key released - stop the note
        //noteOff(uint8_t channel, uint8_t pitch)
        noteOff(midiCH, midinote[0]); // USBMIDI
        // Flush the combined midiusb output
        if(usbmidi) MidiUSB.flush();
  
        if(midibaudrate){
          noteOffSend(0x80, (char)midinote[0]);
          //noteOffSend(0x90, (char)midinote[0]);
        
        }  
        noteoff = false;
        firsttouch = false;

      }
      **/

      // do something on the basis of data above and create a midinote
      //generateNote();    
  
      notetime = millis();
      // remember before changed
      prevmidinote[0] = midinote[0]; // note
      prevmidinote[1] = midinote[1]; // velocity
      prevmidinote[2] = midinote[2]; // timing - if needed
      
     if(usbmidi){
       noteOn(midiCH, midinote[0], midinote[1]); // USBMIDI
       // Flush the combined midiusb output
       MidiUSB.flush();
     }
    
      // duplicate on hw MIDIOUT
     if(midibaudrate){
        //send note on
        noteOnSend(0x90, (char)midinote[0], (char)midinote[1]);
        //Serial1.flush(); //clears the buffer once all outgoing characters have been sent
     } 
     // register the note played
     //if(button[key]==0) button[key]=1;

        
    } else {
      
      // some other key: 1 to 11 or above 28
      
      //#define M_TRANSPOSE_M  1
      //#define M_TRANSPOSE_P  2 
      //#define M_VOLUME_M     3
      //#define M_VOLUME_P     4
      
      if(key==1 && transposition>1){
        transposition--;   // minus 1
      } 
      if(key==2 && transposition<7){
        transposition++;   // plus 1  
      } 
      if(key==3 && velocity>48){
        velocity-=12;   // minus 12 - have some larger interval
      } 
      if(key==4 && velocity<121){
        velocity+=12;   // plus 12 - have some larger interval
      } 
      // limits
      if(velocity<48)  velocity=48;
      if(velocity>127)  velocity=127;
     
    } // play the note

  } // keyboard available
  

  // seems usbmidi goes to sleep after some time
  // send noteoff every ten secs
  if(millis()-notetime>1000){
    notetime = millis();
    noteOff(midiCH, midinote[0]);
    // Flush the output
    if(usbmidi) MidiUSB.flush();
  }

 
}

void keyboardHandler(char key, bool iskeyrelease){
  static bool space_pressed = false;  
  static bool enter_pressed = false;  
  static bool zero_pressed = false;  
  static bool c_pressed = false;  
  static bool v_pressed = false;  
  static bool b_pressed = false;  

  if((key==PS2_PAGEUP)|| (key==PS2_PAGEDOWN) || (key=='+')|| (key=='-'))
  {
    if(!iskeyrelease) 
     {
        if(key=='+') 
        {
          channel2_prog++;
          if(channel2_prog > 0x7F)
          {
            channel2_prog = 0x00; 
          }       
        } 
       if(key=='-') 
        {
          channel2_prog--;
          if(channel2_prog > 0x7F)
          {
            channel2_prog = 0x7F; 
          }       
        } 
       if(key==PS2_PAGEUP) 
        {
          channel1_prog++;
          if(channel1_prog > 0x7F)
          {
            channel1_prog = 0x00; 
          }       
        } 
       if(key==PS2_PAGEDOWN)
        {
          channel1_prog--;
          if(channel1_prog > 0x7F)
          {
            channel1_prog = 0x7F; 
          }       
        } 

       //setupvoices();
     }
  }
  if(key==' ') 
   {
     if(!space_pressed || iskeyrelease) 
     {     
        //midiwrite(drumstatuscode, _BASSDRUM_NOTE, volumecode);      
     }
     space_pressed = !iskeyrelease;
     return;
   }
  if(key==PS2_ENTER) 
   {
     if(!enter_pressed  || iskeyrelease) 
     { 
      //midiwrite(drumstatuscode,_CLOSEDHIHAT_NOTE, volumecode); 
     }
     enter_pressed = !iskeyrelease;
     return;
   }
  if(key=='0') 
   {
     if(!zero_pressed || iskeyrelease) 
     { 
      //midiwrite(drumstatuscode,  _SNAREDRUM_NOTE, volumecode);      
     }
     zero_pressed = !iskeyrelease;
     return;
   }
   if(key=='c') 
   {
     if(!c_pressed || iskeyrelease) 
     { 
      //midiwrite(synthstatuscode,  40, volumecode);      
     }
     c_pressed = !iskeyrelease;
     return;
   }
   if(key=='v') 
   {
     if(!v_pressed || iskeyrelease) 
     { 
      //midiwrite(synthstatuscode,  45, volumecode);      
     }
     v_pressed = !iskeyrelease;
     return;
   }
   
   if(key=='b') 
   {
     if(!b_pressed || iskeyrelease) 
     { 
      //midiwrite(synthstatuscode,  50, volumecode);      
     }
     b_pressed = !iskeyrelease;
     return;
   }
   
   if(key=='1') 
   {
     if(!iskeyrelease) 
     {        
       buttonState = (buttonState==0) ? 1 : 0;        
     }
     return;
   }
   
   if(key=='2') 
   {
     if(!iskeyrelease) 
     { 
        //enablebassline = enablebassline?0:1;
        //if(!enablebassline) {stopallnotes(1);};
     }
     return;
   }
   
   if(key=='3') 
   {
     if(!iskeyrelease) 
     {        
       enabledrumming = (enabledrumming==0) ? 1 : 0;        
     }
     return;
   }
   
}
