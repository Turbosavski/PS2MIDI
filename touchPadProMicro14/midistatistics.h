/**
 * taken from my attinyRobot34
 * still needs to be adapted
 * add last change to statistics - affect the next decision
 * usage:
 * preferred = checkStatistics(movement);
**/

// have some identities to affect the random number
uint8_t ID = 3; // 1, 2, 3 identity should be different for two or more same systems
byte intval = 1; //secs

// compare note (or chord notes) to available melodic lines list and select the next notes from 
// the most likely (or least likely) decision for next
// have some statistics regarding up down - combine this decision with the melodic note selection
// based on timestamps history detect the need for small or large tonal jumps 
// remember last statistics on power down

byte prefstatistics[3]={80,10,10}; // preferred percentage of time per each movement type 
byte statistics[3]={80,10,10}; // measured percentage of time per each movement type 
byte preferred = 0; // preferred next decision

// needed for I2C EEPROM -> sda scl pins: 2 and 3
// EEPROM 24LC64 etc. is at address 0x50
// 24lc512: WP pin 7 must be connected to GND = write operations enabled - here DIO pin 4 can switch it ...
// SDA pin is also an open-drain terminal, therefore, the SDA bus requires a pull-up resistor to VCC 
// typical 10K for 100KHz, 2K for 400kHz and more - of SCL speed
// define to use i2c eeprom
#define USEEEPROM 1
#ifdef USEEEPROM
  #include <Wire.h>
  #define disk1 0x50    //Address of 24LC64 (8Kx8b), 24LC128 (16Kx8b), 24lc256 (32Kx8b), 24lc512 (64Kx8b)
#endif

// eeprom read & write
#ifdef USEEEPROM

void writeEEPROM(int deviceaddress, unsigned int eeaddress, byte rdata ) {
  Wire.beginTransmission(deviceaddress);
  Wire.write((int)(eeaddress >> 8));   // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  Wire.write(rdata);
  Wire.endTransmission();
  delay(5);
}
byte readEEPROM(int deviceaddress, unsigned int eeaddress ) {
  byte rdata = 0xFF;
  Wire.beginTransmission(deviceaddress);
  Wire.write((int)(eeaddress >> 8));   // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  Wire.endTransmission();
  Wire.requestFrom(deviceaddress, 1);
  if (Wire.available()) rdata = Wire.read();
  return rdata;
}
// startup function
void startEEPROM() {
  // Start the I2C interface with EEPROM
  Wire.begin();
  if (test) {
    // to test write something in address 0
    unsigned int address = 0;
    writeEEPROM(disk1, address, 123);
    // now print out what we have
    Serial.print("Eeprom address: ");
    byte data = readEEPROM(disk1, address);
    Serial.print(data, DEC); // should return 123; returns 255 when device address not ok
    if(data==123) Serial.print(" -> OK ");
    else          Serial.print(" -> no good ");
    Serial.println(" ");
  }
}
#endif


byte checkStatistics(byte currmove) {
  // statistics should try to keep to suggested values
  // return the preferred next movement - instead of random? Or combine
  // new value = old values adapted 
  // reduce other values
  // we just need to follow the seconds - no small intervals
  byte minim = 100; // percentage
  byte maxim = 10; // minimal number to have impact on movement
  byte mov = 0; // index
  byte gap;
  // we have prefstatistics to relate to - only occasionaly
  for(byte i=0;i<3;i++){
    if(statistics[i]>99) statistics[i]=99;
    if(i==currmove) {
        // currmove and intval are last values before new movement and interval
        statistics[i] = statistics[i] + intval; // eg: 33+10 sec
    } else {
        statistics[i] = (byte)(statistics[i] - (byte)(intval/2) ); // eg: 33-5 sec
    }
    // also check if element value is minimal of the three statistics

    if(statistics[i]<minim) {
      minim = statistics[i];
      // remember as preferred mov - the least used of the three
      mov = i;
    }
    // now check against the prefstatistics - this can be done more rarely
    // here we would like to get close to the prefstatistics value
    // find the largest gap between wishes and reality
    // if more or all elements are away fromdesire - it doesn't matter which to trigger
    gap = abs(prefstatistics[i] - statistics[i]); // absolute 
    if(gap > maxim) {
      maxim = gap;
      mov = i;
    }
  }
  return mov;
}
