/*
 * by BS 2019
 * version: 2019-10-07
 * this file contains all definitions and variables for some musical structures
 * of midi note based system
 * adapt tone to current tonality method
 * also some chord structures methods
 * some method for changing tonality based on similarity within matrix
 * -> changeTonality(to_defined_new_tonality); // randomly=0-10
 * -> changeTonality(to_randomly_selected_new_tonality); // random in 12 steps 10-20, etc...
 * 

  // tonal modes - scales - affects melodics, harmonics
  // the seven modes that can be derived from the pitches of the C major scale:
  // - C Ionian — CDEFGABC (intervals: Whole - Whole - Half - Whole - Whole - Whole - Half)
  // - D Dorian — DEFGABCD (intervals: Whole - Half - Whole - Whole - Whole - Half - Whole)
  // - E Phrygian — EFGABCDE (intervals: Half - Whole - Whole - Whole - Half - Whole - Whole)
  // - F Lydian — FGABCDEF (intervals: Whole - Whole - Whole - Half - Whole - Whole - Half)
  // - G Mixolydian — GABCDEFG (intervals: Whole - Whole - Half - Whole - Whole - Half - Whole)
  // - A Aeolian (minor) — ABCDEFGA (intervals: Whole - Half - Whole - Whole - Half - Whole - Whole)
  // - B Locrian — BCDEFGAB (intervals: Half - Whole - Whole - Half - Whole - Whole - Whole)
  
  // C=1, Csharp=Dflat=2, D=3, Dsharp=Eflat=4, E=5, F=6, Fsharp=Gflat=7, G=8, Gsharp=Aflat=9, A=10, Asharp=Bflat=11, B(H)=12
  
  // Modes: transpose the above modes to C...
  // C Ionian: C D E F G A B
  // C Dorian: C, D, Eb, F, G, A, Bb
  // C Phrygian: C Db Eb F G Ab Bb
  // C Lydian: C D E F# G A B
  // C Mixolydian: C D E F G A Bb
  // C Aeolian: C D Eb F G Ab Bb
  // C Locrian: C Db Eb F Gb Ab Bb
  
  //Scales:
  // C-Major: C, D, E, F, G, A, B
  // C major pentatonic: C, D, E, G, A
  // C Natural Minor: C, D, Eb, F, G, Ab, Bb
  // C minor pentatonic: C, Eb, F, G, Bb
  // C blues = minor with an added flat fifth: C, Eb, F, Gb, A, Bb
  // C Melodic Minor: C D Eb F G A B
  // C Harmonic Minor: C, D, Eb, F, G, Ab, B
  // Gipsy/Hungarian/Ciprian: C, D, Eb, F#, G, Ab, B 
  // Arabic/Byzantine scale: C, Dflat, E, F, G, Aflat, B
  // Phrygian scale : C, Db, E, F, G, Ab, Bb
  // Blues scale: C, D, Eb, F, Gb, A, Bb 
 
  // do some similiar structured list also for chords 
  //- two -> diads (base tone, third tone or fifth tone)
  //- three -> triads (base tone, third tone and fifth tone)
  //- four -> tetrads (base tone, third tone, fifth tone, plus one)
  // Additional option is: inversion of chords (base note is not the lowest)
  
  // Common chord types: the first and second interval from the base tone -> 0)...  
  // Major triad: 0, 4 (terca), 7 (kvinta) 
  // Major sixth chord: 0, 4, 7, 9
  // Dominant seventh chord: 0, 4, 7, 10
  // Major seventh chord: 0, 4, 7, 11
  // Augmented triad: 0, 4, 8
  // Augmented seventh chord:  0, 4, 8, 10
  // Minor triad: 0, 3, 7
  // Minor sixth chord: 0, 3, 7, 9
  // Minor seventh chord: 0, 3, 7, 10
  // Minor-major seventh chord: 0, 3, 7, 11
  // Diminished triad: 0, 3, 6
  // Diminished seventh chord: 0, 3, 6, 9
  // Half-diminished seventh chord: 0, 3, 6, 10 
 */

// definitions of behaviours
// PUT THE CONSTANT BEHAVIORS IN THE PGM SPACE TO CONSERVE THE SPACE FOR GLOBAL VARIABLES!!!
// we do not change these variables
// PROGMEM: store data in flash (program) memory instead of SRAM
// get it as tmp_1_arr[i] = pgm_read_byte(&behaviour[randpat][i]);

// class declaration
class MidiMusicality {
  public:
  //byte note; // transfer here from the global as shared object - not needed?
  // interval holds previous and current note and adapts the current in relation to previous
  byte interval[2] = {0,0}; // transfer here from the global as shared object
  //byte intervals; // can be array of prefered intervals
  
  // variable container of selected scale notes - current scale
  uint8_t scale[12] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12}; // indexes 0-11
  uint8_t tonali; // transfer here from the global as shared object
  static const uint8_t melodicsize = 13; // size of below
  // Declare the data
  static const PROGMEM uint8_t melodics[melodicsize][12];
  /**
  // return line as array 
  uint8_t *getScale(uint8_t *dest, uint8_t idx, uint8_t len)
  {
    strncpy_P(dest, melodics[idx], len);
    dest[len] = '\0';
    return(dest);
  }
  // get it like this
  // uint8_t scale[MidiMusicality::melodicsize]; // definition
  //for (uint8_t i=0; i<MidiMusicality::melodicsize; i++)
  // Serial.println(Music.getScale(scale, i, MidiMusicality::melodicsize));
  **/

  byte midichord[2]; // holds a number of added note values
  // only use triads and diads
  // check against value > 0
  byte chordi = 0; // index of below
  const uint8_t chordsize = 4;
  const uint8_t chords[4][2] = { 
    { 4, 7}, // Major triad: 0, 4 (terca), 7 (kvinta) 
    { 3, 7}, // Minor triad: 0, 3, 7
    { 3, 8}, // Augmented triad: 0, 4, 8
    { 3, 6}, // Diminished triad: 0, 3, 6
  };
  
  byte newpair[2]={0,0};
  //debouncing time check for changing tonality
  unsigned long lastmelodychangetime = millis();
  unsigned int lastmelodychange = 250; // in ms

  // definition of public functions
  void scaleNote(byte index, byte key, byte incr, byte scale[12]); // this checks for the nearest valid note 
  // going up and down from the note
  byte checkScale(byte note, byte tonality); // function to adapt any note to selected tonality index
  void changeTonality(byte tonality); // function to set / change tonality
  byte changeTonalityAssoc(byte note, byte tonality); // function to set / change tonality associatively
  byte changeTonalityRandom(byte tonality); // function to change tonality randomly
  void generateChord(byte note, byte tonality);
  void generateChordRandom(byte note, byte tonality);
  
  // function to set interval progression of notes/ based on tonality
  byte MidiMusicality::generateInterval(byte note, byte tonality);
  // get it like this: tonality = Music.changeTonality(tonality);

  private:

}; // end musicality class declaration

// Define the data with global scope
const uint8_t MidiMusicality::melodics[13][12] PROGMEM = { 
  { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12}, // 12-tone: C, C#, D, Eb, E, F, Gb, G, Ab, A, Bb, B
  { 1, 0, 3, 4, 0, 6, 0, 8, 0, 10, 0, 12}, // C Melodic Minor: C D Eb F G A B
  { 1, 0, 3, 0, 5, 6, 0, 8, 0, 10, 0, 12}, // C major = C Ionian:  C, D, E, F, G, A, B
  { 1, 0, 3, 0, 5, 0, 0, 8, 0, 10, 0, 0}, // C major pentatonic: C, D, E, G, A
  { 1, 0, 3, 4, 0, 6, 0, 8, 9, 0, 11, 0}, // C natural minor: C, D, Eb, F, G, Ab, Bb
  { 1, 0, 3, 4, 0, 6, 0, 8, 0, 10, 11, 0}, // Dorian: C, D, Eb, F, G, A, Bb
  { 1, 0, 0, 4, 0, 6, 0, 8, 0, 0, 11, 0}, // C minor pentatonic: C, Eb, F, G, Bb
  { 1, 0, 3, 4, 0, 6, 0, 8, 9, 0, 0, 12}, // C Harmonic Minor: C, D, Eb, F, G, Ab, B  
  { 1, 0, 3, 4, 0, 0, 7, 8, 9, 0, 0, 12}, // Gipsy: C, D, Eb, F#, G, Ab, B
  { 1, 2, 0, 0, 5, 6, 0, 8, 9, 0, 0, 12}, // Arabic/Byzantine: C, Db, E, F, G, Ab, B
  { 1, 0, 0, 4, 0, 6, 7, 0, 0, 10, 11, 0}, // Blues: C, Eb, F, Gb, A, Bb
  { 1, 0, 3, 4, 0, 6, 7, 0, 0, 10, 11, 0}, // Blues with added second: C, D, Eb, F, Gb, A, Bb
  { 1, 2, 0, 4, 0, 6, 0, 8, 9, 0, 11, 0}, // C Phrygian: C Db Eb F G Ab Bb
};

// internally used functions

// check current scale for nearest note
void MidiMusicality::scaleNote(byte index, byte key, byte incr, byte scale[12]){
  // try below and above
  // try above
  byte newkey = key;
  int newindex = index + incr;
  if(newindex>11){ // check if above this octave
    // beginning of next octave: max index: 11, index 12 -> index 0
    newindex = newindex - 12;
    newkey = key + 1; //one octave higher
  }
  // if new index zero
  if(scale[newindex]==0){
    // try below
    newindex = index - incr;
    if(newindex<0){ // check if below this octave
      // end of prev octave: index -1 -> index 11
      newindex = newindex + 12;
      newkey = key - 1; //one octave lower
    } 
  }
  // set the inner pair
  newpair[0]=newindex;
  newpair[1]=newkey;
}

// adapt selected note to melodics used - not yet ok
// whatever note comes here - should be translated to one of the valid ones
// first get the number of current octave
byte MidiMusicality::checkScale(byte note, byte tonali){
  // tonality var defines the scale used - index of melodics[6][12]
  // change it as global and use here - tonali is not getting here
  MidiMusicality::changeTonality(tonali);
  
  // scales defined in melodics[6][12]
  // first we have to change the note to relative position in basic octave
  // eg: whatever C to C0
  // A-1=9, A0=21, A1=33, A2=45, A3=57, A4=69, A5=81, A6=93, A7=105, A8=117 
  // C-1=0, C0=12, C1=24, C2=36, C3=48, C4=60, C5=72, C6=84, C7=96, C8=108 
  byte key=0;
  byte refnote = note;
  // note is 1 to 127 
  // refnote is 1 to 12
  // notepos is index -> 0 to 11
  // key is the octave
  // below we should never get refnote == 0

  if(note>=120)     { refnote=note-120+1; key=10; }
  else if(note>=108){ refnote=note-108+1; key=9; }
  else if(note>=96) { refnote=note-96+1; key=8; }
  else if(note>=84) { refnote=note-84+1; key=7; }
  else if(note>=72) { refnote=note-72+1; key=6; }
  else if(note>=60) { refnote=note-60+1; key=5; }
  else if(note>=48) { refnote=note-48+1; key=4; }
  else if(note>=36) { refnote=note-36+1; key=3; }
  else if(note>=24) { refnote=note-24+1; key=2; }
  else if(note>=12) { refnote=note-12+1; key=1; }
  // else key=0 - it is ok
  
  // now check for our note in scale - zero means to check next or previous note
  // eg: fifth note corresponds to fifth scale array element
  // if zero - check the next and previous
  // refnote is 1 to 12 - position of note is 0 for 1 and 11 for 12
  int notepos = refnote-1; // will be able to go into negative //ok
  
  byte newpos = notepos;
  byte newkey = key;
  newpair[0] = newpos;
  newpair[1] = newkey;
  // check if zero
  if(scale[notepos]==0){
    // find the nearest valid note
    // not good! Notes go over 127
    // notepos, increment
    scaleNote(notepos, key, 1, scale); // updates newpair
    newpos = newpair[0];
    newkey = newpair[1];
    // check the new index
    if(scale[newpos]==0){
      // notepos, increment
      scaleNote(notepos, key, 2, scale);
      // check the new index
      newpos = newpair[0];
      newkey = newpair[1];

      if(scale[newpos]==0){
        // notepos, increment
        scaleNote(notepos, key, 3, scale);
        // check the new index
        newpos = newpair[0];
        newkey = newpair[1];
      }      
    }
  }

  byte newref=scale[newpos];

  // put back to absolute position - including -1 
  // here we get tones that are over 127 limit! 
  note = 12*newkey + newref - 1;
  
  return note;
}

// main functions

// function to set interval progression of notes/ based on current tonality
// this is on per-note basis
byte MidiMusicality::generateInterval(byte note, byte tonality){
  // gets a note and adapts it to interval progression
  // progression can be described similarly as chords array
  // especially usefull when multiple same notes are registered
  // so check for previous note
  byte prevnote = interval[1];
  interval[0] = prevnote;
  // use some other array than chords!
  //select random index
  chordi = random(chordsize); 
  interval[1] = note;  
  // also allow to go down!
  byte inversion = random(2); 
  if(inversion>0){
    note = checkScale(note - chords[chordi][0], tonality);  
  } else {
    note = checkScale(note + chords[chordi][0], tonality);  
  }

  return note;
}


void MidiMusicality::changeTonality(byte tonality){
  if(tonality < melodicsize){
    for(byte i=0; i<12;i++){
      // fill the line
      scale[i]=pgm_read_byte(&melodics[tonality][i]);
      //scale[i]=getScale(scale, tonality, melodicsize);
    }
    tonali = tonality;
  }
  // no need to return - this is forced 
}

// returns associatively new tonality index - not the same as tonali
// this checks the upper and lower indexes of melodics if the same note exists
// do this in the same way as checkScale and scaleNote
// note here is real midi note - not index!
// on succes it changes the tonality - do not do this too often
byte MidiMusicality::changeTonalityAssoc(byte note, byte tonality){
  if(tonality < melodicsize){
    // find index for the given note
    byte refnote = note;
    // note is 1 to 127 
    // refnote is 1 to 12
    // notepos is index -> 0 to 11
    // key is the octave
    // below we should never get refnote == 0
  
    if(note>=120)     { refnote=note-120+1; }
    else if(note>=108){ refnote=note-108+1; }
    else if(note>=96) { refnote=note-96+1; }
    else if(note>=84) { refnote=note-84+1; }
    else if(note>=72) { refnote=note-72+1; }
    else if(note>=60) { refnote=note-60+1; }
    else if(note>=48) { refnote=note-48+1; }
    else if(note>=36) { refnote=note-36+1; }
    else if(note>=24) { refnote=note-24+1; }
    else if(note>=12) { refnote=note-12+1; }
    // else key=0 - it is ok
    
    // now check for our note in scale - zero means to check next or previous note
    // eg: fifth note corresponds to fifth scale array element
    // if zero - check the next and previous
    // refnote is 1 to 12 - position of note is 0 for 1 and 11 for 12
    byte notepos = refnote-1;

    // check one index above and below
    if(tonality+1<melodicsize && pgm_read_byte(&melodics[tonality+1][notepos])>0){
      // ok
      tonality = tonality+1;
    } else if(tonality-1>0 && pgm_read_byte(&melodics[tonality-1][notepos])>0){
      // ok
      tonality = tonality-1;
    } // else leave out for now
    // maybe make a jump over two indexes?
    
    tonali = tonality;
    for(byte i=0; i<12;i++){
      // fill the line with new tonality
      scale[i]=pgm_read_byte(&melodics[tonality][i]);
    }

  }
  return tonality;
}

// returns randomly new tonality index - not the same as tonali
byte MidiMusicality::changeTonalityRandom(byte tonality){
  // check here the notes in current scale
  if(tonality < melodicsize){
     //have some time check
    if(millis()-lastmelodychangetime>lastmelodychange){
      lastmelodychangetime = millis();
      // do not select the same tonality! Maybe have some logic to through indexes 
      // if tonalities have some logic of progression
      byte prevtonality = tonality;
      while(tonality==prevtonality){ // find a different tonality index
        tonality = random(melodicsize);
      }
      // set tonality    
      tonali = tonality;
      for(byte i=0; i<12;i++){
        //scale[i]=melodics[tonali][i];
        scale[i]=pgm_read_byte(&melodics[tonali][i]);
      }
    }
  }
  return tonality;
}

// creates chord notes - adapts midichord - transfer elswhere
void MidiMusicality::generateChord(byte note, byte tonali){
  //default = normal - lined above - ok
  chordi=0; // first index // Major triad: 0, 4 (terca), 7 (kvinta) 
  midichord[0]=checkScale(note + chords[chordi][0], tonali);
  midichord[1]=checkScale(note + chords[chordi][1], tonali);

}

// creates chord notes - adapts midichord - transfer elswhere
void MidiMusicality::generateChordRandom(byte note, byte tonali){
  // generate a three tone chord 
  // remember this index over time
  //chordi is index of major, minor, augmented, diminished chord
  //select random index
  chordi = random(chordsize); 
  // here we might want to reduce triad to diad 
  // or do some chord inversion (go below midinote[0]
  // change globals
  // should not go below 0!
  byte inversion = random(4); //0:no inversion, 1:first go below, 2:second go below, 3: both go below
  // generate chord notes - to be transferred to main script
  // with checkscale into minus we can get over 127?!

  //default = normal - lined above - ok
  midichord[0]=checkScale(note + chords[chordi][0], tonali);
  midichord[1]=checkScale(note + chords[chordi][1], tonali);

  // minus can go below 0 if note is low
  if(inversion==1) {
    midichord[0]=checkScale(note - chords[chordi][0], tonali);
  } else if(inversion==2) {
    midichord[1]=checkScale(note - chords[chordi][1], tonali);
  } else if(inversion==3) {
    midichord[0]=checkScale(note - chords[chordi][0], tonali);
    midichord[1]=checkScale(note - chords[chordi][1], tonali);
  }

  // another additional option is to remove some of the tone from the chord note: set the midichord note to 0 
  byte reduction = random(3); //0:no, 1:remove first, 2:remove second
  if(reduction==1){
    midichord[0]=0;
  } else if (reduction==2){
    midichord[1]=0;
  }
  
  // if any of the chord notes is above 127 - drop it
  // this is when note is low and transposed down - minus
  if(midichord[0]>127)  midichord[0]=0;
  if(midichord[1]>127)  midichord[1]=0;
   
  // if any of the chord notes equals basic note - drop it
  if(midichord[0]==note)  midichord[0]=0;
  if(midichord[1]==note)  midichord[1]=0;
 
}
