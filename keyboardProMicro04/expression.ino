/*
 * by BS 2019
 * version: 2019-10-03
 * this file contains dynamics of changes
 * it gets included with the main .ino script automatically if no .h ending
 */

bool timeToChange(){
  // check what we have: midinote, tonality, timing 
  if(millis()-lastchangetime > lastchangeinterval){
    lastchangetime = millis();
    return true;
  } 
  return false;
}
