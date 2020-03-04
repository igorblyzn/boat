#include <EEPROM.h>

struct MyObject{
    float x[9];
  float y[9];
  int z[9];
};

void setup(){

  
  Serial.begin( 57600 );

  //Get the float data from the EEPROM at position 'eeAddress'

  MyObject eeprom; //Variable to store custom object read from EEPROM.
  EEPROM.get( 0, eeprom );

  Serial.println( "Read custom object from EEPROM: " );

   for (int i = 0; i < 9; i++) {
    Serial.print(eeprom.x[i],7); Serial.print(" ");Serial.print(eeprom.y[i],7); Serial.print(" ");Serial.println(eeprom.z[i]);
  }
}

void loop(){ /* Empty loop */ }
