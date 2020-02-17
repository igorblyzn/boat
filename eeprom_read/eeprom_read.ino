#include <EEPROM.h>

struct MyObject{
  float xyz[9][3];
};

void setup(){

  
  Serial.begin( 57600 );

  //Get the float data from the EEPROM at position 'eeAddress'

  MyObject eeprom; //Variable to store custom object read from EEPROM.
  EEPROM.get( 0, eeprom );

  Serial.println( "Read custom object from EEPROM: " );

   for (int i = 0; i < 9; i++) {
    for (int j = 0; j < 3; j++) {
      Serial.print(eeprom.xyz[i][j]); Serial.print(" ");
    }
    Serial.println();
  }
}

void loop(){ /* Empty loop */ }
