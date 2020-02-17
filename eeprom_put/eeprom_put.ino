#include <EEPROM.h>

struct MyObject {
  float xyz[9][3];
};

void setup() {

  Serial.begin(57600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  

  /** Put is designed for use with custom structures also. **/

  //Data to store.
  MyObject customVar = {{
    {48.0066137, 35.1016483, 10},
    {48.0066137, 35.1016483, 10},
    {48.0066137, 35.1016483, 10},
    {48.0066137, 35.1016483, 10},
    {48.0966137, 35.1996483, 10},
    {48.0066137, 35.1016483, 10},
    {48.0066137, 35.1016483, 10},
    {48.0066137, 35.1016483, 10},
    {48.1066137, 35.0086483, 10}}
  };

  EEPROM.put(0, customVar);
  Serial.print("Written custom data type! \n\nView the example sketch eeprom_get to see how you can retrieve the values!");
  
}

void loop() {   /* Empty loop */ }
