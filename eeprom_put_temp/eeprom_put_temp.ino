#include <EEPROM.h>

struct MyObject {
  float x[9];
  float y[9];
  int z[9];
};

void setup() {

  Serial.begin(57600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  /** Put is designed for use with custom structures also. **/

  //Data to store.
  MyObject customVar = {
    {48.0066137, 48.0066137, 48.0066137, 48.0066137, 48.0066137, 48.0066137, 48.0066137, 48.0066137, 48.0066137},
    {35.0066137, 35.0066137, 35.0066137, 35.0066137, 35.0066137, 35.0066137, 35.0066137, 35.0066137, 35.0066137},
    {10, 11, 12, 13, 14, 15, 16, 17, 18}
  };

  EEPROM.put(0, customVar);
  Serial.print("Written custom data type! \n\nView the example sketch eeprom_get to see how you can retrieve the values!");
  
}

void loop() {   /* Empty loop */ }
