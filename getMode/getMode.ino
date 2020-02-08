#include <mavlink.h>
#include <SoftwareSerial.h>
 
#define RXpin 2
#define TXpin 3
SoftwareSerial Serial1(RXpin, TXpin); // sets up serial communication on pins 3 and 2
unsigned long tempModePrevMillis = 0;
byte ee = 1;
void setup() {
  Serial1.begin(57600); //RXTX from Pixhawk (Port 19,18 Arduino Mega)
  Serial.begin(57600); //Main serial port for console output
}
 
void loop() {
  unsigned long tempModeMillis = millis();
  if((tempModeMillis - tempModePrevMillis) > 5000){
    tempModePrevMillis = tempModeMillis;
    while(ee==1){
      getMode();
    }
    ee=1;
    Serial.println("6666666666");
  }
}
 
void getMode(){
  mavlink_message_t msg;
  mavlink_status_t status;
  while(Serial1.available()) {
    uint8_t c = Serial1.read();
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      switch(msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:{  // #0: Heartbeat
            mavlink_heartbeat_t hb;
            mavlink_msg_heartbeat_decode(&msg, &hb);
            Serial.println(hb.custom_mode);
            switch(hb.custom_mode) {
              case 0:
                Serial.println("Stabilize");
                break;
              case 1:
                Serial.println("11111");
                break;
              case 2:
                Serial.println("AltHold");
                break;
              case 3:
                Serial.println("Auto");
                break;
              case 4:
                Serial.println("Hold");
                break;
              case 5:
                Serial.println("Loiter");
                break;
              case 6:
                Serial.println("6666666666");
                break;
              case 7:
                Serial.println("Circle");
                break;
              case 11:
                Serial.println("RTL");
                break;
              default:
                Serial.println("Mode not known");
                break;
            }
            ee = 0;
          }
          break;
       }
    }
  }
}
