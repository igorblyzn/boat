#include <mavlink.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>

/* digital OUTPUT */

#define buzzer 4
#define led LED_BUILTIN
unsigned long digitalMillis;
#define OnTime 500
#define OffTime 100  

/* digital OUTPUT */

#define RXpin 2
#define TXpin 3
SoftwareSerial Serial1(RXpin, TXpin);

byte temp = 0;
byte wpCH = 1;
byte saveCH = 0;
int wpCHRaw;
int svpCHRaw;

#define ch1 8
#define ch2 9

byte startCH1 = 0;
byte startCH2 = 0;

float x = 0.0;
float y;
int z;

int chArray[9] = {900, 1000, 1200, 1300, 1450, 1750, 1800, 1900, 2020};
int svArray[2] = {900, 2020};
#define rangeCH 70

struct MyObject{
  float xyz[9][3];
};
MyObject eeprom;

#define _system_id  255 // system id of sending station. 255 is Ground control software
#define _component_id  2 // component id of sending station 2 works fine
#define _target_system  1 // Pixhawk id
#define _target_component 0 // Pixhawk component id, 0 = all (seems to work fine)
#define frame 0 // Set target frame to global default
#define command MAV_CMD_NAV_WAYPOINT // Specific command for PX4
#define current 0 // Guided mode waypoint
#define autocontinue 0 // Always 0
#define count 2

unsigned long tempPrevMillis = 0;

bool modeHold = false;
bool modeAuto = false;
unsigned long tempModePrevMillis = 0;
byte ee = 1;

void setup() {
  pinMode(ch1, INPUT);
  pinMode(ch2, INPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(led, OUTPUT);
  Serial1.begin(57600); //RXTX from Pixhawk (Port 19,18 Arduino Mega)
  Serial.begin(57600); //Main serial port for console output
  
  EEPROM.get(0, eeprom);
  mission_count();

  while (x == 0.0){
    unsigned long tempMillis = millis();
    if((tempMillis - tempPrevMillis) >= 500){
      tempPrevMillis = tempMillis;  // Remember the time
      Serial.print(".");
      getCoordinates();
      blink(1);
    }
  }
  blink(2);
  beep(2);
  getMode();
}

void loop() {
  unsigned long tempMillis = millis();
  unsigned long tempModeMillis = millis();
  if((tempMillis - tempPrevMillis) > 400){
    tempPrevMillis = tempMillis;
    blink(2);
    getSwState();
    if (wpCH != startCH1) {
      EEPROM.get( 0, eeprom );
      writeWP();
      startCH1 = wpCH;
      blink(1);
      beep(1);
      printAllPoints();
    }
    if (saveCH > 1) {
      Serial.println("Get");
      EEPROM.get( 0, eeprom );
      getCoordinates();
      saveWaipoint();
      writeWP();
      blink(2);
      beep(2);
    }
  }
    if((tempModeMillis - tempModePrevMillis) > 5000){
    tempModePrevMillis = tempModeMillis;
    while(ee==1){
      getMode();
    }
    ee=1;
    Serial.print(modeHold);Serial.println("  ++++++++++++");
  }
}

void writeWP() {
  mission_count();
  while (temp < 1) {
    pushWpAndCheck();
  }
  temp = 0;
}

void getCoordinates() {

  mavlink_message_t msg;
  mavlink_status_t status;
  while (Serial1.available()) {
    uint8_t c = Serial1.read();
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      switch (msg.msgid) {
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
            mavlink_global_position_int_t packet;
            mavlink_msg_global_position_int_decode(&msg, &packet);
            x = packet.lat / 10000000.0f;
            y = packet.lon / 10000000.0f;
            z = packet.alt / 1000.0f;
          }
          break;
      }
    }
  }
}

void pushWpAndCheck(){

  mavlink_message_t msg;
  mavlink_status_t status;
  while (Serial1.available()) {
    uint8_t c = Serial1.read();
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      switch (msg.msgid) {
        case MAVLINK_MSG_ID_MISSION_REQUEST: {
            mavlink_mission_request_t missionreq;
            mavlink_msg_mission_request_decode(&msg, &missionreq);
            if (missionreq.seq == 0) {
              create_home();
            }
            if (missionreq.seq == 1) {
              create_waypoint();
            }
          }
          break;
        case MAVLINK_MSG_ID_MISSION_ACK: {
            mavlink_mission_ack_t missionack;
            mavlink_msg_mission_ack_decode(&msg, &missionack);
            if (missionack.type == 0) {
              Serial.println("Upload OK");
              temp = 1;
            }
          }
          break;
      }
    }
  }

}

void create_home() {
  uint16_t seq = 0; // Sequence number
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_mission_item_pack(_system_id, _component_id, &msg, _target_system, _target_component, seq, frame, command, current, autocontinue, 0, 0, 0, 0, eeprom.xyz[0][0], eeprom.xyz[0][1], eeprom.xyz[0][2]);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial1.write(buf, len);
}

void create_waypoint() {
  uint16_t seq = 1; // Sequence number
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_mission_item_pack(_system_id, _component_id, &msg, _target_system, _target_component, seq, frame, command, current, autocontinue, 0, 0, 0, 0, eeprom.xyz[wpCH][0], eeprom.xyz[wpCH][1], eeprom.xyz[wpCH][2]);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial1.write(buf, len);
}

void mission_count() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_mission_count_pack(_system_id, _component_id, &msg, _target_system, _target_component, count);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial1.write(buf, len);
}

void getSwState() {

  noInterrupts();
  wpCHRaw = pulseIn(ch1, HIGH, 25000);
  svpCHRaw = pulseIn(ch2, HIGH, 25000);
  interrupts();

  for (int i = 0; i < 2; i++) {
    if ((svpCHRaw - svArray[i]) < rangeCH) {
      saveCH = i + 1;
      break;
    }
  }

  for (int i = 0; i < 9; i++) {
    if ((wpCHRaw - chArray[i]) < rangeCH) {
      wpCH = i + 1;
      break;
    }
  }
  Serial.println(wpCH);
}

void saveWaipoint() {
  eeprom.xyz[wpCH - 1][0] = x;
  eeprom.xyz[wpCH - 1][1] = y;
  eeprom.xyz[wpCH - 1][2] = z;
  EEPROM.put(0, eeprom);
}

void printAllPoints() {
  for (int i = 0; i < 9; i++) {
    for (int j = 0; j < 3; j++) {
      Serial.print(eeprom.xyz[i][j]); Serial.print(" ");
    }
    Serial.println();
  }
}

void beep(byte num){
  doDigital(buzzer,num);
}

void blink(byte num){
  doDigital(led,num);
}

void doDigital(byte pin,byte num){
  for (int i = 0; i < num; i++){
    digitalMillis = millis();
    while((millis() - digitalMillis < OffTime)){
      digitalWrite(pin, HIGH);
    }
    digitalMillis = millis();
    while((millis() - digitalMillis < OnTime)){
      digitalWrite(pin, LOW);
    }
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
            switch(hb.custom_mode) {
              case 0:{
                //Serial.println("Stab");
                modeHold = false;
                modeAuto = false;
              }
              break;
              case 4:{
                //Serial.println("Hold");
                if(modeAuto)
                  beep(10);
              }
              break;
              case 10:
                //Serial.println("Auto");
                modeAuto = true;
              break;
              default:{
                modeHold = false;
                modeAuto = false;
              }
              break;
            }
            ee = 0;
          }
          break;
       }
    }
  }
}
