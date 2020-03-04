#include <mavlink.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>

/* digital OUTPUT */
unsigned long digitalMillis;
#define OnTime 500
#define OffTime 200  
/* digital OUTPUT */

#define RXpin 2
#define TXpin 3
#define buzzer 5
#define ch1 8//WP1
#define ch4 12//WP2
#define ch2 9//Save
#define ch3 10//lights
#define led 11//LED_BUILTIN
SoftwareSerial Serial1(RXpin, TXpin);

byte temp = 0;
byte wpCH = 1;
byte saveCH = 0;

byte startCH1 = 0;
byte startCH2 = 0;

float x = 0.0;
float y;
int z;

int chArray[3] = {1100, 1450, 2020};

#define rangeCH 70

struct MyObject{
  float xa[9];
  float ya[9];
  int za[9];
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
unsigned long tempMillis;
unsigned long tempModeMillis;
bool modeAuto = false;
unsigned long tempModePrevMillis = 0;
unsigned long ledMillis;

void setup() {
  delay(15000);
  pinMode(ch1, INPUT);
  pinMode(ch2, INPUT);
  pinMode(ch3, INPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(led, OUTPUT);
  beep(1);
  Serial1.begin(57600);
  EEPROM.get(0, eeprom);
  
  mission_count();
  while (x == 0.0){
    tempMillis = millis();
    if((tempMillis - tempPrevMillis) >= 800){
      getSwState();
      tempPrevMillis = tempMillis;
      getCoordinates();
    }
  }
  beep(1);
  delay(15000);
  x=0.0;
  mission_count();
  while (x == 0.0){
    tempMillis = millis();
    if((tempMillis - tempPrevMillis) >= 800){
      getSwState();
      tempPrevMillis = tempMillis;
      getCoordinates();
    }
  }
  beep(2);
  
}

void loop() {
  
  tempMillis = millis();
  if((tempMillis - tempPrevMillis) > 800){
    tempPrevMillis = tempMillis;
    getSwState();
    if (wpCH != startCH1) {
      EEPROM.get( 0, eeprom );
      writeWP();
      startCH1 = wpCH;
      beep(1);
    }
    if (saveCH > 0) {
      EEPROM.get( 0, eeprom );
      getCoordinates();
      saveWaipoint();
      writeWP();
      beep(2);
    }
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
              create_WP(0,eeprom.xa[0], eeprom.ya[0], eeprom.za[0]);
            }
            if (missionreq.seq == 1) {
              create_WP(1,eeprom.xa[wpCH], eeprom.ya[wpCH], eeprom.za[wpCH]);
            }
          }
          break;
        case MAVLINK_MSG_ID_MISSION_ACK: {
            mavlink_mission_ack_t missionack;
            mavlink_msg_mission_ack_decode(&msg, &missionack);
            if (missionack.type == 0) {
              temp = 1;
            }
          }
          break;
      }
    }
  }

}

void create_WP(byte seq, float x, float y, int z) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_mission_item_pack(_system_id, _component_id, &msg, _target_system, _target_component, seq, frame, command, current, autocontinue, 0, 0, 0, 0, x, y, z);
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
  int wpCHRaw = pulseIn(ch1, HIGH, 25000);
  int svpCHRaw = pulseIn(ch2, HIGH, 25000);
  int ledCHRaw = pulseIn(ch3, HIGH, 25000);
  int wpCHRaw2 = pulseIn(ch4, HIGH, 25000);
  interrupts();

  if(svpCHRaw>1600){
    saveCH = 1;
  }
  else saveCH = 0;
   
  int wpCH2;

  for (byte i = 0; i < 3; i++) {
    if ((wpCHRaw - chArray[i]) < rangeCH) {
      wpCH = i;
      break;
    }
  }

  for (byte i = 0; i < 3; i++) {
    if ((wpCHRaw2 - chArray[i]) < rangeCH) {
      wpCH2 = i;
      break;
    }
  }

  switch (wpCH){
    case 0:
      switch (wpCH2){
        case 0: wpCH = 0;
        break;
        case 1: wpCH = 1;
        break;
        case 2: wpCH = 2;
        break;
      }
      break;
    case 1:
      switch (wpCH2){
        case 0: wpCH = 3;
        break;
        case 1: wpCH = 4;
        break;
        case 2: wpCH = 5;
        break;
      }
      break;
    case 2:
      switch (wpCH2){
        case 0: wpCH = 6;
        break;
        case 1: wpCH = 7;
        break;
        case 2: wpCH = 8;
        break;
      }
      break;
  }

  if(ledCHRaw>1200){
    analogWrite(led, 250);
  }
  else if(ledCHRaw<1000){
    analogWrite(led, 0);
  }
  


}

void saveWaipoint() {
  eeprom.xa[wpCH] = x;
  eeprom.ya[wpCH] = y;
  eeprom.za[wpCH] = z;
  EEPROM.put(0, eeprom);
}

void beep(byte num){
  
  for (byte i = 0; i < num; i++){
    digitalMillis = millis();
    while((millis() - digitalMillis < OffTime)){
      digitalWrite(buzzer, HIGH);
      analogWrite(led, 254);
    }
    digitalMillis = millis();
    while((millis() - digitalMillis < OnTime)){
      digitalWrite(buzzer, LOW);
      analogWrite(led, 0);
    }
  }
}
