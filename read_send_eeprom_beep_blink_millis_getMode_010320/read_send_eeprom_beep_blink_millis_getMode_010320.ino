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
#define ch1 8
#define ch2 9
#define ch3 10
#define led 11//LED_BUILTIN
SoftwareSerial Serial1(RXpin, TXpin);

byte temp = 0;
byte wpCH = 1;
byte saveCH = 0;
int wpCHRaw;
int svpCHRaw;
int ledCHRaw;



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
unsigned long tempMillis;
unsigned long tempModeMillis;
bool modeAuto = false;
unsigned long tempModePrevMillis = 0;
unsigned long ledMillis;

byte sats = 0;

void setup() {
  delay(15000);
  beep(1);
  pinMode(ch1, INPUT);
  pinMode(ch2, INPUT);
  pinMode(ch3, INPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(led, OUTPUT);
  Serial1.begin(57600);
  Serial.begin(57600);
  
  EEPROM.get(0, eeprom);
  
  printAllPoints();
  
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
      Serial.print("put ");
      EEPROM.get( 0, eeprom );
      writeWP();
      startCH1 = wpCH;
      beep(1);
    }
    if (saveCH > 1) {
      Serial.print("save ");
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
              create_WP(0,eeprom.xyz[0][0], eeprom.xyz[0][1], eeprom.xyz[0][2]);
            }
            if (missionreq.seq == 1) {
              create_WP(1,eeprom.xyz[wpCH-1][0], eeprom.xyz[wpCH-1][1], eeprom.xyz[wpCH-1][2]);
            }
          }
          break;
        case MAVLINK_MSG_ID_MISSION_ACK: {
            mavlink_mission_ack_t missionack;
            mavlink_msg_mission_ack_decode(&msg, &missionack);
            if (missionack.type == 0) {
              Serial.println("OK");
              temp = 1;
            }
          }
          break;
      }
    }
  }

}

void create_WP(byte seq, float x1, float y1, int z1) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_mission_item_pack(_system_id, _component_id, &msg, _target_system, _target_component, seq, frame, command, current, autocontinue, 0, 0, 0, 0, x1, y1, z1);
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
  ledCHRaw = pulseIn(ch3, HIGH, 25000);
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

  if(ledCHRaw>1200){
    analogWrite(led, 250);
  }
  else if(ledCHRaw<1000){
    analogWrite(led, 0);
  }
  
  Serial.print(wpCH);
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
      Serial.print(eeprom.xyz[i][j],8); Serial.print(" ");
    }
    Serial.println();
  }
}

void beep(byte num){
  
  for (int i = 0; i < num; i++){
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

void blink(byte num){
  doDigital(led,num);
}

void doDigital(byte pin,byte num){
  for (int i = 0; i < num; i++){
    digitalMillis = millis();
    while((millis() - digitalMillis < OffTime)){
      analogWrite(led, 250);
    }
    digitalMillis = millis();
    while((millis() - digitalMillis < OnTime)){
      analogWrite(led, 0);
    }
  }
}
