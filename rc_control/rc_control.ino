int waypointCH;
int saveCH;
#define ch1 8
#define ch2 9
int CH1 = 900;
int CH5 = 1450;
int chArray[9] = {900, 1000, 1200, 1300, 1450, 1750, 1800, 1900, 2020};
int svArray[2] = {900, 2020};
#define CH9 2050
int rangeCH = 70;
#define numWP 3
byte wpCH;
int wpCHRaw;


void setup() {
  pinMode(ch1, INPUT);
  pinMode(ch2, INPUT);
  Serial.begin(57600);
}

void loop() {

  waypointCH = pulseIn(ch1, HIGH, 25000);
  saveCH = pulseIn(ch2, HIGH, 25000);
  Serial.print("Channel 1:");
  Serial.print(waypointCH);
  Serial.print("  Channel 2:");
  Serial.println(saveCH);

    wpCHRaw = pulseIn(ch1, HIGH, 25000);
for (int i = 0; i < 9; i++){
    if((wpCHRaw-chArray[i])<rangeCH){
      wpCH = i+1;
      break;
    }
  }
  Serial.println(wpCH);
  int svpCHRaw = pulseIn(ch2, HIGH, 25000);
    for (int i = 0; i < 2; i++){
    if((svpCHRaw-svArray[i])<rangeCH){
      saveCH = i+1;
      break;
    }
  }
/*
  wpCHRaw = pulseIn(ch1, HIGH, 25000);

  for (int i = 0; i < 10; i++){

    if((wpCHRaw-chArray[i])<rangeCH){
      wpCH = i+1;
      Serial.print("CH 1: ");
      Serial.println(wpCH);
      break;
    }
  }
*//*  if((wpCHRaw-CH1)<rangeCH){
    wpCH = 1;
    Serial.print("CH 1: ");
    Serial.println(wpCH);
  }
  else
    if((wpCHRaw-CH5)<rangeCH){
      wpCH = 5;
      Serial.print("CH 1: ");
      Serial.println(wpCH);
    }
    else
      if((wpCHRaw-CH9)<rangeCH){
        wpCH = 9;
        Serial.print("CH 1: ");
        Serial.println(wpCH);
      }
*/
  delay(100); // I put this here just to make the terminal
  
  // window happier

}
