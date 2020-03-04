int waypointCH;
int saveCH;
#define ch1 8
#define ch2 12
int CH1 = 900;
int CH5 = 1450;
int chArray[3] = {1100, 1450, 2020};
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
  int wpCH2;
  int wpCH3;
  wpCHRaw = pulseIn(ch1, HIGH, 25000);
  for (int i = 0; i < 3; i++){
    if((wpCHRaw-chArray[i])<rangeCH){
      wpCH = i+1;
      Serial.print("1: "); Serial.println(wpCH);
      break;
    }
  }

  wpCHRaw = pulseIn(ch2, HIGH, 25000);
  for (int i = 0; i < 3; i++){
    if((wpCHRaw-chArray[i])<rangeCH){
      wpCH2 = i+1;
      Serial.print("2: "); Serial.println(wpCH2);
      break;
    }
  }
  switch (wpCH){
    case 1:
      switch (wpCH2){
        case 1: wpCH3 = 1;
        break;
        case 2: wpCH3 = 2;
        break;
        case 3: wpCH3 = 3;
        break;
      }
      break;
    case 2:
    switch (wpCH2){
      case 1: wpCH3 = 4;
      break;
      case 2: wpCH3 = 5;
      break;
      case 3: wpCH3 = 6;
      break;
    }
      break;
    case 3:
    switch (wpCH2){
      case 1: wpCH3 = 7;
      break;
      case 2: wpCH3 = 8;
      break;
      case 3: wpCH3 = 9;
      break;
    }
    break;
  }
  Serial.print("3: "); Serial.println(wpCH3);

  delay(300); // I put this here just to make the terminal
  


}
