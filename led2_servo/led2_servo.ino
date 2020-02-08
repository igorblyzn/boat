unsigned long blinkStartMillis;
unsigned long fadeStartMillis;
unsigned long currentMillis;
const unsigned long blinkPeriod = 1000;  //blink period
const unsigned long fadePeriod = 10;  //fade period
const byte blinkLedPin = 12;    //this LED will blink
const byte fadeLedPin = 10;    //this LED will fade
byte brightness = 0;  //initial brightness of LED
byte increment = 1;  //amount to change PWM value at each change

void setup()
{
  Serial.begin(57600);  //start Serial in case we need to print debugging info
  pinMode(blinkLedPin, OUTPUT);

}

void loop()
{
  currentMillis = millis();  //get the current time
  analogWrite(fadeLedPin, 254);
  //fade();
}


void fade()    //function to fade an LED
{
  if (currentMillis - fadeStartMillis >= fadePeriod)  //test whether the period has elapsed
  {
    analogWrite(fadeLedPin, brightness);    //set the brightness of the LED
    brightness += increment;    //will wrap round because brightness is an unsigned data type
    fadeStartMillis = currentMillis;  //IMPORTANT to save the start time of the current LED state.
    Serial.println(brightness);
  }
}
