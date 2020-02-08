#define ledPin  LED_BUILTIN
int ledState = LOW;
unsigned long ledMillis;        // will store last time LED was updated
long OnTime = 500;           // milliseconds of on-time
long OffTime = 100;          // milliseconds of off-time

void setup()
{
  Serial.begin( 57600 );
  // set the digital pin as output:
  pinMode(ledPin, OUTPUT);  
  pinMode(4, OUTPUT); 
  
  Serial.println(ledPin);
}

void loop()
{
  // check to see if it's time to change the state of the LED
doDigital(ledPin,5);
doDigital(4,2);
delay(5000); 
}

void doDigital(byte pin,byte num){
  for (int i = 0; i < num; i++){
    ledMillis = millis();
    while((millis() - ledMillis < OffTime)){
      digitalWrite(pin, HIGH);
    }
    ledMillis = millis();
    while((millis() - ledMillis < OnTime)){
      digitalWrite(pin, LOW);
    }
  }
}
