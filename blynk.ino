#include "Arduino.h"
#define BLYNK_USE_DIRECT_CONNECT
#include "DHT.h"
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_INA219.h>

// You could use a spare Hardware Serial on boards that have it (like Mega)
#include <SoftwareSerial.h>
SoftwareSerial DebugSerial(11, 12); // RX, TX
#define DHT_PIN_DATA  2
#define BLYNK_PRINT DebugSerial

/* Fill-in your Template ID (only if using Blynk.Cloud) */
//#define BLYNK_TEMPLATE_ID   "YourTemplateID"
#include <BlynkSimpleSerialBLE.h>

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = "C2g7s3--3cF-40PrMx0z6t3e0cBpM9kQ";
DHT dht(DHT_PIN_DATA);
Adafruit_INA219 ina219;
Servo myservo;

unsigned long MOVING_TIME = 1000; // moving time is 1 seconds
unsigned long moveStartTime;

int FPin = 5; //Fan Pin
int LPin = 3; //Led Pin

String voice;
String readStr;

void setup()
{
  // Debug console
  DebugSerial.begin(9600);

  myservo.attach(4);//Door Pin

  // DebugSerial.println("Waiting for connections...");
  dht.begin();
  uint32_t currentFrequency;
  ina219.begin();
  
    
  /* // Blynk will work through Serial
  // 9600 is for HC-06. For HC-05 default speed is 38400
  // Do not read or write this serial manually in your sketch*/
  Serial.begin(9600);
  Blynk.begin(Serial, auth);
}

void loop()
{
  while (Serial.available()){
    delay(2000);
    char c = Serial.read();
    voice += c;
    char cd = Serial.read();
    readStr += cd;
  }   
  
  float dhtTempC = dht.readTempC();
  float shuntvoltage = 0;
  float busvoltage = 0;
  float current_mA = 0;
  float loadvoltage = 0;
  float power = 0;

  shuntvoltage = ina219.getShuntVoltage_mV();
  busvoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  loadvoltage = busvoltage + (shuntvoltage / 1000);
  power = current_mA * loadvoltage;

  DebugSerial.print(dhtTempC);
  DebugSerial.print(power);

  delay(1000);
  
  if((voice.length() > 0) or (readStr.length() > 0)){
    Serial.println(voice);
    Serial.println(readStr);
    if(voice == "turn on fan" or readStr == "fan on"){
      digitalWrite(FPin,HIGH);
    }
    else if(voice == "turn off fan" or readStr == "fan on"){
      digitalWrite(FPin,LOW);
    }
    else if(voice == "turn on light" or readStr == "light on"){
      digitalWrite(LPin,HIGH);
    }
    else if(voice == "turn off light" or readStr == "light off"){
      digitalWrite(LPin,LOW);
    }
    else if(voice == "open the door" or readStr == "open door"){
       myservo.write(90);
    }
    else if(voice == "close the door" or readStr == "close door"){
       myservo.write(0);
    }
    voice = "";
    readStr = "";
  }
  Blynk.run();
}
