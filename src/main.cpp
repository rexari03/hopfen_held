#include <Arduino.h>
#include <ArduinoJson.h> // Include the ArduinoJson library
#include <BluetoothSerial.h>
#include <cmath>
#include <map>
#include <Ultrasonic.h>

BluetoothSerial SerialBT;
int ledState = HIGH;
#define BLINK_INTERVAL 500  // interval at which to blink LED (milliseconds)
unsigned long previousMillis = 0;   


// Drive Motor
const int driverMotorPin1 = 17; 
const int driveMotorPin2 = 16; 
const int driveMotorEnablePin = 4;
const int driveMotorPWMChannel = 0;

// Water Motor
const int waterMotorPin1 = 5;
const int waterMotorPin2 = 18;
const int waterMotorEnablePin = 19;
const int waterMotorPWMChannel = 1;

//Vorne Links
const int pinLEDLR = 25;
const int pinLEDLG = 26;
const int pinLEDLB = 33;

//Vorne Rechts
const int pinLEDRR = 12;
const int pinLEDRG = 14;
const int pinLEDRB = 27;

//Hinten Links
const int pinLEDL = 13;

//Hinten Rechts
const int pinLEDR = 32;

// Ultraschallsensor 
const int triggerPin = 15;
const int echoPin = 23;
Ultrasonic ultrasonic(triggerPin, echoPin);

// Steering Servo
const int servoPin = 21;
const int servoPinPWMChannel = 2;

// Leiter Servo
const int ladderPin = 2;
const int ladderPWMChannel = 3;

// Setting PWM properties
const int freq = 3000;
const int servoFreq = 50;
const int resolution = 8;
int dutyCycle = 200;
const int servoRes = 12;


bool reverse = false;

// Extracts values from string
String getValue(String data, char separator, int index)
{
    int found = 0;
    int strIndex[] = { 0, -1 };
    int maxIndex = data.length() - 1;

    for (int i = 0; i <= maxIndex && found <= index; i++) {
        if (data.charAt(i) == separator || i == maxIndex) {
            found++;
            strIndex[0] = strIndex[1] + 1;
            strIndex[1] = (i == maxIndex) ? i+1 : i;
        }
    }
    return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

void blink(int LED_PIN, int LED_PIN_R, int LED_PIN_G, int LED_PIN_B, long currentMillis){
  if (currentMillis - previousMillis >= BLINK_INTERVAL) {
    // if the LED is off turn it on and vice-versa:
    ledState = (ledState == LOW) ? HIGH : LOW;

    // set the LED with the ledState of the variable:
    digitalWrite(LED_PIN, ledState);
    digitalWrite(LED_PIN_R, ledState);
    digitalWrite(LED_PIN_G, ledState);
    digitalWrite(LED_PIN_B, LOW);

    // save the last time you blinked the LED
    previousMillis = currentMillis;
  }
}

void warnblinck(long currentMillis){
  if (currentMillis - previousMillis >= BLINK_INTERVAL) {
    // if the LED is off turn it on and vice-versa:
    ledState = (ledState == LOW) ? HIGH : LOW;

    // set the LED with the ledState of the variable:
    digitalWrite(pinLEDR, ledState);
    digitalWrite(pinLEDL, ledState);
    digitalWrite(pinLEDLR, ledState);
    digitalWrite(pinLEDLG, ledState);
    digitalWrite(pinLEDLB, LOW);
    digitalWrite(pinLEDRR, ledState);
    digitalWrite(pinLEDRG, ledState);
    digitalWrite(pinLEDRB, LOW);

    // save the last time you blinked the LED
    previousMillis = currentMillis;
  }
}


void setup() {
  //LED
  pinMode(pinLEDL, OUTPUT); 
  pinMode(pinLEDLR, OUTPUT); 
  pinMode(pinLEDLG, OUTPUT); 
  pinMode(pinLEDLB, OUTPUT); 
  pinMode(pinLEDR, OUTPUT);
  pinMode(pinLEDRR, OUTPUT); 
  pinMode(pinLEDRG, OUTPUT); 
  pinMode(pinLEDRB, OUTPUT); 
  digitalWrite(pinLEDL, HIGH);
  digitalWrite(pinLEDLR, HIGH);
  digitalWrite(pinLEDLG, HIGH);
  digitalWrite(pinLEDLB, HIGH);
  digitalWrite(pinLEDR, HIGH);
  digitalWrite(pinLEDRR, HIGH);
  digitalWrite(pinLEDRG, HIGH);
  digitalWrite(pinLEDRB, HIGH);

  // Ultraschallsensor 
  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Pin setup drivemotor
  pinMode(driverMotorPin1, OUTPUT);
  pinMode(driveMotorPin2, OUTPUT);
  pinMode(driveMotorEnablePin, OUTPUT);

  // Pin setup watermotor
  pinMode(waterMotorPin1, OUTPUT);
  pinMode(waterMotorPin2, OUTPUT);
  pinMode(waterMotorEnablePin, OUTPUT);

  // Pin setup servo
  pinMode(servoPin, OUTPUT);

  // Drivemotor PWM setup
  ledcSetup(driveMotorPWMChannel, freq, resolution);
  ledcAttachPin(driveMotorEnablePin, driveMotorPWMChannel);

  // Watermotor PWM setup
  ledcSetup(waterMotorPWMChannel, freq, resolution);
  ledcAttachPin(waterMotorEnablePin, waterMotorPWMChannel);

  // Servo PWM setup
  ledcSetup(servoPinPWMChannel, servoFreq, servoRes);
  ledcAttachPin(servoPin, servoPinPWMChannel);

  // Ladder PWM setup
  ledcSetup(ladderPWMChannel, servoFreq, servoRes);
  ledcAttachPin(ladderPin, ladderPWMChannel);

  Serial.begin(9600);

  // Bluetooth setup
  SerialBT.begin("ESP32");

  // testing
  Serial.println("Testing DC Motor...");
}

void loop() {
  // Bluetooth connected
  if (SerialBT.available()) {
    String receivedString = SerialBT.readStringUntil('\n');

    String steeringVal = getValue(receivedString, ';', 0);
    String throttleVal = getValue(receivedString, ';', 1);
    String blinkVal = getValue(receivedString, ';', 4);
    String autonomousVal = getValue(receivedString, ';', 6);
    String ladderVal = getValue(receivedString, ';', 2);
    String pumpVal = getValue(receivedString, ';', 3);

    float steeringValFloat = (std::stof(steeringVal.c_str())) * 100;

    float throttleValFloat = (std::stof(throttleVal.c_str())) * 100;

    float autonomous = (std::stof(autonomousVal.c_str()));

    float blinkValFloat = (std::stof(blinkVal.c_str()));
    unsigned long currentMillis = millis();

    if(blinkValFloat == 1){
      blink(pinLEDL, pinLEDLR, pinLEDLG, pinLEDLB, currentMillis);
      digitalWrite(pinLEDR, HIGH);
      digitalWrite(pinLEDRR, HIGH);
      digitalWrite(pinLEDRG, HIGH);
      digitalWrite(pinLEDRB, HIGH);
    } else if(blinkValFloat == 2){
      blink(pinLEDR,pinLEDRR, pinLEDRG, pinLEDRB, currentMillis);
      digitalWrite(pinLEDL, HIGH);
      digitalWrite(pinLEDLR, HIGH);
      digitalWrite(pinLEDLG, HIGH);
      digitalWrite(pinLEDLB, HIGH);
    }else if(blinkValFloat == 3){
      warnblinck(currentMillis);
    }else{
      digitalWrite(pinLEDL, HIGH);
      digitalWrite(pinLEDLR, HIGH);
      digitalWrite(pinLEDLG, HIGH);
      digitalWrite(pinLEDLB, HIGH);
      digitalWrite(pinLEDR, HIGH);
      digitalWrite(pinLEDRR, HIGH);
      digitalWrite(pinLEDRG, HIGH);
      digitalWrite(pinLEDRB, HIGH);
      ledState = HIGH;
    }

    if (steeringValFloat == 0 && throttleValFloat == 0){
      float ladderValFloat = (std::stof(ladderVal.c_str())) * 100;
      float pumpValFloat = (std::stof(pumpVal.c_str()));
      int ladderPulseWidth = map(ladderValFloat, 0, 100.0, 190, 310);

      ledcWrite(ladderPWMChannel, ladderPulseWidth);
      ledcWrite(servoPinPWMChannel, 297);
      ledcWrite(driveMotorPWMChannel, 0);
      digitalWrite(driverMotorPin1, LOW);
      digitalWrite(driveMotorPin2, LOW);

      if (pumpValFloat == 1) {
        digitalWrite(waterMotorPin1, HIGH);
        digitalWrite(waterMotorPin2, LOW);
        ledcWrite(waterMotorPWMChannel, 200);
      } else {
        ledcWrite(waterMotorPWMChannel, 0);
        digitalWrite(waterMotorPin1, LOW);
        digitalWrite(waterMotorPin2, LOW);
      }

      if (autonomous == 1) {
        digitalWrite(pinLEDL, HIGH);
        digitalWrite(pinLEDLR, HIGH);
        digitalWrite(pinLEDLG, LOW);
        digitalWrite(pinLEDLB, LOW);
        digitalWrite(pinLEDR, HIGH);
        digitalWrite(pinLEDRR, HIGH);
        digitalWrite(pinLEDRG, LOW);
        digitalWrite(pinLEDRB, LOW);
        
        int distanz = ultrasonic.read();
        if (distanz <= 10) { 
          digitalWrite(driverMotorPin1, LOW);
          digitalWrite(driveMotorPin2, HIGH);
        }
        else {
          digitalWrite(driverMotorPin1, HIGH);
          digitalWrite(driveMotorPin2, LOW);
        }

        Serial.println(distanz);
        if(distanz<=40){
          reverse = false;
          ledcWrite(servoPinPWMChannel, 492);
        }else if(distanz>40){
          reverse = false;
          ledcWrite(servoPinPWMChannel, 297);
        }else if(distanz<=10){
          ledcWrite(servoPinPWMChannel, 102);
        }
        ledcWrite(driveMotorPWMChannel, 140); 
      }
    } else {
      if(autonomous == 0){
        // Changes motor direction
        if (throttleValFloat < 0) {
          digitalWrite(driverMotorPin1, HIGH);
          digitalWrite(driveMotorPin2, LOW);
        }
        else {
          digitalWrite(driverMotorPin1, LOW);
          digitalWrite(driveMotorPin2, HIGH);
        }

        float absThrottleValue = std::abs(throttleValFloat);

        int servoPulseWidth = map(steeringValFloat, -100.0, 100.0, 102, 492);
        int throttlePulseWidth = map(absThrottleValue, 0, 100, 60, 200);

        if(absThrottleValue == 0) {
          throttlePulseWidth = 0;
        }

        // Sets different pwm signals
        ledcWrite(servoPinPWMChannel, servoPulseWidth);
        ledcWrite(driveMotorPWMChannel, throttlePulseWidth);
      }
    }
    delay(50);
    SerialBT.println("OK");
  } 
}
