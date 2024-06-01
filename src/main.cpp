#include <Arduino.h>
#include <ArduinoJson.h> // Include the ArduinoJson library
#include <BluetoothSerial.h>
#include <cmath>
#include <map>

BluetoothSerial SerialBT;

//LED
const int pinLEDL = 18;
const int pinLEDLR = 27;
const int pinLEDLG = 26;
const int pinLEDLB = 25;

const int pinLEDR = 19;
const int pinLEDRR = 5;
const int pinLEDRG = 17;
const int pinLEDRB = 16;
int ledState = HIGH;
#define BLINK_INTERVAL 1000  // interval at which to blink LED (milliseconds)
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
const int leftRedPin = 25;
const int leftGreenPin = 26;
const int leftBluePin = 33;

//Vorne Rechts
const int rightRedPin = 12;
const int rightGreenPin = 14;
const int rightBluePin = 27;

//Hinten Links
const int leftLightPin = 35;

//Hinten Rechts
const int rightLightPin = 32;

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
    String blinkVal = getValue(receivedString, ';', 2);

    float steeringValFloat = (std::stof(steeringVal.c_str())) * 100;

    float throttleValFloat = (std::stof(throttleVal.c_str())) * 100;

    //LED
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

    delay(100);
  }

  // Bluetooth not connected
  else {
    ledcWrite(servoPinPWMChannel, 297);
    ledcWrite(driveMotorPWMChannel, 0);
    digitalWrite(driverMotorPin1, LOW);
    digitalWrite(driveMotorPin2, LOW);
  }
}
