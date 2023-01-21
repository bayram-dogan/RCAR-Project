#include <RF24.h>
#include <RF24_config.h>
#include <nRF24L01.h>
#include <printf.h>
#include <SPI.h>

#define FrontLight A1
#define RightBlinker A0
#define LeftBlinker A5
#define StopLight A4
#define motorpin1 2
#define motorpin2 3
#define motorspeedpin 5
#define steerpin 6
#define trig 4
#define echo 7
#define CE 9
#define CSN 10
#define IRQ_PIN 8

RF24 radio(CE, CSN);  // CE, CSN
int motorValue = 520;
int steeringValue = 500;
int servoValue;
int servoAngle;
int speed;
int motorSpeed;
int Xvalue, Yvalue;
long CrashingDistance = 5;     //distance left before avoid crashing
String drivingStatus;          //
bool LightsOn;                 //flip flop led statue
int HeadLihgt = 0;             //hedlight statue received from controller
long ledinterval = 300;        // time between  leds to be on and off (seconds)
int avoidCrashingSpeed = 150;  // pwm signal so it must be between 0-255
bool canDrive = true;

const byte address[6] = "00002";

void setup() {
  //Serial.begin(9600);
  // Radio SETUP
  radio.begin();
  if (!radio.begin()) {
    Serial.println(F("radio hardware not responding!"));
    while (1) {}  // hold program in infinite loop to prevent subsequent errors
  }
  radio.openReadingPipe(1, address);
  //radio.enableDynamicPayloads();
  //radio.enableAckPayload();
  radio.setPALevel(RF24_PA_MIN);
  radio.setDataRate(RF24_1MBPS);
  radio.startListening();
  pinMode(IRQ_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(IRQ_PIN), isrCallbackFunction, FALLING);

  //motor and direction
  pinMode(motorpin1, OUTPUT);
  pinMode(motorpin2, OUTPUT);
  pinMode(steerpin, OUTPUT);

  //distance sensor
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);

  //lights
  pinMode(FrontLight, OUTPUT);
  pinMode(RightBlinker, OUTPUT);
  pinMode(LeftBlinker, OUTPUT);
  pinMode(StopLight, OUTPUT);
}

void loop() {
  //receiving  values from remote controller
  isrCallbackFunction();

  if (radio.available()) {

    int values[] = { Xvalue, Yvalue, HeadLihgt };

    radio.read(&values, sizeof(values));

    //Serial.print("X Value :");
    //Serial.print(values[0]);
    //Serial.print("---------");
    //Serial.print("Y Value :");
    //Serial.print(values[1]);
    //Serial.print("----");
    //Serial.print(values[2]);
    //Serial.println("----");

    motorValue = values[1];
    steeringValue = values[0];
    HeadLihgt = values[2];
  }

  LedStateChange();  //lights on and off betwwen defined time interval
  //AvoidCrash();          //stop and go bacwards if the crashing distance lower then defined
  Drive();               //setting dc motor direction and speed
  Steer(steeringValue);  //setting servo motor angle
  Blinkers();            //blinkers
  HeadLight();           //headlight

  //Serial Screen
  // Serial.println(drivingStatus);
}  //end of the loop


void Drive() {
  //setting dc motor direction and speed

  if (motorValue <= 470) {
    Forward();
    SpeedUp(motorSpeed);

    digitalWrite(StopLight, LOW);  //stoplights off

    drivingStatus = "Going Forward";
  }

  else if (motorValue >= 570) {
    Backward();
    SpeedUp(motorSpeed);

    if (LightsOn) {
      digitalWrite(StopLight, HIGH);  //stopLights
    } else {
      digitalWrite(StopLight, LOW);
    }
    drivingStatus = "Going Backwards";

  } else {
    Stop();
    digitalWrite(StopLight, LOW);  //stoplights off
    drivingStatus = "Stopped";
  }
}

void AvoidCrash() {
  //avoid crash
  while (Distance() < CrashingDistance) {  // 10 yerine Distance() fonksiyonu yerleştirmek mesafe sensörünü aktive eder
    //canDrive = false;
    Backward();
    SpeedUp(avoidCrashingSpeed);
    LedStateChange();
    //stopLights
    if (LightsOn) {
      digitalWrite(StopLight, HIGH);
    } else {
      digitalWrite(StopLight, LOW);
    }
    //Serial.println("Crashing prevented!!!! GOING BACKWARDS");
  }
}

void HeadLight() {
  //headlight
  if (HeadLihgt == 1) {
    digitalWrite(FrontLight, HIGH);
  } else {
    digitalWrite(FrontLight, LOW);
  }
}

void LedStateChange() {

  unsigned long nowtime = millis();
  static unsigned long previoustime;

  if (nowtime - previoustime >= ledinterval) {

    previoustime = nowtime;

    if (LightsOn == false) {

      LightsOn = true;

    } else {

      LightsOn = false;
    }
  }
}

void Blinkers() {

  if (steeringValue < 150 and LightsOn) {
    digitalWrite(RightBlinker, HIGH);
  } else if (steeringValue > 850 and LightsOn) {
    digitalWrite(LeftBlinker, HIGH);
  } else {
    digitalWrite(RightBlinker, LOW);
    digitalWrite(LeftBlinker, LOW);
  }
}

void Forward() {
  motorSpeed = map(motorValue, 0, 470, 255, 100);
  digitalWrite(motorpin1, LOW);
  digitalWrite(motorpin2, HIGH);
}

void Backward() {
  motorSpeed = map(motorValue, 570, 1023, 100, 255);
  digitalWrite(motorpin1, HIGH);
  digitalWrite(motorpin2, LOW);
}

void SpeedUp(int speed) {
  analogWrite(motorspeedpin, speed);
}

void Stop() {
  analogWrite(motorspeedpin, 0);
}

void Steer(int steeringValue) {

  static int servolowerLimit = 1400;
  static int servoupperLimit = 2000;
  servoAngle = map(steeringValue, 0, 1023, servolowerLimit, servoupperLimit);

  if (steeringValue > 490 and steeringValue < 510) {
    servoAngle = 1650;
  } else {

    //servoAngle = map(steeringValue, 0, 1023, 1850, 1350);
  }

  // servoAngle = map(steeringValue, 0, 1023, 700, 2500); (180 derece icin)
  // or 70 vs 130

  digitalWrite(steerpin, HIGH);
  delayMicroseconds(servoAngle);

  digitalWrite(steerpin, LOW);

  delayMicroseconds(servoupperLimit - servoAngle);

  delayMicroseconds(3000 - servoupperLimit);

  delay(17);
}

long Distance() {

  static long pulsetime;
  static long distance;

  digitalWrite(trig, LOW);

  delayMicroseconds(2);

  digitalWrite(trig, HIGH);

  delayMicroseconds(10);

  pulsetime = pulseIn(echo, HIGH);

  distance = pulsetime * 0.01724;  // distance constant
  return distance;
}

void isrCallbackFunction() {
  bool tx_ds, tx_df, rx_dr;
  radio.whatHappened(tx_ds, tx_df, rx_dr);  // resets the IRQ pin to HIGH radio.available(); // returned data should now be reliable
}