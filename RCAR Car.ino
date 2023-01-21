#include <RF24.h>
#include <RF24_config.h>
#include <nRF24L01.h>
#include <printf.h>

#include <RF24.h>
#include <RF24_config.h>
#include <nRF24L01.h>
#include <printf.h>
#include <SPI.h>

#define FrontLight A0
#define RightBlinker A1
#define LeftBlinker A2
#define StopLight A3

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


int motorValue;
int steeringValue;
int servoValue;
int servoAngle;
int speed;
int motorSpeed;
int Xvalue, Yvalue;
long CrashingDistance = 5;

String drivingStatus;
float counter;
float counterdif;
bool LightsOn;
int HeadLihgt;


const byte address[6] = "00002";

void setup() {

  // Radio SETUP
  radio.begin();
  Serial.begin(9600);

  if (!radio.begin()) {
    Serial.println(F("radio hardware not responding!"));
    while (1) {}  // hold program in infinite loop to prevent subsequent errors
  }
  radio.openReadingPipe(1, address);

  radio.setPALevel(RF24_PA_MIN);
  radio.setDataRate(RF24_1MBPS);

  radio.startListening();

  pinMode(IRQ_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(IRQ_PIN), isrCallbackFunction, FALLING);

  // other Setups

  pinMode(motorpin1, OUTPUT);
  pinMode(motorpin2, OUTPUT);

  pinMode(steerpin, OUTPUT);

  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);

  //lights
  pinMode(FrontLight, OUTPUT);
  pinMode(RightBlinker, OUTPUT);
  pinMode(LeftBlinker, OUTPUT);
  pinMode(StopLight, OUTPUT);
}

void loop() {

  //creating a counter
  if (counter <= 0) {
    counterdif = +0.1;
  }
  if (counter >= 2) {
    counterdif = -0.1;
  }
  counter = counter + counterdif;


  //receiving  values from remote controller
  isrCallbackFunction();

  if (radio.available()) {

    int values[] = { Xvalue, Yvalue, HeadLihgt };


    radio.read(&values, sizeof(values));

    Serial.print("X Value :");
    Serial.print(values[0]);
    Serial.print("---------");
    Serial.print("Y Value :");
    Serial.print(values[1]);
    Serial.print("----");
    Serial.print(values[2]);

    motorValue = values[1];
    steeringValue = values[0];
    HeadLihgt = values[2];
  }






  //avoid crash
  while (Distance() < CrashingDistance) {  // 10 yerine Distance() fonksiyonu yerleştirmek mesafe sensörünü aktive eder
    Backward();
    SpeedUp(150);

    //stopLights
    if (counter > 0.5 and counter < 1.5) {
      digitalWrite(StopLight, HIGH);
    } else {
      digitalWrite(StopLight, LOW);
    }
    Serial.println("Crashing prevented!!!! GOING BACKWARDS");
  }


  //setting dc motor direction and speed
  if (motorValue <= 470) {

    motorSpeed = map(motorValue, 0, 470, 255, 0);
    Forward();
    SpeedUp(motorSpeed);

    //stoplights off
    digitalWrite(StopLight, LOW);
    drivingStatus = "Going Forward";

  } else if (motorValue >= 570) {

    motorSpeed = map(motorValue, 570, 1023, 0, 255);
    Backward();
    SpeedUp(motorSpeed);

    //stopLights
    if (counter > 0.5 and counter < 1.5) {
      digitalWrite(StopLight, HIGH);
    } else {
      digitalWrite(StopLight, LOW);
    }
    drivingStatus = "Going Backwards";

  } else {
    Stop();

    //stoplights off
    digitalWrite(StopLight, LOW);
    drivingStatus = "Stopped";
  }

  //setting servo motor angle
  Steer(steeringValue);

  //blinkers and Lihts
  if (counter > 0.5 and counter < 1.5) {
    LightsOn = true;
  } else {
    LightsOn = false;
  }

  //blinkers
  if (steeringValue < 340 and LightsOn) {
    digitalWrite(LeftBlinker, HIGH);
  } else if (steeringValue > 660 and LightsOn) {
    digitalWrite(RightBlinker, HIGH);
  } else {
    digitalWrite(RightBlinker, LOW);
    digitalWrite(LeftBlinker, LOW);
  }

  //headlight
  if (HeadLihgt == 1) {
    digitalWrite(FrontLight, HIGH);
  } else {
    digitalWrite(FrontLight, LOW);
  }

  //Serial Screen
  Serial.print("---");
  Serial.print(drivingStatus);
  Serial.print("-----");
  Serial.println(counter);
}



void Forward() {

  digitalWrite(motorpin1, HIGH);
  digitalWrite(motorpin2, LOW);
}

void Backward() {

  digitalWrite(motorpin1, LOW);
  digitalWrite(motorpin2, HIGH);
}

void SpeedUp(int speed) {
  analogWrite(motorspeedpin, speed);
}

void Stop() {
  analogWrite(motorspeedpin, 0);
}

void Steer(int steeringValue) {

  static int servolowerLimit = 1350;
  static int servoupperLimit = 1950;
  servoAngle = map(steeringValue, 0, 1023, servoupperLimit, servolowerLimit);

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
void Blinker() {
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