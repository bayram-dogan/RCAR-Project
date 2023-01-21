#include <RF24.h>
#include <RF24_config.h>
#include <nRF24L01.h>
#include <printf.h>
#include <stdio.h>
#include <SPI.h>



#define XPin A5
#define YPin A3
#define LightButton 4
#define LightPin 6
#define CE 9
#define CSN 10

RF24 radio(CE, CSN);  // CE, CSN
int Xvalue, Yvalue;
int HeadLihgt = 0;
bool currentButtonState;
bool lastbuttonState;
int values[] = { Xvalue, Yvalue, HeadLihgt };
const byte address[6] = "00002";

void setup() {
  Serial.begin(9600);
  pinMode(XPin, INPUT);
  pinMode(YPin, INPUT);
  pinMode(LightButton, INPUT_PULLUP);
  radio.begin();
  if (!radio.begin()) {
    Serial.println(F("radio hardware not responding!"));
    while (1) {}  // hold program in infinite loop to prevent subsequent errors
  }
  radio.openWritingPipe(address);
  //radio.enableDynamicPayloads();
  //radio.enableAckPayload();
  //radio.setRetries(10, 15);
  //radio.setAutoAck(true);
  radio.setPALevel(RF24_PA_MIN);
  radio.setDataRate(RF24_1MBPS);
  radio.printDetails();
  radio.stopListening();
}

void loop() {
  values[0] = smoothValueX(analogRead(XPin));
  values[1] = smoothValueY(analogRead(YPin));
  values[2] = HeadLihgt;
  currentButtonState = digitalRead(LightButton);
  if (currentButtonState == LOW and lastbuttonState) {
    if (HeadLihgt) {
      HeadLihgt = 0;
    } else {
      HeadLihgt = 1;
    }
  }
  lastbuttonState = digitalRead(LightButton);
  //led on the controller scnyed to the headlight
  if (HeadLihgt == 1) {
    digitalWrite(LightPin, HIGH);
  } else {
    digitalWrite(LightPin, LOW);
  }
  int rslt = radio.write(&values, sizeof(values));
  if (rslt) {}
  Serial.print("X Value :");
  Serial.print(values[0]);
  Serial.print("---------");
  Serial.print("Y Value :");
  Serial.print(values[1]);
  Serial.print("---------");
  Serial.print(values[2]);
  Serial.print("---------");
  Serial.println(rslt);
}
int smoothValueX(int value) {
  const static int arraySize = 3;
  static boolean isArrayFull = false;
  static int index = 0;
  static int denominator;
  static int sum = 0;
  static int average = 0;
  static int array[arraySize];
  if (isArrayFull) {
    sum -= array[index];
    denominator = arraySize;
  } else {
    denominator = index + 1;
  }
  sum += value;
  array[index] = value;
  average = sum / denominator;
  if (index < arraySize) {
    index += 1;
  }
  if (index == arraySize) {
    isArrayFull = true;
  }
  if (index >= arraySize) {
    index = 0;
  }
  return average;
}

int smoothValueY(int value) {
  const static int arraySize = 3;
  static boolean isArrayFull = false;
  static int index = 0;
  static int denominator;
  static int sum = 0;
  static int average = 0;
  static int array[arraySize];
  if (isArrayFull) {
    sum -= array[index];
    denominator = arraySize;
  } else {
    denominator = index + 1;
  }
  sum += value;
  array[index] = value;
  average = sum / denominator;
  if (index < arraySize) {
    index += 1;
  }
  if (index == arraySize) {
    isArrayFull = true;
  }
  if (index >= arraySize) {
    index = 0;
  }
  return average;
}