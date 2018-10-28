#include "Arduino.h"
#include "NewPing.h"

const int motorRightPWM = 11;
const int motorRightA = 5;
const int motorRightB = 4;

const int motorLeftPWM = 10;
const int motorLeftA = 3;
const int motorLeftB = 2;

const int frontTrig = 7;
const int frontEcho = 6;

const int leftTrig = 9;
const int leftEcho = 8;

const int rightTrig = 13;
const int rightEcho = 12;

const int MAX_DISTANCE = 200;

// 1 - Forwards, -1 - Backwards, 0 - Stopped
int leftDirection = 0;
int rightDirection = 0;

NewPing frontSonar(frontTrig, frontEcho, MAX_DISTANCE);
NewPing leftSonar(leftTrig, leftEcho, MAX_DISTANCE);
NewPing rightSonar(rightTrig, rightEcho, MAX_DISTANCE);

void setup() {
  Serial.begin(9600);

  while (!Serial) {;} // Wait for serial to connect

  Serial.println("Awake!");

  pinMode(motorLeftPWM, OUTPUT);
  pinMode(motorLeftA, OUTPUT);
  pinMode(motorLeftB, OUTPUT);

  pinMode(motorRightPWM, OUTPUT);
  pinMode(motorRightA, OUTPUT);
  pinMode(motorRightB, OUTPUT);

  digitalWrite(motorLeftA, LOW);
  digitalWrite(motorLeftB, HIGH);
  digitalWrite(motorRightA, LOW);
  digitalWrite(motorRightB, HIGH);
}

// -255 to 255
void setLeft(int s) {
  if (s > 0) {
    digitalWrite(motorLeftA, LOW);
    digitalWrite(motorLeftB, HIGH);
  } else if (s < 0) {
    digitalWrite(motorLeftA, HIGH);
    digitalWrite(motorLeftB, LOW);
  } else {
    digitalWrite(motorLeftA, LOW);
    digitalWrite(motorLeftB, LOW);
  }

  s = abs(s);
  //Serial.print("Left: " + String(s));

  analogWrite(motorLeftPWM, s);
}

void setRight(int s) {
  if (s > 0) {
    digitalWrite(motorRightA, LOW);
    digitalWrite(motorRightB, HIGH);
  } else if (s < 0)  {
    digitalWrite(motorRightA, HIGH);
    digitalWrite(motorRightB, LOW);
  } else {
    digitalWrite(motorRightA, LOW);
    digitalWrite(motorRightB, LOW);
  }

  s = abs(s);
  //Serial.print("Right: " + String(s));

  analogWrite(motorRightPWM, s);
}

void loop(){
  // setLeft(200);
  // setRight(200);

  // delay(2000);

  // setLeft(100);
  // setRight(100);

  // delay(2000);

  // setLeft(-200);
  // setRight(-200);

  // delay(2000);

  int frontDistance = frontSonar.ping_cm();
  Serial.println("Front Distance: " + String(frontDistance) + "cm");

  int leftDistance = leftSonar.ping_cm();
  Serial.println("Left Distance: " + String(leftDistance) + "cm");

  int rightDistance = rightSonar.ping_cm();
  Serial.println("Right Distance: " + String(rightDistance) + "cm");

  Serial.println();

  delay(500);
}


