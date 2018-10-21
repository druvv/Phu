const int motorLeftPWM = 11;
const int motorLeftA = 5;
const int motorLeftB = 4;

const int motorRightPWM = 10;
const int motorRightA = 3;
const int motorRightB = 2;

const int frontTrig = 7;
const int frontEcho 6;

const int leftTrig = 9;
const int leftEcho = 9;

const int rightTrig = 13;
const int rightEcho = 12;

void setup() {
  pinMode(motorLeftPWM, OUTPUT);
  pinMode(motorLeftA, OUTPUT);
  pinMode(motorLeftB, OUTPUT);

  pinMode(motorRightPWM, OUTPUT);
  pinMode(motorRightA, OUTPUT);
  pinMode(motorRightB, OUTPUT);
}


// s should be from - 255 to 255
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

  analogWrite(motorLeftPWM, s);
}

void setRight(int s) {
  if (s > 0) {
    digitalWrite(motorRightA, LOW);
    digitalWrite(motorRightB, HIGH);
  } else if (s < 0) {
    digitalWrite(motorRightA, HIGH);
    digitalWrite(motorRightB, LOW);
  } else {
    digitalWrite(motorRightA, LOW);
    digitalWrite(motorRightB, LOW);
  }

  s = abs(s);

  analogWrite(motorRightPWM, s);
}

void loop() {
  setLeft(255);
  setRight(255);

  delay(1000);
  setLeft(0);
  setRight(0);
  delay(1000);

  setLeft(-255);
  setRight(-255);

  delay(1000);
  setLeft(0);
  setRight(0);
  delay(1000);

  setLeft(124);
  setRight(124);
}
