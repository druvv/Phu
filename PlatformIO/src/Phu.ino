#include "Arduino.h"
#include "NewPing.h"

#define DUBUG_MODE

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

// -- MARK: PROGRAM CONFIGURATION
const int MAX_DISTANCE = 100; // cm
const int WALL_DETECT_THRESHOLD = 10; // 10 cm
const int WALL_CENTERING_DISTANCE = 7; // 7cm
const int TURN_TIME = 1000; // ms
const int WALL_AVOIDANCE_TIME = 500; // ms
bool defaultTurnIsLeft = true;

enum TurnDirection {counterclockwise, clockwise, around};
enum Facing {left, right, forwards};
Facing robotDirection = forwards;

NewPing frontSonar(frontTrig, frontEcho, MAX_DISTANCE);
NewPing leftSonar(leftTrig, leftEcho, MAX_DISTANCE);
NewPing rightSonar(rightTrig, rightEcho, MAX_DISTANCE);
long frontDistance = 0;
long leftDistance = 0;
long rightDistance = 0;

void setup() {
  #ifdef DUBUG_MODE
    Serial.begin(9600);
    while (!Serial) {} // Wait for serial to connect
    Serial.println("Awake!");
  #endif

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

// - MARK: MOTOR SPEEDS

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

// - MARK: TURNING

void rotateRobotDirection(TurnDirection direction) {
  switch (robotDirection) {
  case left:
    if (direction == clockwise) {
      robotDirection = forwards;
    }

    if (direction == around) {
      robotDirection = right;
    }

    break;
  case right:
    if (direction == counterclockwise) {
      robotDirection = forwards;
    }

    if (direction == around) {
      robotDirection = left;
    }

    break;
  case forwards:
    if (direction == counterclockwise) {
      robotDirection = left;
    }

    if (direction == clockwise) {
      robotDirection = right;
    }

    break;
  }

  #ifdef DUBUG_MODE
    if (robotDirection == forwards) {
      Serial.println("DIR: F");
    } else if (robotDirection == left) {
      Serial.println("DIR: L");
    } else {
      Serial.println("DIR: R");
    }
  #endif
}

void turn(TurnDirection direction) {
  switch (direction) {
  case counterclockwise:
    setLeft(-250);
    setRight(250);
    delay(TURN_TIME);
    break;
  case clockwise:
    setLeft(250);
    setRight(-250);
    robotDirection = right;
    delay(TURN_TIME);
    break;
  case around:
    setLeft(-250);
    setRight(250);
    if (robotDirection == left) {
      robotDirection = right;
    } else {
      robotDirection = left;
    }
    delay(TURN_TIME * 2);
    break;
  }

  rotateRobotDirection(direction);
}

void crawl(TurnDirection direction) {
  int fastSpeed = 250;
  int slowSpeed = 150;

  switch (direction) {
  case clockwise:
      setLeft(fastSpeed);
      setRight(slowSpeed);
      break;
  case counterclockwise:
      setLeft(slowSpeed);
      setRight(fastSpeed);
      break;
  default:
    break;
  }
}

// Moves forward with a basic centering algorithm
void moveForwards() {
  if (rightDistance <= WALL_DETECT_THRESHOLD && leftDistance <= WALL_DETECT_THRESHOLD) {
    if (rightDistance > leftDistance) {
      crawl(clockwise);
    } else {
      crawl(counterclockwise);
    }
  } else if (rightDistance <= WALL_DETECT_THRESHOLD) {
    if (rightDistance < WALL_CENTERING_DISTANCE) {
      crawl(counterclockwise);
    } else {
      crawl(clockwise);
    }
  } else if (leftDistance <= WALL_DETECT_THRESHOLD) {
    if (leftDistance < WALL_CENTERING_DISTANCE) {
      crawl(clockwise);
    } else {
      crawl(counterclockwise);
    }
  }
}

void verifyDistances() {
  if (frontDistance == 0) {
    frontDistance = 10000;
  }

  if (leftDistance == 0) {
    leftDistance = 10000;
  }

  if (rightDistance == 0) {
    rightDistance = 10000;
  }
}

// - MARK: MAIN LOOP

void loop() {
   // Needs to be a minimum of 29ms between pings to prevent cross-sensor echo according to NewPing documentation.
  frontDistance = frontSonar.ping_cm();
  delay(30);
  leftDistance = leftSonar.ping_cm();
  delay(30);
  rightDistance = rightSonar.ping_cm();

  verifyDistances();

  // If we are facing forwards, move forward until we detect a wall.
  if (robotDirection == forwards && frontDistance <= WALL_DETECT_THRESHOLD) {
    int difference = leftDistance - rightDistance;
    // If the left and right walls are about equally apart turn in the default direction.
    if (abs(difference) < 10) {
      if (defaultTurnIsLeft) {
        turn(counterclockwise);
      } else {
        turn(clockwise);
      }
    // If the left wall is closer than the right wall, move right.
    } else if (difference < 0) {
      turn(clockwise);
    // If the right wall us closer than the left wall, move left.
    } else {
      turn(counterclockwise);
    }
  // If we are facing left, move until we don't detect a right wall.
  } else if (robotDirection == left) {
    // If there are walls on all three sides, turn around.
    if (leftDistance <= WALL_DETECT_THRESHOLD && rightDistance <= WALL_DETECT_THRESHOLD && frontDistance <= WALL_DETECT_THRESHOLD) {
      turn(around);
    } else if (rightDistance > WALL_DETECT_THRESHOLD) {
      // Move a little to center ourselves, then go forwards.
      setLeft(250); setRight(250);
      delay(WALL_AVOIDANCE_TIME);
      turn(clockwise);
    }
   // If we are facing right, move until we don't detect a left wall. 
  } else if (robotDirection == right) {
    // If there are walls on all three sides, turn around.
    if (leftDistance <= WALL_DETECT_THRESHOLD && rightDistance <= WALL_DETECT_THRESHOLD && frontDistance <= WALL_DETECT_THRESHOLD) {
      turn(around);
    } else if (leftDistance > WALL_DETECT_THRESHOLD) {
      // Move a little to center ourselves, then go forwards.
      setLeft(250); setRight(250);
      delay(WALL_AVOIDANCE_TIME);
      turn(counterclockwise);
    }
  }

  // Move according to centering program
  moveForwards();
  delay(30);
} 


