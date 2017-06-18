#include "Robot.h"

Robot::Robot() {
  currentState       = 0;

  firstLineIndex     = 9;
  lastLineIndex      = 9;
  amountSeen         = 0;

  wallSensorDistance = 0;
  clawSensorDistance = 0;
  frontSensorDistance = 0;

  grabTimerInt = 0;
//	Timer oneTimer;
//  Timer ejectTimer;
//	Timer jiggleTime;
}

Robot::Robot(const Robot& previousRobot) {
  currentState = previousRobot.currentState;
  oneTimer = previousRobot.oneTimer;
	ejectTimer = previousRobot.ejectTimer;
	jiggleTimer = previousRobot.jiggleTimer;
	grabTimerInt = previousRobot.grabTimerInt;
  
  for(int i = 0; i < 4; i++) {
    motors[i] = previousRobot.motors[i];
  }
}

void Robot::writeToServo(const int SERVO, int val) {
  motors[SERVO].write(val);
}

Servo& Robot::getServo(const int SERVO) {
  return motors[SERVO];
}
