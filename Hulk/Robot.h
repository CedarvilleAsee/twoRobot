#include <Servo.h>
#include "pins.h"
#include "Timer.h"

#ifndef ROBOT_H
#define ROBOT_H



/*
 * Robot class:
 *  Holds all of the data for the Robot, including the Servo objects and the 
 *  sensor data.
 *  - modified - added previousMicros and state variables for soft PWM 3/1/2017 - NSH
 *  - modified - remove stuff for soft PWM 3/18/17- NSH
 *  note: the servos array could probably be safely made private as it was in previous versions
 */
class Robot {

public:

  // Constants for accessing Servo objects
  const static int EJECT = 0;
  const static int ARM   = 1;
  const static int DUMP  = 2;
  const static int CLAW  = 3;

  /*
   * Constructs a new Robot with all initial values set.
   */
  Robot();

  /*
   * Constructs a new Robot with the data except sensor data copied
   */
  Robot(const Robot&);

  /*
   * Gets a Servo reference on this robots.
   */
  Servo& getServo(const int SERVO);

  /*
   * Writes to a servo. Use the above constants to access a specific Servo object.
   */
  void writeToServo(const int, int);


  int currentState;

  int firstLineIndex, lastLineIndex, amountSeen;
  int pastFirstIndex, pastLastIndex, pastAmountSeen;

  int wallSensorDistance, clawSensorDistance, frontSensorDistance;

  int grabTimerInt;
  
  Timer oneTimer;
  Timer ejectTimer;
  Timer jiggleTimer;
  Timer turnTimer;
  Timer errorTimer;

  Servo motors[4];
  
};

#endif
