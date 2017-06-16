/*
 * TwoRobotMain.ino:
 *
 *     This class contains the main loop and the intitialization code 
 *    for the two Robot solution. All interaction with the arduino
 *    board is handled from here.
 *
 *    Authors: Nathan Herr, Daschel Fortner
 *    Date:  1/20/16
 *      Modifications:
 *      - Added documentation: DTF
 *      
 */
#include <Servo.h>
#include <Arduino.h>
#include "Robot.h"
#include "constants.h"
#include "pins.h"
#include "stateMachine.h"
#include "PT6961.h"
#include "Timer.h"

using namespace stateMachine;


// The seven segment display 
Robot theRobot;   // A compilation of the Robot's data
int count; //Integer value counting the number of iterations through the main loop.
PT6961 display(DIN, CLOCK, CS);	


/*
 * Initializes all of the pins, and preps all of the hardware for use.
 */
void setup() {

  display.initDisplay();
  
  // Line sensor initialization
  pinMode(SENSOR_0, INPUT);
  pinMode(SENSOR_1, INPUT);
  pinMode(SENSOR_2, INPUT);
  pinMode(SENSOR_3, INPUT);
  pinMode(SENSOR_4, INPUT);
  pinMode(SENSOR_5, INPUT);
  pinMode(SENSOR_6, INPUT);
  pinMode(SENSOR_7, INPUT);
  
  pinMode(GO_BUTTON, INPUT_PULLUP);

  // Motor controller pins for the right motor
  pinMode(MC_PWMA, OUTPUT);
  pinMode(MC_AIN2, OUTPUT);
  pinMode(MC_AIN1, OUTPUT);
  
  // Motor controller pins for the left motor
  pinMode(MC_PWMB, OUTPUT);
  pinMode(MC_BIN1, OUTPUT);
  pinMode(MC_BIN2, OUTPUT);

  pinMode(CLAW_SENSOR, INPUT);
  
  pinMode(START_SENSOR, INPUT);
   
  /*
   * The following output configurations set both motors 
   * to move forward. The two Robot solution doesn't require
   * backwards movement, so the wheels should be permanently
   * forward.
   *
   * - Note: when the new boards are installed, this code will be 
   * 	     changed because the board is going to tie these to high
   *  - Issues here the right wheel is only spinning backwards
   */
   
  digitalWrite(MC_AIN1, LOW);
  digitalWrite(MC_AIN2, HIGH);
  digitalWrite(MC_BIN1, HIGH);
  digitalWrite(MC_BIN2, LOW); 

  
  // Attach the Robot's Servos to the Arduino pins
  theRobot.getServo(theRobot.EJECT).attach(EJECT_SERVO);   //this isn't working for some unknown reason
  theRobot.getServo(theRobot.ARM).attach(ARM_SERVO);       //the Robot class has been modified to make the
  theRobot.getServo(theRobot.DUMP).attach(DUMP_SERVO);     //servos public 3/2/17 NH
  theRobot.getServo(theRobot.CLAW).attach(CLAW_SERVO);

  // Set the initial position of the Servo
  theRobot.writeToServo(theRobot.EJECT, EJECT_FRONT_POSITION);
  theRobot.writeToServo(theRobot.DUMP, DUMP_DOWN);
  theRobot.writeToServo(theRobot.ARM, ARM_DOWN);
  theRobot.writeToServo(theRobot.CLAW, CLAW_OPEN);
  
  
  theRobot.amountSeen = 0;
  theRobot.firstLineIndex = 0; 
  theRobot.lastLineIndex = 0;
  theRobot.clawSensorDistance = 0;
  count = 0;
	stateMachine::resetRobot(theRobot);
}

/*
 * Executes the "thought process" of the Robot. 
 * This is the main loop for the Robot's execution of the problem's states.
 */
void loop() {

  // Refresh the Robot's data
  //theRobot = readData(theRobot);
	readData(theRobot);
	
  /*
   * Check the current state's exit conditions and execute the action 
   * for the current state.
   */
   
   stateMachine::updateState(theRobot);
   stateMachine::execute(theRobot);

 
  count++;
  // Send a debug message if the number of loops has exceeded 131
  if(count % 131 == 0){
		//debug(theRobot.frontSensorDistance/1000, (theRobot.wallSensorDistance%1000)/ 100, 
            //(theRobot.frontSensorDistance%100)/10, theRobot.currentState % 10);
    //debug(theRobot.firstLineIndex, theRobot.lastLineIndex,
          //(theRobot.amountSeen), theRobot.currentState % 10);
    debugFourDigits(theRobot.startSensor);
   
  }
 
}

/*
 * Creates a new Robot object with current data, while preserving some 
 * from the previous Robot. Preserves the Robot's current state and 
 * Servo information.
 */
void readData(Robot& previousRobot) {
  //Robot newRobot(previousRobot);
  
  // Preserve necessary information

  // Reads in the line sensor data
  int lineData[8];
  lineData[0] = digitalRead(SENSOR_0);
  lineData[1] = digitalRead(SENSOR_1);
  lineData[2] = digitalRead(SENSOR_2);// Can't use a for() b/c of alphanumeric pin ids
  lineData[3] = digitalRead(SENSOR_3);
  lineData[4] = digitalRead(SENSOR_4);
  lineData[5] = digitalRead(SENSOR_5);
  lineData[6] = digitalRead(SENSOR_6);
  lineData[7] = digitalRead(SENSOR_7);

  previousRobot.amountSeen = 0;
  previousRobot.lastLineIndex = 9;
  previousRobot.firstLineIndex = 9;
  // Record the line sensor's data
  for(int i = 0; i < 8; i++) {
    if(lineData[i] == 1) {
	
       previousRobot.lastLineIndex = i;
        
       if(previousRobot.firstLineIndex == 9) {
          previousRobot.firstLineIndex = i;
       }
	   
       previousRobot.amountSeen++;
    }
  }
  if(previousRobot.amountSeen == 0 && theRobot.currentState > 25){
    previousRobot.amountSeen = previousRobot.pastAmountSeen;
    previousRobot.firstLineIndex = previousRobot.pastFirstIndex;
    previousRobot.lastLineIndex = previousRobot.pastLastIndex;
  }

  //reading distance sensors
  previousRobot.frontSensorDistance = analogRead(FRONT_SENSOR);
  previousRobot.clawSensorDistance = analogRead(CLAW_SENSOR);
  previousRobot.wallSensorDistance = analogRead(WALL_SENSOR1);
  previousRobot.pastAmountSeen = previousRobot.amountSeen;
  previousRobot.pastFirstIndex = previousRobot.firstLineIndex;
  previousRobot.pastLastIndex = previousRobot.lastLineIndex; 

  //read start sensor
  previousRobot.startSensor = analogRead(START_SENSOR);
 
}

/*
 * Sends a message to the Seven Segment display. 
 * To preserve a previous number, send -1 for that index of the display.
 * For example, after these lines of code:
 *
 *      debug(0, 1, 2, 3);
 *      debug(-1, 2, -1, -1);
 *
 * The Seven Segment will display these digits:
 * 
 * 		0 2 2 3
 */
void debug(int indexZero, int indexOne, int indexTwo, int indexThree) {
  static int message[4] = {9, 9, 9, 9};

  if(indexZero != -1){
    message[0] = indexZero;
  }
  if(indexOne != -1){
    message[1] = indexOne;
  }
  if(indexTwo != -1){
    message[2] = indexTwo;
  }
  if(indexThree != -1){
    message[3] = indexThree;
  }
    
  display.sendDigits((char)message[0], (char)message[1], (char)message[2], (char)message[3], 0);
}

void debugFourDigits(int number){
    if(number > -1 && number < 65535){
        display.sendNum(number, '0');
    }
}

