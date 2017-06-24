/*
	stateMachine.cpp

   	Handles the behavior of the Robot for any given state. This file contains methods
   	for both carrying out a state and advancing it.

   	Author: Nathan Herr, Daschel Fortner, Spencer Graffunder
   	Date:   1/20/17
   	Modifications:
 		- Added documentation - DTF 1/27/17
    	- Added soft PWM in stateMachine - NH 3/1/17
    	- Removed soft PWM in stateMachine - NH 3/1/17
		- We made it work DTF & NH 3/20 through 4/6
*/

#include "stateMachine.h"
#include "constants.h"
#include "pins.h"

/*
   updateState method:

       Advances the current state of the passed Robot by evaluating the exit conditions for
       a given state, as defined by Robot.currentState.

       - theRobot: the data which is evaluated to update the state
			 -this method will need heavy modification for use in D2_RIGHT
*/
void stateMachine::updateState(Robot& theRobot) {
  	// Check for a reset button push
  	if(digitalRead(STOP_BUTTON) == LOW) {
    	resetRobot(theRobot);
    	return;
  	}else {
		commonStates(theRobot);
	}
}

void stateMachine::commonStates(Robot& theRobot){

	switch (theRobot.currentState) {
		case 0: // WAIT -> LEFT_TURN
            resetRobot(theRobot);
			digitalWrite(START_LIGHT, HIGH);
            if (digitalRead(GO_BUTTON) == LOW) {
				//delay(1000);//to make recording easier. take out before competition
                //accelerate(0, FULL_SPEED, 300);
                theRobot.writeToServo(theRobot.ARM, ARM_MID);
                theRobot.currentState++;
                //theRobot.currentState = 35;
            }
            break;

		case 1: // LEFT_TURN -> LINE_FOLLOW
			digitalWrite(START_LIGHT, LOW);
			if(theRobot.amountSeen > 1) {
				theRobot.currentState++;
				theRobot.oneTimer.set(200);
			}
			break;

		case 2: // LINE_FOLLOW -> LEFT_TURN
			digitalWrite(MC_AIN1, LOW);
			digitalWrite(MC_AIN2, HIGH);
			digitalWrite(MC_BIN1, HIGH);
			digitalWrite(MC_BIN2, LOW);
			if(!theRobot.errorTimer.isTimerSet()){
				theRobot.errorTimer.set(2400);
			}
			if(theRobot.errorTimer.isTimeUpUnset()){
				digitalWrite(MC_AIN1, HIGH);
				digitalWrite(MC_AIN2, LOW);
				digitalWrite(MC_BIN1, LOW);
				digitalWrite(MC_BIN2, HIGH);
				writeToWheels(60, 90);
				delay(1000);
				digitalWrite(MC_AIN1, LOW);
				digitalWrite(MC_AIN2, HIGH);
				digitalWrite(MC_BIN1, HIGH);
				digitalWrite(MC_BIN2, LOW);
				writeToWheels(50, 50);
				delay(200);
			}
			if(theRobot.amountSeen > 3 && theRobot.oneTimer.isTimeUpUnset()) {
				theRobot.currentState++;
				theRobot.oneTimer.set(300 * TIMING_CONST);
				theRobot.errorTimer.unset();
			}
			break;

		case 3: // LEFT_TURN -> LINE_FOLLOW
			if(theRobot.amountSeen > 1 && theRobot.oneTimer.isTimeUpUnset()){
				theRobot.currentState++;
			}
			break;

		case 4: // LINE_FOLLOW -> HANDLE_OBSTACLE
			if(theRobot.amountSeen == 0){
				theRobot.currentState++;
				// SG 6/11 changed from 3000 because it was hitting corner
				theRobot.oneTimer.set(2200 * TIMING_CONST);
			}
			break;

		case 5: // HANDLE_OBSTACLE -> FIND_LINE
            if(theRobot.oneTimer.isTimeUpUnset()){
				theRobot.writeToServo(theRobot.ARM, ARM_DOWN);
                theRobot.currentState++;
            }
            break;

		case 6: // FIND_LINE
			if (!theRobot.errorTimer.isTimerSet()) {
				theRobot.errorTimer.set(2000);
			} else if (theRobot.errorTimer.isTimeUpUnset()) {
				digitalWrite(MC_AIN1, HIGH);
				digitalWrite(MC_AIN2, LOW);
				//digitalWrite(MC_BIN1, LOW);
				//digitalWrite(MC_BIN2, HIGH);
				writeToWheels(0, 60);
				delay(1500);
				digitalWrite(MC_AIN1, LOW);
				digitalWrite(MC_AIN2, HIGH);
				digitalWrite(MC_BIN1, HIGH);
				digitalWrite(MC_BIN2, LOW);
			}
			if(theRobot.amountSeen > 1){
				theRobot.oneTimer.set(400*TIMING_CONST);
				theRobot.currentState++;
			}
			break;

		case 7: // LINE_FOLLOW_OFFSET2
			if(theRobot.oneTimer.isTimeUpUnset()){
				theRobot.currentState++;
			}
			break;

		case 8:  // LINE_FOLLOW_OFFSET
			if(theRobot.leftSensorDistance < WALL_TRIGGER) {
				theRobot.currentState++;
				theRobot.ejectTimer.set(250);
			}
			break;

		case 9: // EJECT_BARREL
			if(theRobot.clawSensorDistance < CLAW_GRAB_TRIGGER_1_3){
				theRobot.currentState++;
				theRobot.grabTimerInt = millis();
				grabBarrel(theRobot);
				theRobot.oneTimer.set(1000*TIMING_CONST);
			}
			break;

		case 10: // GRAB_BARREL
			if(theRobot.oneTimer.isTimeUpUnset()){
				theRobot.currentState++;
				theRobot.ejectTimer.unset();
				theRobot.writeToServo(theRobot.EJECT, EJECT_BACK_POSITION);
				theRobot.oneTimer.unset();
			}
			break;

		case 11: // LINE_FOLLOW
			if(!theRobot.oneTimer.isTimerSet() && theRobot.amountSeen > 3) {
				theRobot.currentState++;
				theRobot.oneTimer.set(500*TIMING_CONST);
				theRobot.ejectTimer.unset();
			}
			break;

		case 12:  // LINE_FOLLOW_OFFSET
			if(theRobot.amountSeen > 3 && theRobot.oneTimer.isTimeUpUnset()){
				theRobot.currentState++;
				theRobot.ejectTimer.unset();
			}
			break;

		case 13:  //FIND_CORNER_BARREL
			if(theRobot.clawSensorDistance < CLAW_GRAB_TRIGGER_2){
				theRobot.currentState++;
				theRobot.grabTimerInt = millis();
				theRobot.ejectTimer.unset();
			}
			break;

		case 14:  // GRAB_CORNER_BARREL
			if(theRobot.rightSensorDistance < FRONT_CORNER_TRIGGER){
				theRobot.oneTimer.set(300*TIMING_CONST);
				theRobot.currentState++;
			}
			break;

		case 15: // ROUND_A_BOUT
			if(theRobot.amountSeen > 1 && theRobot.oneTimer.isTimeUpUnset()){
				theRobot.currentState++;
				theRobot.oneTimer.set(200*TIMING_CONST);
			}
      		break;

		case 16: // ROUND_A_BOUT
			if(theRobot.firstLineIndex == 3 && theRobot.oneTimer.isTimeUpUnset()){
				theRobot.currentState++;
				theRobot.oneTimer.set(200);
			}
            break;

		case 17: // LINE_FOLLOW
			if(theRobot.oneTimer.isTimeUpUnset()){
				theRobot.currentState++;
			}
            break;

		case 18: // LINE_FOLLOW
			theRobot.ejectTimer.unset();
			theRobot.writeToServo(theRobot.EJECT, EJECT_BACK_POSITION);
			digitalWrite(MC_AIN1, LOW);
			digitalWrite(MC_AIN2, HIGH);
			digitalWrite(MC_BIN1, HIGH);
			digitalWrite(MC_BIN2, LOW);
			// SG 6/11 added
			theRobot.writeToServo(theRobot.CLAW, CLAW_OPEN);
			if(!theRobot.oneTimer.isTimerSet()){
				// SG 6/11 changed from 60 to avoid wheele advancing state
				theRobot.oneTimer.set(200*TIMING_CONST);
			}
			if(theRobot.amountSeen > 3 && theRobot.oneTimer.isTimeUpUnset()){
				theRobot.currentState++;
				theRobot.writeToServo(theRobot.ARM, ARM_DOWN);
				theRobot.oneTimer.set(350*TIMING_CONST);
			}
			break;

		case 19: // LEFT_TURN
			if(theRobot.amountSeen > 1 && theRobot.oneTimer.isTimeUpUnset()){
				theRobot.currentState++;
				theRobot.writeToServo(theRobot.ARM, ARM_DOWN);
				theRobot.currentState++;
				theRobot.ejectTimer.set(500);
			}
			break;

		case 20: // LINE_FOLLOW_OFFSET

			break;
		
		case 21: // EJECT_BARREL
			if(theRobot.clawSensorDistance < CLAW_GRAB_TRIGGER_1_3){
				theRobot.currentState++;
				theRobot.grabTimerInt = millis();
				theRobot.oneTimer.set(300*TIMING_CONST);
			}
			break;

		case 22: // GRAB_BARREL
			if(theRobot.oneTimer.isTimeUpUnset()){
				theRobot.currentState++;
				//theRobot.oneTimer.set(100*TIMING_CONST);
			}
			break;

		case 23: // ROUND_A_BOUT
			if(!theRobot.turnTimer.isTimerSet()) {
                theRobot.turnTimer.set(400);
            }
            else if(theRobot.firstLineIndex > 1 && theRobot.amountSeen > 1 && theRobot.turnTimer.isTimeUpUnset()) {
                theRobot.currentState++;
                theRobot.turnTimer.set(500);
            }
            break;

		case 24: // LINE_FOLLOW
            if(theRobot.amountSeen > 3 && theRobot.turnTimer.isTimeUpUnset()) {
				theRobot.currentState++;
				theRobot.oneTimer.set(400);
			}
			break;

		case 25: // LEFT_TURN
			if(theRobot.amountSeen > 1 && theRobot.oneTimer.isTimeUpUnset()){
				theRobot.currentState++;
				//theRobot.writeToServo(theRobot.ARM, ARM_DOWN);
			}
			break;

		case 26: // LINE_FOLLOW
            theRobot.writeToServo(theRobot.ARM, ARM_UP);
            if(theRobot.rightSensorDistance < 3100){
                theRobot.currentState++;
            }
			break;

		case 27: // LINE_FOLLOW
            theRobot.writeToServo(theRobot.CLAW, CLAW_OPEN);
			if(theRobot.rightSensorDistance > 3500){
                theRobot.currentState++;
                theRobot.oneTimer.set(500);
            }
			break;

		case 28: // LINE_FOLLOW
			if (theRobot.oneTimer.isTimeUpUnset()){
                theRobot.currentState++;
				theRobot.oneTimer.set(200);
            }
			break;

        case 29: // WALL_FOLLOW_RIGHT
            if (theRobot.amountSeen > 2 && theRobot.oneTimer.isTimeUpUnset()){
                theRobot.currentState++;
                //theRobot.oneTimer.unset();
                theRobot.oneTimer.set(200);
            }
			break;

        case 30: // WALL_FOLLOW_RIGHT
            if(theRobot.oneTimer.isTimeUpUnset()){
                theRobot.currentState++;
            }
            break;

        case 31: // VEER_LEFT
            if(theRobot.amountSeen > 1){
                theRobot.currentState++;
				theRobot.oneTimer.set(300);
            }
			break;

		case 32: // LINE_FOLLOW
			if(theRobot.amountSeen > 3 && theRobot.oneTimer.isTimeUpUnset()){
				theRobot.currentState++;
				theRobot.oneTimer.set(300);
			}
			break;
		
		case 33: // LEFT_TURN
			if(theRobot.oneTimer.isTimeUp() && theRobot.amountSeen > 1){
				theRobot.currentState++;
				theRobot.oneTimer.set(500);
			}
			break;
	
		case 34: // LINE_FOLLOW_OFFSET
			if(theRobot.oneTimer.isTimeUpUnset()){
				theRobot.currentState++;
			}
			break;

		case 35: // DUMP_BARRELS
			
			break;
	}
}	//end stateMachine::commonStates

/*
   execute method:

       The execute method performs the function for a state. The state is determined
       by the theRobot's currentState field, which becomes the index of the stateMap
       as defined in constants.h.

       - theRobot: the data used to execute the current state
*/
void  stateMachine::execute(Robot& theRobot) {

  switch (stateMap[theRobot.currentState]) {

	case LINE_FOLLOW:

        digitalWrite(MC_AIN1, LOW);
        digitalWrite(MC_AIN2, HIGH);
        digitalWrite(MC_BIN1, HIGH);
        digitalWrite(MC_BIN2, LOW);

		if(theRobot.currentState == 17){
			lineFollow(theRobot, 0, 90);
		}else{
			lineFollow(theRobot, 0);
		}
		
        break;

	case WAIT:
		writeToWheels(0, 0);
		break;

    case LINE_FOLLOW_OFFSET2:
        lineFollow(theRobot, 2);
        break;

    case DEPART_SPAIN:
		digitalWrite(MC_AIN1, LOW);
		digitalWrite(MC_AIN2, HIGH);
        writeToWheels(FULL_SPEED*.6, FULL_SPEED*.6);
        break;

    case GRAB_BARREL: //using fall through
		grabBarrel(theRobot);
		if(theRobot.currentState == 22){
			writeToWheels(0, 0);
			break;
		}
	case EJECT_BARREL:
		ejectBarrel(theRobot);
	case LINE_FOLLOW_OFFSET:
		lineFollow(theRobot, -2);
        break;

    case GRAB_CORNER_BARREL:
        grabBarrel(theRobot);
		ejectCornerBarrel(theRobot);
	case FIND_CORNER_BARREL:
		writeToWheels(FULL_SPEED*.5, FULL_SPEED*.5);
        break;

    case ROUND_A_BOUT:
		if(theRobot.currentState == 16){
			theRobot.writeToServo(theRobot.ARM, ARM_UP);
		}
		digitalWrite(MC_AIN1, HIGH);
		digitalWrite(MC_AIN2, LOW);
		if(theRobot.currentState == 15){
			writeToWheels(FULL_SPEED * 0.35, FULL_SPEED * 0.35);
		}else{
			writeToWheels(FULL_SPEED * 0.5, FULL_SPEED * 0.5);
		}
		ejectCornerBarrel(theRobot);
        break;

    case LEFT_TURN:
		if(theRobot.currentState == 1){
			writeToWheels(0, FULL_SPEED * 0.6);
		} else {
			writeToWheels(0, FULL_SPEED);
        }
		break;

	case LEFT_TURN_SPIN:
		digitalWrite(MC_BIN1, LOW);
		digitalWrite(MC_BIN2, HIGH);
		writeToWheels(FULL_SPEED * .45, FULL_SPEED* .7);
		break;

    case RIGHT_TURN:
        writeToWheels(FULL_SPEED, 0);
        break;

    case FIND_LINE:
		#ifdef R2_LEFT
			writeToWheels(FULL_SPEED, RIGHT_WHEEL_SPEEDS[5]);
		#else
			writeToWheels(LEFT_WHEEL_SPEEDS[9], FULL_SPEED);
		#endif

      break;

    case HANDLE_OBSTACLE:
		wallFollowLeft(theRobot.leftSensorDistance, WALL_FOLLOW_CENTER_LEFT);
        //wallFollow(theRobot, WALL_FOLLOW_CENTER);
        grabBarrel(theRobot);
        break;

    case WALL_FOLLOW_FAR:
		wallFollowLeft(theRobot.leftSensorDistance, WALL_FOLLOW_CENTER_LEFT);
        //wallFollow(theRobot, WALL_FOLLOW_CENTER_LEFT);
        break;

    case DUMP_BARRELS:
		writeToWheels(0, 0);
		theRobot.writeToServo(theRobot.ARM, ARM_MID);
		if(theRobot.dumpPosition == 700 && theRobot.dumpTimer < millis()){
			theRobot.writeToServo(theRobot.DUMP, DUMP_UP);
			delay(500);
			theRobot.currentState = 0;
		}
	
	    //just starting to dump
        if(theRobot.dumpTimer == 0){
			theRobot.writeToServo(theRobot.DUMP, 6.0/8.0*(float)(DUMP_UP-DUMP_DOWN)+DUMP_DOWN);
			theRobot.dumpPosition = 6.0/8.0*(float)(DUMP_UP-DUMP_DOWN)+DUMP_DOWN;
			theRobot.dumpTimer = millis() + 400;
		}
		//already dumped all the way
		if(theRobot.dumpPosition == DUMP_UP && theRobot.dumpTimer < millis()){
			theRobot.writeToServo(theRobot.DUMP, DUMP_DOWN);
			theRobot.dumpTimer = millis() + 700;
			theRobot.dumpPosition = 700;
			
		//need to dump a little more
		}else if(theRobot.dumpTimer < millis()){
			theRobot.dumpPosition += 1;
			theRobot.dumpTimer = millis()+ 40;
			theRobot.writeToServo(theRobot.DUMP, theRobot.dumpPosition);
			
			//wait for barrels to roll out
			if(theRobot.dumpPosition == DUMP_UP){
				theRobot.dumpTimer = millis() + 700;
			}
		}
        break;

    case WALL_FOLLOW_RIGHT:
		wallFollowRight(theRobot.rightSensorDistance, 2900);
        //wallFollow(theRobot.rightSensorDistance, 2900);
        break;

    case VEER_LEFT:
        writeToWheels(FULL_SPEED * 0.8, FULL_SPEED);
        break;

  }

}

/*
  writeToWheels method:

     Writes a specific left and right turn speed to the wheels.

     leftSpeed  - the speed to write to the left wheel
     rightSpeed - the speed to write to the right wheel
*/
void stateMachine::writeToWheels(int leftSpeed, int rightSpeed) {
  analogWrite(MC_PWMA, rightSpeed);
  analogWrite(MC_PWMB, leftSpeed);
}

/*
    ejectBarrel method:
    	Ejects a barrel from the Robot.
		- theRobot: the data containing the necessary servo information
		-This method should be the same for both robots, assuming the constants are changed correctly
*/
void stateMachine::ejectBarrel(Robot& theRobot) {
	if(theRobot.ejectTimer.isTimeUpUnset()){
		theRobot.writeToServo(theRobot.EJECT, EJECT_FRONT_POSITION);
	}
}

void stateMachine::ejectCornerBarrel(Robot& theRobot) {

	if (!theRobot.ejectTimer.isTimerSet()) {
		theRobot.ejectTimer.set(400); // SG 6/14 changed from 200
	}
	if (theRobot.ejectTimer.isTimeUpUnset()) {
		theRobot.writeToServo(theRobot.EJECT, EJECT_FRONT_POSITION);
		theRobot.ejectTimer.set(KICKER_MOVE_BACK_TIME);
	}

}

/**
   grabBarrel method:
   method for grabbing the barrel, dropping it in the bin and resetting the arm.
   the method is intended to be called continously throughout the process.
	 -This method should be the same for both robots, assuming the constants are changed correctly
*/
void stateMachine::grabBarrel(Robot& theRobot) {
	theRobot.writeToServo(theRobot.DUMP, DUMP_DOWN);
	if (theRobot.grabTimerInt + 150 > millis()) { //initial call
		theRobot.writeToServo(theRobot.CLAW, CLAW_CLOSED);
	} //the following statements are in inverted order of expected excution
	else if (theRobot.grabTimerInt + 800 < millis()) { //last part: reseting the arm to the lowered position
		theRobot.writeToServo(theRobot.ARM, ARM_DOWN);
	}
	else if (theRobot.grabTimerInt + 700 < millis()) { //opening the claw to drop the barrel
		theRobot.writeToServo(theRobot.CLAW, CLAW_OPEN);
	}
	else if (theRobot.grabTimerInt + 150 < millis()) { //lifting the arm once the barrel has been grabbed
		theRobot.writeToServo(theRobot.ARM, ARM_UP);
	}
}

/**
 * getTurnIndex method
 * created 3/29/17; NSH
 * this method is takes the firstLineIndex, lastLineIndex and amountSeen and converts them
 * into a integer value [-7, 7]
 * 0 respents the position far to the right, 7 is centered with two sensors on the line
 * 14 is the postion far to the left
 */
int stateMachine::getTurnIndex(Robot& theRobot){
	int finalIndex = 0;

	if(theRobot.amountSeen == 1){
		finalIndex = theRobot.firstLineIndex * 2;  //firstLineIndex is a value 0 to 7
	}
	else if(theRobot.amountSeen == 2){
		finalIndex = theRobot.firstLineIndex + theRobot.lastLineIndex; //here firstLineIndex is [0, 6]; lastLineIndex is [1, 7]
	}
	else if(theRobot.amountSeen > 2){//houston we have a problem...
	//but seriously this will need to be dealt with in a better way
		finalIndex = (theRobot.lastLineIndex + theRobot.firstLineIndex);
	}
	else if(theRobot.amountSeen == 0){
		finalIndex = 0;
	}
	
	return finalIndex;
}

/**
 * method lineFollow
 * created 3/29/17 by NSH
 * implements line following based on the getTurnIndex method and the LEFT_WHEEL_SPEEDS and RIGHT_WHEEL_SPEEDS
 * arrays
 * offset should be a value from -7 to +7
 *
 */
void stateMachine::lineFollow(Robot& theRobot, int offset, int fullSpeed){
	int index = getTurnIndex(theRobot);

	index += offset;//adding in offset and making sure it doesn't run off the end of the arrays
	if(index < 0){
		index = 0;
	}else if(index > 14){
		index = 14;
	}
	if(theRobot.currentState == 4){
		writeToWheels(LEFT_WHEEL_SPEEDS[index] * 1.4,	RIGHT_WHEEL_SPEEDS[index] * 1.4);
	} else {
		writeToWheels(LEFT_WHEEL_SPEEDS[index],	RIGHT_WHEEL_SPEEDS[index]);
	}
}

/**
 * resetRobot method
 * created by NH & DF on 3/31/17
 * this method resets the state to 0 and resets the function timing values to their default
 * this method is a full stop for the robot
 * modified 4/5 to accomadate Timer class
 */
void stateMachine::resetRobot(Robot& theRobot){
  theRobot.currentState = 0;
  theRobot.ejectTimer.unset();
  theRobot.oneTimer.unset();
  theRobot.errorTimer.unset();
  theRobot.turnTimer.unset();
  
  theRobot.dumpTimer = 0;
  theRobot.dumpPosition = DUMP_DOWN;
	
  writeToWheels(0,0);
  digitalWrite(MC_AIN1, LOW);
  digitalWrite(MC_AIN2, HIGH);

  theRobot.writeToServo(theRobot.EJECT, EJECT_BACK_POSITION);
  theRobot.writeToServo(theRobot.DUMP, DUMP_DOWN);
  theRobot.writeToServo(theRobot.ARM, ARM_START);
  theRobot.writeToServo(theRobot.CLAW, CLAW_OPEN);
}


/**
 * Modified wall Follow method
 * modified by DF on 6/17/17; Basically just NH's method
 * This method takes an integer instead of the whole robot, so it can be used
 * with either wall sensor.
 */
void stateMachine::wallFollow(int sensorDistance, int center){
	float ratio = 0.0;
	if(sensorDistance < center){ //turn right
		ratio = (center  - sensorDistance)/(float)LEFT_WF_FAR_GAIN;
		ratio = 1.0 - ratio;
	if(ratio < 0.0){
		ratio = 0.0;
	}
	writeToWheels(FULL_SPEED * ratio, FULL_SPEED);
	} else { //turn left
		ratio = (sensorDistance - center)/(float)LEFT_WF_CLOSE_GAIN;
		ratio = 1.0 - ratio;
		if(ratio < 0.0){
		ratio = 0.0;
	}
	writeToWheels(FULL_SPEED, FULL_SPEED * ratio);
	}
}

void stateMachine::wallFollowRight(int sensorDistance, int center){
	float ratio = 0.0;
	if(sensorDistance < center){ //too far. turn left
		//ratio = (sensorDistance - center)/(float)LEFT_WF_FAR_GAIN;
		ratio = (center - sensorDistance)/(float)RIGHT_WF_FAR_GAIN;
		ratio = 1.0 - ratio;
		if(ratio < 0.0){
			ratio = 0.0;
		}
		writeToWheels(FULL_SPEED * ratio * 1.3, FULL_SPEED * 1.3);
	} else { //too close. turn right
		ratio = (sensorDistance - center)/(float)RIGHT_WF_CLOSE_GAIN;
		//ratio = (center - sensorDistance)/(float)LEFT_WF_CLOSE_GAIN;
		ratio = 1.0 - ratio;
		if(ratio < 0.0){
			ratio = 0.0;
		}
		writeToWheels(FULL_SPEED * 1.3, FULL_SPEED * ratio * 1.3);
	}
}

void stateMachine::wallFollowLeft(int sensorDistance, int center){
	float ratio = 0.0;
	if(sensorDistance < center){ //too far. turn left
		//ratio = (sensorDistance - center)/(float)LEFT_WF_FAR_GAIN;
		ratio = (center - sensorDistance)/(float)LEFT_WF_FAR_GAIN;
		ratio = 1.0 - ratio;
		if(ratio < 0.0){
			ratio = 0.0;
		}
		writeToWheels(FULL_SPEED, FULL_SPEED * ratio);
	} else { //too close. turn right
		ratio = (sensorDistance - center)/(float)LEFT_WF_CLOSE_GAIN;
		//ratio = (center - sensorDistance)/(float)LEFT_WF_CLOSE_GAIN;
		ratio = 1.0 - ratio;
		if(ratio < 0.0){
			ratio = 0.0;
		}
		writeToWheels(FULL_SPEED * ratio, FULL_SPEED);
	}
}

/**
 * wallFollow method
 * created by NH on 4/1/17
 * this method uses the distance sensor in the front left corner of the robot
 * to keep the robot on at a constant distance from the wall
 */
void stateMachine::wallFollow(Robot& theRobot, int center){
  float ratio = 0.0;
  if(theRobot.leftSensorDistance < center){ //turn right
    ratio = (center  - theRobot.leftSensorDistance)/LEFT_WF_FAR_GAIN;
    ratio = 1 - ratio;
    if(ratio < 0.0){
        ratio = 0.0;
    }

    writeToWheels(FULL_SPEED, FULL_SPEED*(ratio));

  }
  else{ //turn left
    ratio = (theRobot.leftSensorDistance - center)/LEFT_WF_CLOSE_GAIN;
    ratio = 1.0 - ratio;
    if(ratio < 0.0){
      ratio = 0.0;
    }
	
    writeToWheels(FULL_SPEED * ratio, FULL_SPEED);

  }
}

/**
 *  accelerate method
 *  created on 4/4/17 by NH
 *  takes robot from speed zero to sixty in time milliseconds, driving straight forward
 *  - This method needs no modification for the other robot
 */

void stateMachine::accelerate(int zero, int sixty, int time){

  int currentSpeed = 0;

  for(int i = 0; i < time/ACCELERATE_STEP_SIZE; i++){
    currentSpeed = map(i, 0, time/ACCELERATE_STEP_SIZE, zero, sixty);
    writeToWheels(0, currentSpeed);
    delay(ACCELERATE_STEP_SIZE);
  }
}