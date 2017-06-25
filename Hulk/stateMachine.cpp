/*
   stateMachine.cpp

   Handles the behavior of the Robot for any given state. This file contains methods
   for both carrying out a state and advancing it.

   Author: Nathan Herr, Daschel Fortner
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
		if(theRobot.currentState == 4){
			lineFollow(theRobot, 0, 180);
		}else if(theRobot.currentState == 2){
			lineFollow(theRobot, 0, 100);
		}else if(theRobot.currentState == 25){
			lineFollow(theRobot, 0, 130);
		}else{
			lineFollow(theRobot, 0);
		}
		break;
        
    case LINE_FOLLOW_OFFSET:
		#ifdef R2_LEFT
		  lineFollow(theRobot, -2);
		#else
		  lineFollow(theRobot, 2);
		#endif          
		break;
      
    case LINE_FOLLOW_OFFSET2:
      lineFollow(theRobot, 2);
      break;
      
    case DEPART_SPAIN:
      writeToWheels(FULL_SPEED*6, FULL_SPEED*6);
      break;
      
    case GRAB_BARREL: //using fall through
            grabBarrel(theRobot);
			if(theRobot.currentState == 23){
				writeToWheels(0, 0);
				break;
			}
        case EJECT_BARREL:
            ejectBarrel(theRobot);
            lineFollow(theRobot, 2, 125);
            break;
      
    case GRAB_CORNER_BARREL:
      grabBarrel(theRobot);
      ejectCornerBarrel(theRobot);
      case FIND_CORNER_BARREL:
        writeToWheels(FULL_SPEED*.5, FULL_SPEED*.5);
        break;
      
    case ROUND_A_BOUT:
      grabBarrel(theRobot);
      ejectCornerBarrel(theRobot);
    case ROUND_A_BOUT2:
      digitalWrite(MC_BIN1, LOW);  
      digitalWrite(MC_BIN2, HIGH);
      writeToWheels(FULL_SPEED * 0.5, FULL_SPEED * 0.5);
      break;
      
    case LEFT_TURN:
      writeToWheels(0, FULL_SPEED * 0.6);
      break;
        
    case RIGHT_TURN_SPIN:
      digitalWrite(MC_AIN1, HIGH);  
      digitalWrite(MC_AIN2, LOW);
      writeToWheels(FULL_SPEED * .45, FULL_SPEED* .7);
      break;
      
    case RIGHT_TURN:
		if(theRobot.currentState == 1){
			writeToWheels(FULL_SPEED * 0.6, 0);
		} else {
			writeToWheels(FULL_SPEED, 0);
		}
      break;
    
    case FIND_LINE:
      writeToWheels(LEFT_WHEEL_SPEEDS[9], FULL_SPEED);
      break;
	  
	case FIND_LINE_RIGHT:
      writeToWheels(FULL_SPEED, RIGHT_WHEEL_SPEEDS[4]);
      break;

    case HANDLE_OBSTACLE:
      wallFollow(theRobot, WALL_FOLLOW_CENTER);
      grabBarrel(theRobot);
      break;
    
    case WALL_FOLLOW_LEFT: 
      wallFollowLeft(theRobot, WALL_FOLLOW_CENTER_LEFT);
      break;
      
    case WALL_FOLLOW_FAR: 
      wallFollowLeft(theRobot, WALL_FOLLOW_CENTER_LEFT);
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
	}
}

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
	}else{  
		commonStates(theRobot);
	}
}

void stateMachine::commonStates(Robot& theRobot){
    
    switch (theRobot.currentState) {
      case 0: // WAIT -> DEPART_SPAIN
        resetRobot(theRobot);
        if(theRobot.startSensor < 800){
            theRobot.isStartLightOn = true;
        }
        if (digitalRead(GO_BUTTON) == LOW || (theRobot.isStartLightOn && theRobot.startSensor > 1000)) {
            
            accelerate(0, FULL_SPEED, 300);
            theRobot.writeToServo(theRobot.ARM, ARM_MID);
            //theRobot.currentState++;  //=9 to start from just after first pickup
			theRobot.currentState++;
            theRobot.isStartLightOn = false;
        }
        break;
		
	case 1: // RIGHT_TURN
		if(theRobot.amountSeen > 1 && theRobot.lastLineIndex > 3){
			theRobot.currentState++;
		}
		break;
		
	case 2: // LINE_FOLLOW
		if(theRobot.amountSeen > 3){
			theRobot.currentState++;
			theRobot.oneTimer.set(300);
		}
		break;
		
	case 3: // LEFT_TURN
		if(theRobot.amountSeen > 1 && theRobot.oneTimer.isTimeUpUnset()){
			theRobot.currentState++;
		}
		break;
		
	case 4: // LINE_FOLLOW
		if(theRobot.amountSeen > 4){
			theRobot.currentState++;
			theRobot.oneTimer.set(3100 * TIMING_CONST);
		}
		break;

    case 5:

      if(theRobot.oneTimer.isTimeUpUnset()){
        theRobot.writeToServo(theRobot.ARM, ARM_DOWN);
        theRobot.currentState++;
      }
      break;
      
    case 6:  //FIND_LINE -> LINE_FOLLOW
 
      if(theRobot.amountSeen > 1){
          theRobot.oneTimer.set(300*TIMING_CONST);
          theRobot.currentState++;
      }
      break;
              
    case 7: //LINE_FOLLOW -> LINE_FOLLOW_OFFSET
      if(theRobot.oneTimer.isTimeUpUnset()){
          theRobot.currentState++;
      }
      break;
        
      
    case 8:
    case 21:
      if(theRobot.wallSensorDistance < WALL_TRIGGER){
        theRobot.currentState++;
      }
      break;    

    case 9:
    case 22: // EJECT_BARREL -> GRAB_BARREL
      if(theRobot.clawSensorDistance < CLAW_GRAB_TRIGGER_1_3){
          theRobot.currentState++;
          theRobot.grabTimerInt = millis();
          theRobot.oneTimer.set(300*TIMING_CONST) ;
        }    
        break; 

    case 10: // GRAB_BARREL -> LINE_FOLLOW_OFFSET
   
      if(theRobot.oneTimer.isTimeUpUnset()){
          theRobot.currentState++;
          theRobot.ejectTimer.unset();
          theRobot.oneTimer.unset();
      }     
      
      break;
	  
	case 11: // LINE_FOLLOW -> LINE_FOLLOW
      grabBarrel(theRobot);
      if(!theRobot.oneTimer.isTimerSet() && theRobot.amountSeen > 3) {
          theRobot.currentState++;
          theRobot.oneTimer.set(500*TIMING_CONST);
      }
      break;

    case 12:
      grabBarrel(theRobot);
      if(theRobot.amountSeen > 3 && theRobot.oneTimer.isTimeUpUnset()){
          theRobot.currentState++;
      }
      break;
        
    case 13:
      if(theRobot.clawSensorDistance < CLAW_GRAB_TRIGGER_2){
          theRobot.currentState++;
          theRobot.grabTimerInt = millis();
          theRobot.ejectTimer.unset();
      }
      break;
        
    case 14: // GRAB_CORNER_BARREL -> ROUND_A_BOUT
      theRobot.writeToServo(theRobot.DUMP, DUMP_DOWN); 
      if(theRobot.frontSensorDistance < FRONT_CORNER_TRIGGER){
          theRobot.oneTimer.set(300*TIMING_CONST);
          theRobot.currentState++;
      }
      break;

    case 15: //ROUND_A_BOUT -> ROUND_A_BOUT         
  
      if(theRobot.wallSensorDistance > WALL_TRIGGER && theRobot.amountSeen > 1 && theRobot.oneTimer.isTimeUpUnset()){
          theRobot.currentState++;
          theRobot.oneTimer.set(200*TIMING_CONST);
      }
      break;


    case 16: //ROUND_A_BOUT -> ROUND_A_BOUT         

      if(theRobot.firstLineIndex != 3 && theRobot.oneTimer.isTimeUpUnset()){
          theRobot.currentState++;
      }
      break;


    case 17: //ROUND_A_BOUT -> ROUND_A_BOUT         
      theRobot.writeToServo(theRobot.EJECT, EJECT_BACK_POSITION);
      if(theRobot.firstLineIndex == 3 && theRobot.amountSeen > 1){
          theRobot.currentState++;
      }
      break;

    case 18: //ROUND_A_BOUT -> LINE_FOLLOW
      digitalWrite(MC_BIN1, HIGH);
      digitalWrite(MC_BIN2, LOW);
    case 19: // LINE_FOLLOW_OFFSET -> RIGHT_TURN
      if(!theRobot.oneTimer.isTimerSet()){ 
          theRobot.oneTimer.set(60*TIMING_CONST);
      }
      else if(theRobot.amountSeen > 3 && theRobot.oneTimer.isTimeUpUnset()){
          theRobot.currentState++;
          theRobot.writeToServo(theRobot.ARM, ARM_DOWN);
          theRobot.oneTimer.set(400);
          theRobot.ejectTimer.unset();
      }
      break;      

    case 20: // RIGHT_TURN -> LINE_FOLLOW_OFFSET
      if(theRobot.amountSeen > 1 && theRobot.oneTimer.isTimeUpUnset()) {
          theRobot.currentState++;
      }
      break;

    case 23: // GRAB_BARREL -> ROUND_A_BOUT

      if(theRobot.oneTimer.isTimeUpUnset()) {
          theRobot.currentState++; 
          theRobot.oneTimer.set(100*TIMING_CONST);
          theRobot.writeToServo(theRobot.ARM, ARM_UP);
      }
      break;

    case 24: //ROUND_A_BOUT -> LINE_FOLLOW

      if(theRobot.firstLineIndex == 3 && theRobot.oneTimer.isTimeUpUnset()) {
          theRobot.currentState++;
      }
      break;
    
    case 25: //LINE_FOLLOW -> RIGHT_TURN_SPIN
      digitalWrite(MC_BIN1, HIGH);
      digitalWrite(MC_BIN2, LOW);
      theRobot.writeToServo(theRobot.CLAW, CLAW_OPEN);
      if(theRobot.amountSeen > 3){
          theRobot.oneTimer.set(400 * TIMING_CONST);
          theRobot.currentState++;
      }
      break;

    case 26: //RIGHT_TURN_SPIN -> LINE_FOLLOW
      if(theRobot.oneTimer.isTimeUpUnset()){
          theRobot.currentState++;
          theRobot.writeToServo(theRobot.DUMP, DUMP_DOWN);
          theRobot.oneTimer.set(500);
      }
      break;
  
    case 27: // LINE_FOLLOW -> LINE_FOLLOW
      digitalWrite(MC_AIN1, LOW);
      digitalWrite(MC_AIN2, HIGH);
      if(theRobot.frontSensorDistance > 3600 &&theRobot.oneTimer.isTimeUpUnset()){
        theRobot.oneTimer.set(700 * TIMING_CONST);
        theRobot.currentState++;
      }
      break;
      
    case 28: // LINE_FOLLOW -> FIND_LINE
      if(theRobot.oneTimer.isTimeUpUnset()){
        theRobot.currentState++;
      }
      break;
  
    case 29: // FIND_LINE -> WALL_FOLLOW_LEFT  //not actually finding a line
      theRobot.writeToServo(theRobot.ARM, ARM_START - 10);
      if (theRobot.frontSensorDistance < 3400){
        theRobot.currentState++;
        theRobot.oneTimer.set(2900 * TIMING_CONST);
      }
      break;
        
      
    case 30:
      if(theRobot.oneTimer.isTimeUpUnset()){
        theRobot.currentState++;
      }
      break;
      
    case 31:  // FIND_LINE_RIGHT -> LINE_FOLLOW
      if(theRobot.firstLineIndex == 3){
          theRobot.currentState++;
          theRobot.oneTimer.set(130*TIMING_CONST);
      }
      break;
        
    case 32: // LINE_FOLLOW -> RIGHT_TURN
      if(theRobot.amountSeen > 3){
          theRobot.currentState++;
          theRobot.oneTimer.set(200*TIMING_CONST);
      }
      break;
        
    case 33: // RIGHT_TURN -> LINE_FOLLOW
        if(theRobot.firstLineIndex == 3 && theRobot.oneTimer.isTimeUpUnset()){
          theRobot.currentState++;
          theRobot.oneTimer.set(570);
        }
        break; 

    case 34: // LINE_FOLLOW -> DUMP_BARRELS
        if(theRobot.frontSensorDistance < 3550 && theRobot.oneTimer.isTimeUp()){
          theRobot.currentState++;
        }
        break;

    case 35:
      theRobot.writeToServo(theRobot.ARM, ARM_MID);
			theRobot.writeToServo(theRobot.DUMP, (DUMP_UP - DUMP_DOWN) * 5 / 8 + DUMP_DOWN);
			delay(500);
			for(int i = (DUMP_UP - DUMP_DOWN) * 3 / 4 + DUMP_DOWN; i < DUMP_UP; i++){
				theRobot.writeToServo(theRobot.DUMP, i);
				delay(30);
			}
			delay(500);
			theRobot.writeToServo(theRobot.DUMP, DUMP_DOWN * 3 / 4);
			delay(500);
			theRobot.writeToServo(theRobot.DUMP, (DUMP_UP - DUMP_DOWN) * 5 / 8 + DUMP_DOWN);
			delay(500);
			for(int i = (DUMP_UP - DUMP_DOWN) * 3 / 4 + DUMP_DOWN; i < DUMP_UP; i++){
				theRobot.writeToServo(theRobot.DUMP, i);
				delay(30);
			}
			theRobot.writeToServo(theRobot.DUMP, (DUMP_UP + DUMP_DOWN) / 2);
			delay(500);
      while(true) {
        theRobot.writeToServo(theRobot.DUMP, 5 * (DUMP_UP + DUMP_DOWN) / 8);
        delay(500);
        theRobot.writeToServo(theRobot.DUMP, 3 * (DUMP_UP + DUMP_DOWN) / 4);
        delay(500);
        theRobot.writeToServo(theRobot.DUMP, 7 * (DUMP_UP + DUMP_DOWN) / 8);
        delay(500);
        theRobot.writeToServo(theRobot.DUMP, DUMP_UP);
        delay(1000);
        theRobot.writeToServo(theRobot.DUMP, (DUMP_UP + DUMP_DOWN) / 4);
        delay(1000);
        theRobot.writeToServo(theRobot.DUMP, DUMP_UP);
        delay(1000);
      }
			
			theRobot.currentState = 0;
			break;
    }
}   //end stateMachine::commonStates        

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
	
	if(theRobot.currentState > 18){
		if(!theRobot.ejectTimer.isTimerSet()) {
			theRobot.ejectTimer.set(KICKER_MOVE_BACK_TIME * 2.0);
		}
		else if (theRobot.ejectTimer.isTimeUp()){
			theRobot.writeToServo(theRobot.EJECT, EJECT_BACK_POSITION);
		}
		else if (theRobot.ejectTimer.timeElapsed() > KICKER_MOVE_BACK_TIME * .5) {
			theRobot.writeToServo(theRobot.EJECT, EJECT_FRONT_POSITION);
		}
		else if (theRobot.ejectTimer.timeElapsed() < KICKER_MOVE_BACK_TIME * .5) {
			theRobot.writeToServo(theRobot.EJECT, EJECT_BACK_POSITION);
		}
	}else{
		if(!theRobot.ejectTimer.isTimerSet()) {
			theRobot.ejectTimer.set(KICKER_MOVE_BACK_TIME*2);
		}
		else if (theRobot.ejectTimer.isTimeUp()){
			theRobot.writeToServo(theRobot.EJECT, EJECT_BACK_POSITION);
		}
		else if (theRobot.ejectTimer.timeElapsed() > KICKER_MOVE_BACK_TIME) {
			theRobot.writeToServo(theRobot.EJECT, EJECT_FRONT_POSITION);
		}
		else if (theRobot.ejectTimer.timeElapsed() < KICKER_MOVE_BACK_TIME) {
			theRobot.writeToServo(theRobot.EJECT, EJECT_BACK_POSITION);
		}
	}
}

void stateMachine::ejectCornerBarrel(Robot& theRobot) {

  if(!theRobot.ejectTimer.isTimerSet()) {
    theRobot.ejectTimer.set(300);
  }

  if (theRobot.ejectTimer.isTimeUpUnset() ) {
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
  if (theRobot.grabTimerInt + 50 > millis()) { //initial call
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
    writeToWheels(LEFT_WHEEL_SPEEDS[index]*((float)fullSpeed/(float)FULL_SPEED), 
            RIGHT_WHEEL_SPEEDS[index]*((float)fullSpeed/(float)FULL_SPEED));    

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
 * wallFollow method
 * created by NH on 4/1/17
 * this method uses the distance sensor in the front left corner of the robot
 * to keep the robot on at a constant distance from the wall 
 */
void stateMachine::wallFollow(Robot& theRobot, int center, int fullSpeed){
  float ratio = 0.0;
  if(theRobot.wallSensorDistance < center){ //turn right
    ratio = (center  - theRobot.wallSensorDistance)/WALL_FOLLOW_FAR_GAIN;
    ratio = 1 - ratio;
    if(ratio < 0.0){
        ratio = 0.0;
    }       
    
    writeToWheels(fullSpeed * ratio, fullSpeed);
  } 
  else{ //turn left
    ratio = (theRobot.wallSensorDistance - center)/WALL_FOLLOW_CLOSE_GAIN;
    ratio = 1.0 - ratio; 
    if(ratio < 0.0){
      ratio = 0.0;
    }
  
    writeToWheels(fullSpeed, fullSpeed * (ratio));
  } 
}

void stateMachine::wallFollowLeft(Robot& theRobot, int center){
  float ratio = 0.0;
  if(theRobot.frontSensorDistance < center){ //turn right
    ratio = (center  - theRobot.frontSensorDistance)/WALL_FOLLOW_FAR_GAIN;
    ratio = 1 - ratio;
    if(ratio < 0.0){
        ratio = 0.0;
    }       
    
    writeToWheels(FULL_SPEED, FULL_SPEED*(ratio));
  } 
  else{ 
    ratio = (theRobot.frontSensorDistance - center)/WALL_FOLLOW_CLOSE_GAIN;
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
    writeToWheels(currentSpeed, 0);
    delay(ACCELERATE_STEP_SIZE);
  }  
}


/**
 *  method jiggleBox
 *  created 4/8/17 - NH
 *  this method moves the box up and down just slightly
 *  This method is intended to be called in the time between
 *  when a barrel is dropped into the box and when the next barrel 
 *  is grabbed
 *  -This function should be the same for both robots
 */
void stateMachine::jiggleBox(Robot& theRobot){
		theRobot.jiggleTimer.isTimeUpUnset();
		
        if(!theRobot.jiggleTimer.isTimerSet()){
            theRobot.jiggleTimer.set(JIGGLE_TIME_PERIOD);
        }       
		int time = theRobot.jiggleTimer.timeElapsed();
		int nextPosition; 
		if(time < JIGGLE_TIME_PERIOD/2){
			nextPosition = DUMP_DOWN; 
		}
		else{
			nextPosition = DUMP_DOWN + 47;
		}
        theRobot.writeToServo(theRobot.DUMP, nextPosition);
}

void stateMachine::errorCheck(Robot& theRobot){
  if(!theRobot.errorTimer.isTimerSet()){
    theRobot.errorTimer.set(3000);
  }
  
  if(theRobot.errorTimer.isTimeUp()) {
      digitalWrite(MC_BIN1, LOW);
      digitalWrite(MC_BIN2, HIGH); 
      writeToWheels(FULL_SPEED*.6, 0);
      delay(2500);
      digitalWrite(MC_BIN1, HIGH);
      digitalWrite(MC_BIN2, LOW); 
      theRobot.currentState = 4;
  }
}

