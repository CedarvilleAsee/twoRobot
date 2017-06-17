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
      lineFollow(theRobot, 0);
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
        case EJECT_BARREL:
            ejectBarrel(theRobot);
            #ifdef R2_LEFT
                lineFollow(theRobot, -2);
            #else
                lineFollow(theRobot, 2, 125);
            #endif
      break;
      
    case GRAB_CORNER_BARREL:
      grabBarrel(theRobot);
            ejectCornerBarrel(theRobot);
        case FIND_CORNER_BARREL:
                writeToWheels(FULL_SPEED*.5, FULL_SPEED*.5);
        jiggleBox(theRobot);
      break;
      
    case ROUND_A_BOUT:
            #ifdef R2_LEFT
                digitalWrite(MC_AIN1, HIGH);  
                digitalWrite(MC_AIN2, LOW);
                writeToWheels(FULL_SPEED * 0.5, FULL_SPEED * 0.5);
            #else
                digitalWrite(MC_BIN1, LOW);  
                digitalWrite(MC_BIN2, HIGH);
                writeToWheels(FULL_SPEED * 0.5, FULL_SPEED * 0.5);
                grabBarrel(theRobot);
            #endif

            ejectCornerBarrel(theRobot);
      break;
      
    case LEFT_TURN:
      writeToWheels(0, FULL_SPEED);
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
      wallFollow(theRobot, WALL_FOLLOW_CENTER);
      grabBarrel(theRobot);
      break;
    
    case WALL_FOLLOW_FAR:
      wallFollow(theRobot, WALL_FOLLOW_CENTER_LEFT);
      break;
            
      
        case COME_HOME:
            comeHome(theRobot);
            break;
            
    case DUMP_BARRELS:
      theRobot.writeToServo(theRobot.DUMP, DUMP_UP);
      writeToWheels(0, 0);
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
  }
    else{
        commonStates(theRobot);
        #ifdef R2_LEFT
            midStatesLeftBot(theRobot);
            endStateLeftBot(theRobot);
        #else
            midStatesRightBot(theRobot);
        #endif
    }

}

void stateMachine::commonStates(Robot& theRobot){
    
    switch (theRobot.currentState) {
        case 0: // WAIT -> DEPART_SPAIN
			jiggleBox(theRobot);
            //resetRobot(theRobot);
            if(theRobot.startSensor < 800){
                theRobot.isStartLightOn = true;
            }
            if (digitalRead(GO_BUTTON) == LOW || (theRobot.isStartLightOn && theRobot.startSensor > 1000)) {
                accelerate(0, FULL_SPEED, 300);
                theRobot.writeToServo(theRobot.ARM, ARM_MID);
                theRobot.currentState++; //=9 to start from just after first pickup
                theRobot.isStartLightOn = false;
            }
            break;

    #ifdef R2_LEFT  
        case 2: //HANDLE_OBSTACLE -> FIND_LINE
                if(!theRobot.oneTimer.isTimerSet() && theRobot.amountSeen > 4){
                    theRobot.oneTimer.set(2100*TIMING_CONST);
                }
    #else //D2_RIGHT
        case 5:
    #endif
      if(theRobot.oneTimer.isTimeUpUnset()){
        theRobot.writeToServo(theRobot.ARM, ARM_DOWN);
        theRobot.currentState++;
      }
      break;
        #ifdef R2_LEFT
            case 3: //FIND_LINE -> LINE_FOLLOW
        #else //D2_RIGHT
            case 6:
        #endif
            if(theRobot.amountSeen > 1){
                theRobot.oneTimer.set(400*TIMING_CONST);
                theRobot.currentState++;
            }
            break;

        case 30: //LINE_FOLLOW -> LINE_FOLLOW_OFFSET
        #ifdef R2_LEFT
            case 4:  //LINE_FOLLOW -> LINE_FOLLOW_OFFSET
        #else //D2_RIGHT
            case 7:
        #endif
            if(theRobot.oneTimer.isTimeUpUnset()){
                theRobot.currentState++;
            }
			else if(theRobot.wallSensorDistance < WALL_TRIGGER){
				theRobot.currentState += 2;
				theRobot.oneTimer.unset();
			}
            break;
        
        
        #ifdef R2_LEFT
            case 5: //LINE_FOLLOW_OFFSET -> EJECT_BARREL
            case 17:    //LINE_FOLLOW_OFFSET -> EJECT_BARREL
        #else
            case 8:
            case 21:
        #endif
            if(theRobot.wallSensorDistance < WALL_TRIGGER){
                theRobot.currentState++;
            }
            break;    

        #ifdef R2_LEFT
            case 6: // EJECT_BARREL -> GRAB_BARREL
            case 18: // EJECT_BARREL -> GRAB_BARREL
        #else //D2_RIGHT
            case 9:
            case 22: // EJECT_BARREL -> GRAB_BARREL
        #endif
        
            if(theRobot.clawSensorDistance < CLAW_GRAB_TRIGGER_1_3){
                theRobot.currentState++;
                theRobot.grabTimerInt = millis();
                theRobot.oneTimer.set(1000*TIMING_CONST) ;
            }    
            break; 
        #ifdef R2_LEFT  
            case 7: // GRAB_BARREL -> LINE_FOLLOW_OFFSET
        #else   //D2_RIGHT
            case 10:
        #endif
            if(theRobot.oneTimer.isTimeUpUnset()){
                theRobot.currentState++;
                theRobot.ejectTimer.unset();
                theRobot.oneTimer.unset();
            }     
            
            break;

        #ifdef R2_LEFT
            case 15:   // LINE_FOLLOW -> LINE_FOLLOW
            case 8:    // LINE_FOLLOW_OFFSET -> LINE_FOLLOW_OFFSET
        #else 
            case 11: // LINE_FOLLOW -> LINE_FOLLOW
        #endif  
        
            jiggleBox(theRobot);
            if(!theRobot.oneTimer.isTimerSet() && theRobot.amountSeen > 3) {
                theRobot.currentState++;
                theRobot.oneTimer.set(500*TIMING_CONST);
            }
            break;

        #ifdef R2_LEFT
            case 9:   // LINE_FOLLOW_OFFSET -> FIND_CORNER_BARREL
        #else // D2_RIGHT
            case 12:
        #endif  
            jiggleBox(theRobot);
            if(theRobot.amountSeen > 3 && theRobot.oneTimer.isTimeUpUnset()){
                theRobot.currentState++;
            }
            break;
            
        #ifdef R2_LEFT
            case 10: //FIND_CORNER_BARREL -> GRAB_CORNER_BARREL
        #else   // D2_RIGHT
            case 13:
        #endif
            if(theRobot.clawSensorDistance < CLAW_GRAB_TRIGGER_2){
                theRobot.currentState++;
                theRobot.grabTimerInt = millis();
                theRobot.ejectTimer.unset();
            }
            break;
            
        #ifdef R2_LEFT
            case 11: // GRAB_CORNER_BARREL -> ROUND_A_BOUT
        #else  // D2_RIGHT
            case 14:
        #endif
            theRobot.writeToServo(theRobot.DUMP, DUMP_DOWN); 
            if(theRobot.frontSensorDistance < FRONT_CORNER_TRIGGER){
                theRobot.oneTimer.set(300*TIMING_CONST);
                theRobot.currentState++;
            }
            break;

        #ifdef R2_LEFT  
            case 12: //ROUND_A_BOUT -> ROUND_A_BOUT
        #else // D2_RIGHT
            case 15: //ROUND_A_BOUT -> ROUND_A_BOUT         
        #endif
            if(theRobot.wallSensorDistance > WALL_TRIGGER && theRobot.amountSeen > 1 && theRobot.oneTimer.isTimeUpUnset()){
                theRobot.currentState++;
                theRobot.oneTimer.set(200*TIMING_CONST);
            }
      break;

        #ifdef R2_LEFT  
            case 13: //ROUND_A_BOUT -> ROUND_A_BOUT
        #else // D2_RIGHT
            case 16: //ROUND_A_BOUT -> ROUND_A_BOUT         
        #endif
            if(theRobot.firstLineIndex != 3 && theRobot.oneTimer.isTimeUpUnset()){
                theRobot.currentState++;
            }
      break;

        #ifdef R2_LEFT  
            case 14: //ROUND_A_BOUT -> ROUND_A_BOUT
        #else // D2_RIGHT
            case 17: //ROUND_A_BOUT -> ROUND_A_BOUT         
            theRobot.writeToServo(theRobot.EJECT, EJECT_BACK_POSITION);
        #endif
            if(theRobot.firstLineIndex == 3 && theRobot.amountSeen > 1){
                theRobot.currentState++;
            }
     break;

            #ifdef R2_LEFT
                case 15:   //LINE_FOLLOW -> LEFT_TURN
                    digitalWrite(MC_AIN1, LOW);
                    digitalWrite(MC_AIN2, HIGH);
            #else
                case 18: //ROUND_A_BOUT -> LINE_FOLLOW
                    digitalWrite(MC_BIN1, HIGH);
                    digitalWrite(MC_BIN2, LOW);
                case 19: // LINE_FOLLOW_OFFSET -> RIGHT_TURN
            #endif
            
            if(!theRobot.oneTimer.isTimerSet()){ 
                theRobot.oneTimer.set(60*TIMING_CONST);
            }
            else if(theRobot.amountSeen > 3 && theRobot.oneTimer.isTimeUpUnset()){
                theRobot.currentState++;
                theRobot.writeToServo(theRobot.ARM, ARM_DOWN);
            }
            break;      

        #ifdef R2_LEFT
            case 16: // LEFT_TURN -> LINE_FOLLOW_OFFSET  
        #else
            case 20: // RIGHT_TURN -> LINE_FOLLOW_OFFSET
        #endif

            if(!theRobot.oneTimer.isTimerSet()){
                theRobot.ejectTimer.unset();
                theRobot.oneTimer.set(400);
            }
            else if(theRobot.amountSeen > 1 && theRobot.oneTimer.isTimeUpUnset()) {
                theRobot.currentState++;
            }
            break;
        #ifdef R2_LEFT
            case 19: // GRAB_BARREL -> RIGHT_TURN
        #else   //D2_RIGHT
            case 23: // GRAB_BARREL -> LEFT_TURN_SPIN
        #endif
            if(theRobot.amountSeen > 4) {
                theRobot.currentState++; 
                theRobot.oneTimer.set(100*TIMING_CONST);
                theRobot.writeToServo(theRobot.ARM, ARM_UP);
            }
            break;

        #ifdef R2_LEFT
            case 20: //RIGHT_TURN -> LINE_FOLLOW
        #else //D2_RIGHT
            case 24: //LEFT_TURN -> LINE_FOLLOW
                digitalWrite(MC_BIN1, HIGH);
                digitalWrite(MC_BIN2, LOW);
        #endif
            if(theRobot.firstLineIndex == 3 && theRobot.oneTimer.isTimeUpUnset()) {
                theRobot.currentState++;
            }
            break;
        
        #ifdef R2_LEFT
            case 22:   //LINE_FOLLOW -> LINE_FOLLOW
        #else
            case 25:
        #endif
            jiggleBox(theRobot);
            theRobot.writeToServo(theRobot.CLAW, CLAW_OPEN);
            if(theRobot.wallSensorDistance > WALL_TRIGGER + 50){
                theRobot.oneTimer.set(400 * TIMING_CONST);
                theRobot.currentState++;
            }
            break;

        #ifdef R2_LEFT
            case 23:   //LINE_FOLLOW -> HANDLE_OBSTACLE
        #else
            case 26:
        #endif
            if(theRobot.oneTimer.isTimeUp()){
                theRobot.currentState++;
                theRobot.writeToServo(theRobot.DUMP, DUMP_DOWN);
            }
            break;

        case 29: //FIND_LINE -> LINE_FOLLOW
            theRobot.writeToServo(theRobot.ARM, ARM_DOWN);
            if(theRobot.amountSeen > 1){
                theRobot.currentState++;
                theRobot.oneTimer.set(500 * TIMING_CONST);
            }
            break;
            
        case 31:  // LINE_FOLLOW_OFFSET2 -> LEFT_TURN_SPIN
            if(theRobot.amountSeen > 3){
                theRobot.currentState++;
                theRobot.oneTimer.set(130*TIMING_CONST);
            }
            break;
            
        case 32: // LEFT_TURN_SPIN -> LINE_FOLLOW_OFFSET
            if(theRobot.firstLineIndex == 3 && theRobot.oneTimer.isTimeUpUnset()){
                theRobot.currentState++;
            }
            break;
            
        case 33: // LINE_FOLLOW_OFFSET -> LINE_FOLLOW_OFFSET
            if(theRobot.wallSensorDistance < WALL_TRIGGER){
                theRobot.currentState++;
                theRobot.oneTimer.set(120*TIMING_CONST);
            }
            break; 

        case 34: // LINE_FOLLOW_OFFSET -> ROUND_A_BOUT
            if(theRobot.oneTimer.isTimeUpUnset()){
                theRobot.currentState++;
            }
            break;

        case 35: // ROUND_A_BOUT -> DEPART_SPAIN
            digitalWrite(MC_BIN1, HIGH);
            digitalWrite(MC_BIN2, LOW);
            if(theRobot.frontSensorDistance < 3550){
                theRobot.currentState++;
            }
            break;
        
        case 36: //DEPART_SPAIN -> DUMP_BARRELS
            if(theRobot.wallSensorDistance < 2000){
                theRobot.currentState++;
            }
            break;
        
    }
}   //end stateMachine::commonStates

#ifdef R2_LEFT

        void stateMachine::endStateLeftBot(Robot& theRobot){
            switch (theRobot.currentState) {
                case 1: //DEPART_SPAIN -> HANDLE_OBSTACLE
                    if(theRobot.wallSensorDistance < 3400){
                        theRobot.currentState++;
                    }
                    break;

                case 21:  //LINE_FOLLOW -> LINE_FOLLOW
                    jiggleBox(theRobot);
                    theRobot.writeToServo(theRobot.DUMP, DUMP_DOWN);
                    if(theRobot.wallSensorDistance < WALL_TRIGGER){
                        theRobot.currentState++;
                    }
                    break;

                case 24:  //WALL_FOLLOW -> WALL_FOLLOW_FAR
                    if(theRobot.amountSeen == 0){
                        theRobot.writeToServo(theRobot.CLAW, CLAW_OPEN);
                        theRobot.oneTimer.set(500 * TIMING_CONST);  
                        theRobot.currentState++;
                    }
                    break;
                    
                case 25:   //WALL_FOLLOW_FAR -> WALL_FOLLOW_FAR
                    jiggleBox(theRobot);
                    if(theRobot.oneTimer.isTimeUp()){
                        theRobot.currentState++;
                        theRobot.oneTimer.set(3000 * TIMING_CONST);
                    }
                    break;

                                        
                case 26: //WALL_FOLLOW_FAR -> FIND_LINE
                    jiggleBox(theRobot);
                    if(theRobot.amountSeen > 1 && theRobot.oneTimer.isTimeUp()){
                        theRobot.oneTimer.set(2000 * TIMING_CONST);
                    }
                    
                    if(theRobot.oneTimer.isTimeUpUnset()){
                        theRobot.currentState++;
                    }
                    break;
            }   
        }//  end  stateMachine::endStateLeftBot
        
        
#else
        
        void stateMachine::midStatesRightBot(Robot& theRobot){
            switch (theRobot.currentState) {
                case 1: //RIGHT_TURN -> LINE_FOLLOW
            
                    if(theRobot.amountSeen > 1){
                        theRobot.currentState++;
                    }
                    break;
                
                case 2: //LINE_FOLLOW -> LEFT_TURN
                    if(theRobot.amountSeen > 3){
                        theRobot.currentState++;
                        theRobot.oneTimer.set(300 * TIMING_CONST);
                    }
                    break;

                case 3: // LEFT_TURN -> LINE_FOLLOW
                    if(theRobot.amountSeen > 1 && theRobot.oneTimer.isTimeUpUnset()){
                        theRobot.currentState++;
                    }
                    break;

                case 4: // LINE_FOLLOW -> HANDLE_OBSTACLE
                    if(theRobot.amountSeen > 3){
                        theRobot.currentState++;
                        theRobot.oneTimer.set(3000 * TIMING_CONST);
                    }
                    
                    case 27: // HANDLE_OBSTACLE -> HANDLE_OBSTACLE
                        if(theRobot.amountSeen > 4){
                            theRobot.oneTimer.set(1200 * TIMING_CONST);
                            theRobot.currentState++;
                        }
                        break;
                    case 28: // HANDLE_OBSTACLE -> FIND_LINE
                        if(theRobot.oneTimer.isTimeUpUnset()){
                            theRobot.currentState++;
                        }
                        break;
            }
        } // end  midStatesRightBot
        
#endif // end D2_RIGHT

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

  if(!theRobot.ejectTimer.isTimerSet()) {
            theRobot.ejectTimer.set(KICKER_MOVE_BACK_TIME);
  }

  if (theRobot.ejectTimer.isTimeUp()) {
    theRobot.writeToServo(theRobot.EJECT, EJECT_BACK_POSITION);
  }
  else {
        theRobot.writeToServo(theRobot.EJECT, EJECT_FRONT_POSITION);
  }

}

void stateMachine::ejectCornerBarrel(Robot& theRobot) {

  if(!theRobot.ejectTimer.isTimerSet()) {
            theRobot.ejectTimer.set(200);
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
void stateMachine::wallFollow(Robot& theRobot, int center){
  float ratio = 0.0;
  if(theRobot.wallSensorDistance < center){ //turn right
    ratio = (center  - theRobot.wallSensorDistance)/WALL_FOLLOW_FAR_GAIN;
    ratio = 1 - ratio;
    if(ratio < 0.0){
        ratio = 0.0;
    }       
    
    #ifdef R2_LEFT
      writeToWheels(FULL_SPEED, FULL_SPEED*(ratio));
    #else
      writeToWheels(FULL_SPEED * ratio, FULL_SPEED);
    #endif
  } 
  else{ //turn left
    ratio = (theRobot.wallSensorDistance - center)/WALL_FOLLOW_CLOSE_GAIN;
    ratio = 1.0 - ratio; 
    if(ratio < 0.0){
      ratio = 0.0;
    }
  
    #ifdef R2_LEFT
      writeToWheels(FULL_SPEED * ratio, FULL_SPEED);
    #else
      writeToWheels(FULL_SPEED, FULL_SPEED*(ratio));
    #endif
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



/*  method comeHome
 *  4/14/17 - NH
 *  this method is supposed to help navigate the robot from the last corner
 *  to Spain. It does this by making and initial right turn, then a left turn, then drives straight.
 *  
 *  Currently uses only timing. :(
 *
 */ 
void stateMachine::comeHome(Robot& theRobot){
    if(!theRobot.oneTimer.isTimerSet()){
        theRobot.oneTimer.set(1150);
    }

    float timeElapsed = (float)theRobot.oneTimer.timeElapsed();
    float speedProportion = 1 - timeElapsed/COME_HOME_TIME;

        
    if(speedProportion < 0.6){
        speedProportion = 0.6;
    }
    int topSpeed = speedProportion * FULL_SPEED;

    float ratio = 0.0;
    
    if(timeElapsed < 390){//initial turn
        ratio = timeElapsed/750 + 0.3;
        ratio = (1 - ratio);
        #ifdef R2_LEFT
            writeToWheels(topSpeed, topSpeed * ratio);
        #else // D2_RIGHT
            writeToWheels(topSpeed* ratio, topSpeed);
        #endif
    }
    else if(timeElapsed < 830){//opposite turn
        ratio = (timeElapsed - 390)/850 + 0.3;
        ratio = (1 - ratio);

        #ifdef R2_LEFT
            writeToWheels(topSpeed * ratio, topSpeed);
        #else // D2_RIGHT
            writeToWheels(topSpeed, topSpeed * ratio);
        #endif
    }
    else{                   

    }   
}