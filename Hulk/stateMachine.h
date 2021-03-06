/*
 * stateMachine.h
 * 
 * This namespace conains methods for executing and advancing the current state.
 * 
 * Author: Nathan Herr
 * Date:   1/21/2017
 * 4/17 - NH removed wallFollowSpain and wallRatioSpain see MOAB_Drop verision for those
 */
#include "Robot.h"
#include "Timer.h"

namespace stateMachine{

  /*
   * Executes the current state the Robot is in.
   */
  void execute(Robot& theRobot);
  
  /*
   * Checks the exit conditions and updates the Robot accordingly.
   */
  void updateState(Robot& theRobot);

  /*
   * Writes a value to the wheels.
   */
  void writeToWheels(int leftSpeed, int rightSpeed);

  
	void earlyStates(Robot& theRobot);
	#ifdef R2_LEFT
		void midStatesLeftBot(Robot& theRobot);
		void endStateLeftBot(Robot& theRobot);
	#else
		void midStatesRightBot(Robot& theRobot);
		void endStateRightBot(Robot& theRobot);
	#endif
  void ejectBarrel(Robot& theRobot);
  void lineFollow(Robot& theRobot, int offset = 0);
  void grabBarrel(Robot& theRobot);
  void handleObstacle(Robot& theRobot, int numLines);
  int getTurnIndex(Robot& theRobot);
  void resetRobot(Robot& theRobot);
  void wallFollow(Robot& theRobot, int center);
  void accelerate(int zero, int sixty, int time); 
  void jiggleBox(Robot& theRobot);
	void comeHome(Robot& theRobot);
}

