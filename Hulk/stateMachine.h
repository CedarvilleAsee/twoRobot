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

	void commonStates(Robot& theRobot);
	void ejectBarrel(Robot& theRobot);
	void ejectCornerBarrel(Robot& theRobot);
	void lineFollow(Robot& theRobot, int offset, int fullSpeed = 150);
	void grabBarrel(Robot& theRobot);
	void handleObstacle(Robot& theRobot, int numLines);
	int getTurnIndex(Robot& theRobot);
	void resetRobot(Robot& theRobot);
	void wallFollow(Robot& theRobot, int center, int fullSpeed = 150);
	void accelerate(int zero, int sixty, int time); 
	void jiggleBox(Robot& theRobot);
	void wallFollowLeft(Robot& theRobot, int center);
	void errorCheck(Robot& theRobot);
}

