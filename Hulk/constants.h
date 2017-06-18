#ifndef CONSTANTS_H
#define CONSTANTS_H

#ifndef R2_LEFT
#define R2_LEFT 3
	const int FULL_SPEED = 140; // was 150
#endif

	enum State {
		WAIT               = 0,
		LINE_FOLLOW        = 1,
		RIGHT_TURN         = 2,
		HANDLE_OBSTACLE    = 3,
		LINE_FOLLOW_OFFSET = 4, //offset left
		GRAB_BARREL        = 5,
		LEFT_TURN          = 6,
		EJECT_BARREL       = 7,
		FIND_CORNER_BARREL = 8,
		ROUND_A_BOUT       = 9,
		GRAB_CORNER_BARREL = 10,
		FIND_LINE          = 11,
		DEPART_SPAIN       = 12,
		LINE_FOLLOW_OFFSET2= 13, //offset right
		DUMP_BARRELS       = 14,
		WALL_FOLLOW_FAR    = 15,
		LEFT_TURN_SPIN	   = 17,
        WALL_FOLLOW_RIGHT  = 18,
        VEER_LEFT          = 19
	};


	const int RIGHT_WHEEL_SPEEDS[15] = {
		FULL_SPEED * .05,
		FULL_SPEED * .2,
		FULL_SPEED * .3,
		FULL_SPEED * .45,
		FULL_SPEED * .55,
		FULL_SPEED * .65,
		FULL_SPEED * .8,  //6
		FULL_SPEED, FULL_SPEED, FULL_SPEED, FULL_SPEED,
		FULL_SPEED, FULL_SPEED, FULL_SPEED, FULL_SPEED
	};

	const int LEFT_WHEEL_SPEEDS[15] = {
		FULL_SPEED, FULL_SPEED, FULL_SPEED, FULL_SPEED,
		FULL_SPEED, FULL_SPEED, FULL_SPEED, FULL_SPEED,
		FULL_SPEED * .8, //8
		FULL_SPEED * .65,
		FULL_SPEED * .55,
		FULL_SPEED * .45,
		FULL_SPEED * .3,
		FULL_SPEED * .2,
		FULL_SPEED * .05,
	};

	/******************************************\
	| Left Robot Constants                     |
	| R2 constants                             |
	\******************************************/

	const float TIMING_CONST = .9555555; // Old value .833333
	
	//servos
	const int EJECT_FRONT_POSITION = 88;
	const int EJECT_BACK_POSITION = 58;
	const int CLAW_OPEN = 80;
	const int CLAW_CLOSED = 38; // was 42
	const int ARM_DOWN = 155;  // was 130
	const int ARM_UP   = 5;
	const int ARM_START = 25;
	const int ARM_MID = 55;
	const int DUMP_UP = 125;
	const int DUMP_DOWN = 40;

	//sensor trigger distances
	const int CLAW_GRAB_TRIGGER_1_3	= 2500;
	const int CLAW_GRAB_TRIGGER_2  	= 2100;
	const int WALL_TRIGGER          = 3750; //was 3550
	const int FRONT_CORNER_TRIGGER  = 2500;

	const int FRONT_HOME            = 3000;
	const int WALL_HOME             = 3200; // was 3000

	//wall following constants
	const int WALL_FOLLOW_CENTER  	  = 3650;
	const int WALL_FOLLOW_CENTER_LEFT = 3740;
	const int LEFT_WF_CLOSE_GAIN      = 370;  //too far right
	const int LEFT_WF_FAR_GAIN        = 220;  //too far left
	const int RIGHT_WF_CLOSE_GAIN     = 600;
	const int RIGHT_WF_FAR_GAIN       = 400;
	
	//spain wall following constants
	const int WALL_F_FRONT_FAR_GAIN   = 370;
	const int WALL_F_FRONT_CLOSE_GAIN = 220;
	const int WALL_F_FRONT_CENTER     = 2700;


	//Timing constants
	const int ACCELERATE_STEP_SIZE = 	10;
	const int KICKER_MOVE_BACK_TIME = 250;
	const int JIGGLE_TIME_PERIOD		= 200;

	const int stateMap[] = {
		WAIT,                 //start sequence
		LEFT_TURN,
		LINE_FOLLOW,          //2
		RIGHT_TURN,
		LINE_FOLLOW,          //4

		HANDLE_OBSTACLE,      //obstacle sequence
		FIND_LINE,
		LINE_FOLLOW_OFFSET2,
		LINE_FOLLOW_OFFSET,
		EJECT_BARREL,         //first barrel eject/pickup

		GRAB_BARREL,
		LINE_FOLLOW,   //11
		LINE_FOLLOW_OFFSET,
		FIND_CORNER_BARREL,
		GRAB_CORNER_BARREL,   //14

		ROUND_A_BOUT,
		ROUND_A_BOUT,
		ROUND_A_BOUT,
		LINE_FOLLOW,
		LEFT_TURN,            //19

		LINE_FOLLOW_OFFSET, //20
		EJECT_BARREL,   //21
		GRAB_BARREL,
		ROUND_A_BOUT,
		LINE_FOLLOW,

		LEFT_TURN,    // 25
		LINE_FOLLOW,  // 26
		LINE_FOLLOW,
		LINE_FOLLOW,
		WALL_FOLLOW_RIGHT,
		WALL_FOLLOW_RIGHT, //30
		VEER_LEFT,         //31
		LINE_FOLLOW,
		LEFT_TURN,         //33
		LINE_FOLLOW_OFFSET,
		WAIT
	};

#endif
