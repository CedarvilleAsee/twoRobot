#ifndef CONSTANTS_H
#define CONSTANTS_H

#ifndef R2_LEFT
//#define R2_LEFT 3
	//const int FULL_SPEED = 140; // was 150
#endif


#ifndef D2_RIGHT
#define D2_RIGHT 1
	const int FULL_SPEED = 150;
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
		COME_HOME		       = 16,
		RIGHT_TURN_SPIN	   = 17,
		WALL_FOLLOW_LEFT   = 18,
		FIND_LINE_RIGHT    = 19,
		ROUND_A_BOUT2      = 20
	};


  const int RIGHT_WHEEL_SPEEDS[15] = {
    FULL_SPEED * .05,
    FULL_SPEED * .2,
    FULL_SPEED * .3,
    FULL_SPEED * .45,
    FULL_SPEED * .6, // .55
    FULL_SPEED * .7,  //.65
    FULL_SPEED * .85,  //.8
    FULL_SPEED, FULL_SPEED, FULL_SPEED, FULL_SPEED, 
    FULL_SPEED, FULL_SPEED, FULL_SPEED, FULL_SPEED
  };

  const int LEFT_WHEEL_SPEEDS[15] = {
    FULL_SPEED, FULL_SPEED, FULL_SPEED, FULL_SPEED, 
    FULL_SPEED, FULL_SPEED, FULL_SPEED, FULL_SPEED, 
    FULL_SPEED * .85, //.8
    FULL_SPEED * .7, //.65
    FULL_SPEED * .6, //.55
    FULL_SPEED * .45,
    FULL_SPEED * .3,
    FULL_SPEED * .2,
    FULL_SPEED * .05,	
  };


  /******************************************\
  | Right Robot Constants                    |
  | D2 constants                             |
  \******************************************/
  const float TIMING_CONST = .8333333332;
  const int EJECT_FRONT_POSITION  = 115;
  const int EJECT_BACK_POSITION   = 85;
  const int CLAW_OPEN 			= 55;
  const int CLAW_CLOSED 			= 85;  
  const int ARM_DOWN 				= 5;  
  const int ARM_UP 				= 140;  
  const int ARM_START				= 110;
  const int ARM_MID				= 70;
  const int DUMP_UP 				= 92;
  const int DUMP_DOWN 			= 10;

  //spain wall following constants
  const int WALL_F_FRONT_FAR_GAIN   = 320;
  const int WALL_F_FRONT_CLOSE_GAIN = 270;
  const int WALL_F_FRONT_CENTER     = 2700;

  //sensor trigger distances
  const int CLAW_GRAB_TRIGGER_1_3	= 2500;
  const int CLAW_GRAB_TRIGGER_2  	= 2100;
  const int WALL_TRIGGER          = 3550;
  const int FRONT_CORNER_TRIGGER  = 3250;

  //wall following constants
  const int WALL_FOLLOW_CENTER  	= 3620;
  const int WALL_FOLLOW_CENTER_LEFT = 3750;
  const int WALL_FOLLOW_CLOSE_GAIN  = 230;  //too far right 
  const int WALL_FOLLOW_FAR_GAIN    = 200;  //too far left 

  //Timing constants
  const int ACCELERATE_STEP_SIZE = 	10;
  const int KICKER_MOVE_BACK_TIME = 250 * TIMING_CONST;
  const int COME_HOME_TIME 		= 2000 * TIMING_CONST;
  const int JIGGLE_TIME_PERIOD		= 1500;

  
  const int stateMap[] = {
    WAIT,                 //start sequence
    RIGHT_TURN,
    LINE_FOLLOW,
    LEFT_TURN,
    LINE_FOLLOW,	//4

    HANDLE_OBSTACLE,
    FIND_LINE,				
    LINE_FOLLOW_OFFSET2,
    LINE_FOLLOW_OFFSET,
    EJECT_BARREL,          //9 first barrel eject/pickup
    
    GRAB_BARREL,           
    LINE_FOLLOW_OFFSET,    // counting lines
    LINE_FOLLOW_OFFSET,   
    FIND_CORNER_BARREL,
    GRAB_CORNER_BARREL, //14
    
    ROUND_A_BOUT,         
    ROUND_A_BOUT, 
    ROUND_A_BOUT,
    LINE_FOLLOW,   
    LINE_FOLLOW,  //19 
   
    RIGHT_TURN,    
    LINE_FOLLOW_OFFSET,    // third barrel pickup/drop
    EJECT_BARREL,      
    GRAB_BARREL,    //23
    ROUND_A_BOUT2,  //24
    
    LINE_FOLLOW,
    RIGHT_TURN_SPIN,
    LINE_FOLLOW,
    LINE_FOLLOW, 
    FIND_LINE, // 29 
    
    WALL_FOLLOW_LEFT,
    FIND_LINE_RIGHT,  
    LINE_FOLLOW,   
    RIGHT_TURN,   
    LINE_FOLLOW_OFFSET, //34
    
    DUMP_BARRELS
  };
  
  

#endif
