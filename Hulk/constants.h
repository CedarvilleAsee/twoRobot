#ifndef CONSTANTS_H
#define CONSTANTS_H

#ifndef R2_LEFT
//#define R2_LEFT 3
	//const int FULL_SPEED = 250;
#endif


#ifndef D2_RIGHT
#define D2_RIGHT 1
	const int FULL_SPEED = 100;
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
		COME_HOME					 = 16
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

	#ifdef R2_LEFT
		const int FULL_SPEED = 250;
		const int EJECT_FRONT_POSITION = 87;
		const int EJECT_BACK_POSITION = 58;
		const int CLAW_OPEN = 80;
		const int CLAW_CLOSED = 42;  
		const int ARM_DOWN = 35;  
		const int ARM_UP   = 150;  
		const int DUMP_UP = 20;
		const int DUMP_DOWN = 130;

		//sensor trigger distances
		const int CLAW_GRAB_TRIGGER_1_3	= 2500;
		const int CLAW_GRAB_TRIGGER_2  	= 2100;
		const int WALL_TRIGGER          = 3550;
		const int FRONT_CORNER_TRIGGER  = 2500;
		//wall following constants

		const int WALL_FOLLOW_CENTER  	= 3650;
		const int WALL_FOLLOW_CENTER_LEFT= 3740;
		const int WALL_FOLLOW_CLOSE_GAIN= 180;  //too far right 
		const int WALL_FOLLOW_FAR_GAIN  = 150;  //too far left 


		//spain wall following constants
		const int WALL_F_FRONT_FAR_GAIN   = 370;
		const int WALL_F_FRONT_CLOSE_GAIN = 220;
		const int WALL_F_FRONT_CENTER     = 2700;
		
		
		//Timing constants
		const int ACCELERATE_STEP_SIZE = 	10;
		const int KICKER_MOVE_BACK_TIME = 250;
		const int COME_HOME_TIME 				= 2000;
		const int JIGGLE_TIME_PERIOD		= 200;

		const int stateMap[] = {
			WAIT,                 //start sequence
			DEPART_SPAIN,
			HANDLE_OBSTACLE,      //obstacle sequence
			FIND_LINE,
			LINE_FOLLOW_OFFSET2,   //4
			
			LINE_FOLLOW_OFFSET,
			EJECT_BARREL,          //first barrel eject/pickup
			GRAB_BARREL,           
			LINE_FOLLOW_OFFSET,    //counting lines
			LINE_FOLLOW_OFFSET,    //9
			
			FIND_CORNER_BARREL,   
			GRAB_CORNER_BARREL,
			ROUND_A_BOUT,         
			ROUND_A_BOUT,
			LINE_FOLLOW,  //14
			
			LEFT_TURN,    
			LINE_FOLLOW_OFFSET,    //third barrel pickup/drop
			EJECT_BARREL,          
			GRAB_BARREL,           
			RIGHT_TURN,     //19
			
			LINE_FOLLOW,     
			LINE_FOLLOW,
			LINE_FOLLOW,
			HANDLE_OBSTACLE,  
			WALL_FOLLOW_FAR,   //24
			
			WALL_FOLLOW_FAR,
			FIND_LINE,            
			LINE_FOLLOW,   
			LINE_FOLLOW_OFFSET,   
			COME_HOME,      //29
			DUMP_BARRELS        
		};

	
	#else 
	 
		/******************************************\
		| Right Robot Constants                    |
		| D2 constants                             |
		\******************************************/
		const int EJECT_FRONT_POSITION  = 115;
		const int EJECT_BACK_POSITION   = 95;
		const int CLAW_OPEN 			= 55;
		const int CLAW_CLOSED 			= 85;  
		const int ARM_DOWN 				= 10;  
		const int ARM_UP 				= 140;  
		const int DUMP_UP 				= 100;
		const int DUMP_DOWN 			= 10;

		//spain wall following constants
		const int WALL_F_FRONT_FAR_GAIN   = 370;
		const int WALL_F_FRONT_CLOSE_GAIN = 220;
		const int WALL_F_FRONT_CENTER     = 2700;

		//sensor trigger distances
		const int CLAW_GRAB_TRIGGER_1_3	= 2500;
		const int CLAW_GRAB_TRIGGER_2  	= 2100;
		const int WALL_TRIGGER          = 3550;
		const int FRONT_CORNER_TRIGGER  = 2500;

		//wall following constants
		const int WALL_FOLLOW_CENTER  	= 3650;
		const int WALL_FOLLOW_CENTER_LEFT= 3740;
		const int WALL_FOLLOW_CLOSE_GAIN= 180;  //too far right 
		const int WALL_FOLLOW_FAR_GAIN  = 150;  //too far left 

		//Timing constants
		const int ACCELERATE_STEP_SIZE = 	10;
		const int KICKER_MOVE_BACK_TIME = 250;
		const int COME_HOME_TIME 				= 2000;
		const int JIGGLE_TIME_PERIOD		= 200;

		
		const int stateMap[] = {
			WAIT,                 //start sequence
			DEPART_SPAIN,
			HANDLE_OBSTACLE,      //obstacle sequence
			FIND_LINE,
			LINE_FOLLOW_OFFSET2,   //4
			
			LINE_FOLLOW_OFFSET,
			EJECT_BARREL,          //first barrel eject/pickup
			GRAB_BARREL,           
			LINE_FOLLOW_OFFSET,    //counting lines
			LINE_FOLLOW_OFFSET,    //9
			
			FIND_CORNER_BARREL,   
			GRAB_CORNER_BARREL,
			ROUND_A_BOUT,         
			ROUND_A_BOUT,
			LINE_FOLLOW,  //14
			
			LEFT_TURN,    
			LINE_FOLLOW_OFFSET,    //third barrel pickup/drop
			EJECT_BARREL,          
			GRAB_BARREL,           
			RIGHT_TURN,     //19
			
			LINE_FOLLOW,     
			LINE_FOLLOW,
			LINE_FOLLOW,
			HANDLE_OBSTACLE,  
			WALL_FOLLOW_FAR,   //24
			
			WALL_FOLLOW_FAR,
			FIND_LINE,            
			LINE_FOLLOW,   
			LINE_FOLLOW_OFFSET,   
			COME_HOME,      //29
			DUMP_BARRELS        
		};
	#endif
  
  

#endif
