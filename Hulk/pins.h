/*
 * PINS.H
 *
 * Defines all of the pins for interaction with the Arduino
 * board.
 *
 * Author: Daschel Fortner, Nathan Herr
 * Date:   1/21/17
 * Modifications:
 *		- Added documentation - DTF 1/27/17
 *   - modified pin mapping for new shield 2/11/17 NSH
 *		- modified file to have inclusion guards so this file can be 
 * 			used for both robot's (R2 and D2) pinouts
 *		Do NOT USE PA12 or PA15 they are pulled to 3.3 volts by default
 *
 */
 
#include <Arduino.h>
#include "constants.h"
#ifndef PINS_H
#define PINS_H


/**
*   BUTTONS
*/
#define STOP_BUTTON   PB4
#define GO_BUTTON     PB5

//7 segment pins
#define  CS           PC13
#define CLOCK         PC14
#define DIN           PC15


/******************************************\
| Left Robot Pinouts                       |
| R2 pinouts                               |
\******************************************/
#ifdef R2_LEFT
		/**
		 * MOTOR CONTROLLER PINS
		 *    ______________________________________
		 *   |                                      |
		 *   | CCW -> MC_AIN2 = HIGH, MC_AIN1 = LOW |
		 *   | CW  -> MC_AIN1 = LOW, MC_AIN2 = HIGH |
		 *   | Short brake -> MC_AIN1 = MC_AIN2 = 1 |
		 *   | Stop -> MC_AIN1 = MC_AIN2 = 0        |
		 *    --------------------------------------
		 */
		 
		/* RIGHT MOTOR */
		#define MC_PWMA       PB0
		#define MC_AIN2       PB12
		#define MC_AIN1       PB13

		/* LEFT MOTOR */
		#define MC_BIN1       PB14
		#define MC_BIN2       PB15
		#define MC_PWMB       PB1

		/**
		 * SERVO PINS
		 */
		#define DUMP_SERVO    PA9
		#define EJECT_SERVO   PB6
		#define ARM_SERVO     PA8
		#define CLAW_SERVO    PA10


		/**
		 * SENSORS
		 */
		 #define SENSOR_0     PB11
		 #define SENSOR_1     PB10
		 #define SENSOR_2     PA11
		 #define SENSOR_3     PA5
		 #define SENSOR_4     PA15
		 #define SENSOR_5     PA4
		 #define SENSOR_6     PA7
		 #define SENSOR_7     PA6


		#define WALL_SENSOR1  PA0
		#define CLAW_SENSOR   PA2
		#define FRONT_SENSOR  PA1
		#define START_LIGHT   PB8

		#define S_LED_ON     PA0
	
	
	/******************************************\
	| Right Robot pinouts                      |
	| D2 pinouts 	                             |
	\******************************************/
	#else
	
		/**
		 * MOTOR CONTROLLER PINS
		 *    ______________________________________
		 *   |                                      |
		 *   | CCW -> MC_AIN2 = HIGH, MC_AIN1 = LOW |
		 *   | CW  -> MC_AIN1 = LOW, MC_AIN2 = HIGH |
		 *   | Short brake -> MC_AIN1 = MC_AIN2 = 1 |
		 *   | Stop -> MC_AIN1 = MC_AIN2 = 0        |
		 *    --------------------------------------
		 */
		 
		/* RIGHT MOTOR */
		#define MC_PWMA       PB0
		#define MC_AIN2       PB12
		#define MC_AIN1       PB13

		/* LEFT MOTOR */
		#define MC_BIN1       PB14
		#define MC_BIN2       PB15
		#define MC_PWMB       PB1     

		/**
		 * SERVO PINS
		 */
		#define DUMP_SERVO    PA8
		#define EJECT_SERVO   PB6
		#define ARM_SERVO     PA9
		#define CLAW_SERVO    PA10 
    

		/**
		 * SENSORS
		 */
		 #define SENSOR_0     PA6
		 #define SENSOR_1     PA7
		 #define SENSOR_2     PB9
		 #define SENSOR_3     PA4
		 #define SENSOR_4     PA5
		 #define SENSOR_5     PA11
		 #define SENSOR_6     PB10
		 #define SENSOR_7     PB11         

		#define WALL_SENSOR1  PA0
		#define CLAW_SENSOR   PA1
		#define FRONT_SENSOR  PA2

		#define S_LED_ON     
	
	#endif //D2_RIGHT
	
#endif

