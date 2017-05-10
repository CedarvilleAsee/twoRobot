
#include "Timer.h"
#include <Arduino.h>
/*
  method Timer constructor
  4/5/17 - NH
  This doesn't do to much useful, just set up the timer
*/
Timer::Timer(){
	endTime = -1;
	startTime = 0;
}
	
Timer::Timer(const Timer& old){
  endTime = old.endTime;
  startTime = old.startTime;
}
  
  
/*
  methods set(int) and set(int[], int)
  4/5/17 - NH
  
*/
void Timer::set(int time){
	endTime = time;
	startTime = millis();
}



/*
  method isTimeUp
  4/5/17 - NH
  intended for use with a single Timer (where size == 1).
  returns true if the time is up 
*/
bool Timer::isTimeUp(){
	if(!isTimerSet()){
		return false;
	}
	return millis() >= (endTime + startTime);
}


/*
  method isTimeUpUnset
  4/5/17 - NH
  if the time is up, this method puts the Timer into the isSet == false state
  and returns true.
  Thus this function will only return true once and only once
*/
bool Timer::isTimeUpUnset(){
 
  if(isTimerSet() && (millis() >= (endTime + startTime))){
  	unset();
    return true;
  }
  return false;
}
  
/*
 method isTimerSet
	

*/
bool Timer::isTimerSet(){
	return endTime != -1;
}

/*
 method unset
	

*/
void Timer::unset(){
	endTime = -1;
}

/*
 method timeElapsed
	

*/
int Timer::timeElapsed(){
	if(!isTimerSet()){
		return -1;
	}
	return (millis() - startTime);
}




