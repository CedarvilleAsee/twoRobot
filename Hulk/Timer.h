
#ifndef TIMER_H
#define TIMER_H

/*
	clase Timer
	4/5/17 - NH
	this class provides an abstraction for 
	timing. 
*/

class Timer{
	public:
		
		Timer(); //worthless constuctor
    Timer(const Timer&);
		void set(int time); //basic start button
		bool isTimeUp(); //check your watch and holler
		bool isTimeUpUnset();
		bool isTimerSet();
		
		void unset();
		int timeElapsed();

	private:
		int endTime;
		int startTime;
};

#endif

