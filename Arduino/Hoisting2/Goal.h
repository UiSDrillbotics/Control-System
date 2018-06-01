// Goal.h

#ifndef _GOAL_h
#define _GOAL_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "definitions.h"

class Goal
{
 protected:


 public:
	 virtual bool isFinished(beliefs b) {};  //the goal has been reached
	 virtual actions_t plan(beliefs b) {};   //the action required to reach the goal is planned
	 virtual bool canContinue(beliefs b) {}; //there is a plan
	 virtual bool isValid() {}; //the goal is valid (the goal state doesn't violate the constraints)
	 virtual String describe() {};
};

class StopGoal : public Goal {
public:
	bool isFinished(beliefs b);
	actions_t plan(beliefs b);
	bool canContinue(beliefs b);
	bool isValid();
	String describe();
};

class MoveByGoal : public Goal {
private:
		constraints_t _constraints;
		int _finalPosition;
public:
	MoveByGoal(int distance, HoistingDirectionCommand direction, beliefs b, constraints_t constraints);
	bool isFinished(beliefs b);
	actions_t plan(beliefs b);
	bool canContinue(beliefs b);
	bool isValid();
	String describe();
};

class WOBGoal : public Goal {
private:
	constraints_t _constraints;
	int _finalPosition;
	double _kp, _ki, _kd, _ITerm;
	double _setpoint;

public:
	WOBGoal(int finalPosition, int kp, int ki, int kd, double setpoint, constraints_t constraints);
	bool isFinished(beliefs b);
	actions_t plan(beliefs b);
	bool canContinue(beliefs b);
	bool isValid();
	String describe();
};

class BrakesGoal : public Goal {
private:
	BrakeStatus _goalState;

public:
	BrakesGoal(BrakeStatus goalState);
	bool isFinished(beliefs b);
	actions_t plan(beliefs b);
	bool canContinue(beliefs b);
	bool isValid();
	String describe();
};

class TagBottomGoal : public TagBottomGoal {
private:
	bool finished = false;
public:
	bool isFinished(beliefs b);
	actions_t plan(beliefs b);
	bool canContinue(beliefs b);
	bool isValid();
	String describe();
};
#endif

