// 
// 
// 

#include "Goal.h"

bool StopGoal::isFinished(beliefs b)
{
	return ((b.speed == 0) && (b.brake == BrakeStatus::CLOSED));
}

actions_t StopGoal::plan(beliefs b)
{
	actions_t a = actions_t();	
	a.moveSpeed = 0;
	a.moveDirection = b.hoistingDirection;
	a.brakeCommand = BrakeCommand::CLOSE;
	return a;
}

bool StopGoal::isValid()
{
	return true;
}

String StopGoal::describe()
{
	return String("StopGoal");
}

bool StopGoal::canContinue(beliefs b)
{
	return true; //stop can always continue;
}

MoveByGoal::MoveByGoal(int distance, HoistingDirectionCommand direction, beliefs b, constraints_t constraints)
{
	switch (direction) {
	case UP:
		_finalPosition = distance - b.position;
		break;
	case DOWN:
		_finalPosition = distance + b.position;
	}
	
	_constraints = constraints;
}

bool MoveByGoal::isFinished(beliefs b)
{
	return (b.position == _finalPosition);
}

actions_t MoveByGoal::plan(beliefs b)
{	
	actions_t a = actions_t();
	if (_finalPosition > b.position) {
		a.moveDirection = HoistingDirectionCommand::UP;
	}
	else if (_finalPosition < b.position) {
		a.moveDirection = HoistingDirectionCommand::DOWN;
	}
	a.moveSpeed = _constraints.maxSpeed;
	a.brakeCommand = BrakeCommand::CLOSE;
}

bool MoveByGoal::canContinue(beliefs b)
{
	if ((b.position > _constraints.maxPosition) && (_finalPosition > b.position)) {
		return false;
	}

	if ((b.position < _constraints.minPosition) && (_finalPosition < b.position)) {
		return false;
	}

	if ((b.hookload > _constraints.maxHookload) && (_finalPosition > b.position))  {
		return false;
	}

	return true;
}

bool MoveByGoal::isValid()
{
	if (_finalPosition > _constraints.maxPosition) {
		return false;
	}
	if (_finalPosition < _constraints.minPosition) {
		return false;
	}

	return true;
}

String MoveByGoal::describe()
{
	String s = "MoveByGoal : finalPosition";
	s.concat(_finalPosition);
	return s;
}

WOBGoal::WOBGoal(int finalPosition, int kp, int ki, int kd, double setpoint, constraints_t constraints)
{
	_finalPosition = finalPosition;
	_kp = kp;
	_ki = ki;
	_kd = kd;
	_constraints = constraints;
	_ITerm = 0;
}

bool WOBGoal::isFinished(beliefs b)
{
	return (abs(b.position - _finalPosition) < 5);
}

actions_t WOBGoal::plan(beliefs b)
{
	double WOB = (b.hookload - b.freeWeight);
	double error = _setpoint - WOB;
	_ITerm += (_ki * error);
	if (_ITerm > outMax) { _ITerm = outMax; }
	else if (_ITerm < outMin) { _ITerm = outMin; }
	double dInput = (WOB - lastWOB);

	double speed = _kp * error + _ITerm - _kd * dInput;

	actions_t a = actions_t();
	a.moveSpeed = speed;
	return actions_t();
}

bool WOBGoal::canContinue(beliefs b)
{
	return true;
}

bool WOBGoal::isValid()
{
	return (_setpoint < _constraints.maxWOB) && (_setpoint > 0) && (_ki >= 0) && (_kp >= 0) && (_kd >= 0);
	
}

String WOBGoal::describe()
{
	String s = "WOBGoal : finalPosition";
	s.concat(_finalPosition);
	return s;
}

BrakesGoal::BrakesGoal(BrakeStatus goalState)
{
	_goalState = goalState;
}

bool BrakesGoal::isFinished(beliefs b)
{
	return (b.brake == _goalState);
}

actions_t BrakesGoal::plan(beliefs b)
{
	actions_t a = actions_t();
	if (_goalState == BrakeStatus::OPENED) {
		a.brakeCommand = BrakeCommand::OPEN;
	}
	if (_goalState == BrakeStatus::CLOSED) {
		a.brakeCommand = BrakeCommand::CLOSE;
	}

	return a;
}

bool BrakesGoal::canContinue(beliefs b)
{
	return true;
}

bool BrakesGoal::isValid()
{
	return true;
}

String BrakesGoal::describe()
{
	String s = "BrakeGoal";
	s.concat(_goalState);
	return s;
}

bool TagBottomGoal::isFinished(beliefs b)
{
	return finished;
}

actions_t TagBottomGoal::plan(beliefs b)
{
	actions_t a = actions_t();

	return a;
}
