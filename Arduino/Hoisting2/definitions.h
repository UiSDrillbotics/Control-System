// definitions.h

#ifndef _DEFINITIONS_h
#define _DEFINITIONS_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#define SPACIAL_DIMENSIONS 3
#define NUM_GOALS 10
#define NUM_BELIEFS 255
#define NUM_ACTUATORS 3
#define NUM_LOADCELLS 3

#pragma region Pin Constants
const int loadcellX1 = A4;
const int loadcellY1 = A11; // was A5 Needed to be changed due to Z1
const int loadcellZ1 = A5; // was A11
const int loadcellX2 = A7;
const int loadcellY2 = A8;
const int loadcellZ2 = A0;
const int loadcellX3 = A1;
const int loadcellY3 = A2;
const int loadcellZ3 = A3; // was A5

const int Brake1 = 12; // was 3
const int Brake2 = 10; // was 5
const int Brake3 = 13; // was 9 

const int direction_wire1 = 3;  // was 45 previously
const int direction_wire2 = 6;  // was 49
const int direction_wire3 = 9;  // was 53

const int step_wire1 = 2;   // was 43
const int step_wire2 = 4;   // was 47
const int step_wire3 = 7;   // was 51

const int pushButton1 = 11;
const int pushButton2 = 12;
const int pushButton3 = 13;
#pragma endregion

enum CommandType {
	MOVE,
	BRAKE
};

enum BrakeStatus {
	OPENED = 2,
	CLOSED = 1
};

enum BrakeCommand {
	OPEN = 2,
	CLOSE = 1
};

enum HoistingDirectionCommand {
	UP = 1,
	DOWN = 2
};

struct measurements {
	int hookload;
	int position; // from laser sensor
	int timestamp;
};

struct beliefs {
	int hookload;
	int position;
	HoistingDirectionCommand hoistingDirection;
	int speed;
	int freeWeight;
	int steps;
	int timestamp;
	BrakeStatus brake;
	Goal* currentGoal;
	int age;
};

typedef struct {
	int maxHookload;
	int maxWOB;
	int maxSpeed;
	int maxPosition;
	int minPosition;
} constraints_t;

typedef struct {
	int moveDirection;
	int moveSpeed;
	BrakeCommand brakeCommand;
} actions_t;

const char END_OF_TRANSMISSION = '\n';

enum commandType {
	STOP = 1,
	CALIBRATE = 2,
	MOVE = 3,
	BRAKES = 4,
	NOTHING = 5,
	TOGGLE_TELEMTRY = 6,
	TOGGLE_WOBCONTROL = 7,
	SET_PID_TUNINGS = 8,
	INTERROGATE = 9,
	ZERO_WOB = 10,
	RESET_STEPPERS = 11,
	TOGGLE_HAMMER = 12,
	TOGGLE_AUTOMATION = 13
};

enum errorTypes {
	BAD_COMMAND_STRING = 0,
	BAD_COMMAND = 1
};
#endif

