/*
 Name:		Hoisting2.ino
 Created:	5/30/2018 12:49:03 PM
 Author:	roew
*/

// the setup function runs once when you press reset or power the board
#include "Parser.h"
#include "Goal.h"
#include "HookLoad.h"
#include "definitions.h"
#include "Hookload.h"

HookLoad _hookLoad = HookLoad();
//uint beliefsIdx = 0;
//beliefs _beliefs[NUM_BELIEFS];

beliefs _beliefs;


// different goals: TareWOB, Move Up, Move Down, DrillROP, DrillWOB 
// different constraints: maxWOB, maxROP, maxSpeed, maxPosition, minPosition
uint goalIdxHead = 0;
uint goalIdxTail = 0;
Goal* goals[NUM_GOALS];

int replaceCurrentGoal(Goal* g) {
	//if (goalIdxHead != goalIdxTail) //the buffer is not empty
	//{
	//	Goal* old = goals[goalIdxHead];
	//	delete old;
	//	goals[goalIdxHead] = g;
	//}
	//else {
	//	return goalsPush(g);
	//}
	beliefs b = _beliefs;
	Goal* old = b.currentGoal;
	delete old;
	b.currentGoal = g;
	return 0;
}

int goalsPush(Goal* g) {
	int next = goalIdxHead + 1;
	if (next >= NUM_GOALS) {
		next = 0;
	}

	if (next == goalIdxTail) {
		return -1; //error, buffer full
	}

	goals[goalIdxHead] = g;
	goalIdxHead = next;

	return 0; //success
}

Goal* goalsPop() {
	if (goalIdxHead == goalIdxTail) {
		return new StopGoal(); // empty buffer, we stop;
	}
	Goal* g = goals[goalIdxTail];
	uint next = goalIdxTail + 1;
	if (next >= NUM_GOALS) {
		next = 0;
	}
	goalIdxTail = next;
	return g;
}



void setup() {
	_hookLoad.setup();
}

// detect whats going on in the world, the clock is a sensor too....
measurements percieve() {
	_hookLoad.detect();	
	measurements m;
	m.hookload = _hookLoad.getRawZ();
	m.position = 0;	
	return m;
}

beliefs updateBeliefs(measurements m) {
	beliefs b = _beliefs;
	b.hookload = m.hookload;
	b.timestamp = m.timestamp;	
	//if (lastAction.moveDirection) {
	//	b.position--;
	//}
	//else {
	//	b.position++;
	//}

}

// plan what should be done to reach the goals
actions_t plan(beliefs b) {	
	while (b.currentGoal->isFinished(b)) {
		delete b.currentGoal;
		b.currentGoal = goalsPop();
	}

	actions_t a = b.currentGoal->plan(b);
	return a;
}

// execute the plan
void act(actions_t a) {
	beliefs b = _beliefs;
	for (uint actIdx = 0; actIdx < NUM_ACTUATORS; actIdx++)
	{
		//uint pos = act[actIdx].getPosition();
	}
}

Parser parser;

// provide status and receive goals + constraints
void communicate() {
	int bytes = Serial.available();
	if (bytes > 0) {
		parser.parseData(Serial.readStringUntil(END_OF_TRANSMISSION));
	}
}


// the loop function runs over and over again until power down or reset
void loop() {	
	act(plan(updateBeliefs(percieve())));	
	communicate();
}
