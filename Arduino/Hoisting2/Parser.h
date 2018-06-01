// Parser.h

#ifndef _PARSER_h
#define _PARSER_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "definitions.h"

class Parser {
private:
	errorTypes Error;
	bool telemetry_on = true;
	double kp;
	double ki;
	double kd;
	bool WOBControlenabled = false;
	int WOBSetpoint_int;
public:	
	int parseData(String read);
};

#endif

