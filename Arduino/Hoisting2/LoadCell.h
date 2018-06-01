// LoadCell.h

#ifndef _LOADCELL_h
#define _LOADCELL_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "definitions.h"

class LoadCellClass
{
 protected:
	 const int _XPin, _YPin, _ZPin;
	 int _raw[SPACIAL_DIMENSIONS];
	 int _filtered[SPACIAL_DIMENSIONS];
	 int _age;
	 bool _isSetup;

 public:
	 LoadCellClass(int XPin, int YPin, int ZPin);

	 void detect();

	 int getRaw(uint i) { return _raw[i]; }

	 int getRawX() { return _raw[0]; }

	 int getRawY() { return _raw[1]; }

	 int getRawZ() { return _raw[2]; }

	 int getX() { return _filtered[0]; }

	 int getY() { return _filtered[1]; }

	 int getZ() { return _filtered[2]; }
	void setup();
};

extern LoadCellClass LoadCell;

#endif

