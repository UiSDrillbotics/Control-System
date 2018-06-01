// HookLoad.h

#ifndef _HOOKLOAD_h
#define _HOOKLOAD_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "definitions.h"
#include "LoadCell.h"


class HookLoad
{
 protected:
	 LoadCellClass lc[NUM_LOADCELLS] = {
		 LoadCellClass(loadcellX1, loadcellY1, loadcellZ1),
		 LoadCellClass(loadcellX2, loadcellY2, loadcellZ2),
		 LoadCellClass(loadcellX3, loadcellY3, loadcellZ3) };
	 int _raw[SPACIAL_DIMENSIONS] = { 0,0,0 };
	 int _filtered[SPACIAL_DIMENSIONS] = { 0,0,0 };

public:
	void setup();

	void detect();

	int getRawX() {
		return _raw[0];
	}

	int getRawY() {
		return _raw[1];
	}

	int getRawZ() {
		return _raw[2];
	}

	int getX() {
		return _filtered[0];
	}

	int getY() {
		return _filtered[1];
	}

	int getZ() {
		return _filtered[2];
	}
};

#endif

