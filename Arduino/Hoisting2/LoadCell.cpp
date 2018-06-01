// 
// 
// 

#include "LoadCell.h"

LoadCellClass::LoadCellClass(int XPin, int YPin, int ZPin) :
	_XPin(XPin),
	_YPin(YPin),
	_ZPin(ZPin)
{
	_isSetup = false;
}

void LoadCellClass::detect()
{
	_raw[0] = analogRead(_XPin);
	_raw[1] = analogRead(_YPin);
	_raw[2] = analogRead(_ZPin);
	_age = 0;
}

void LoadCellClass::setup()
{
	_isSetup = true;
	this->detect();
}


LoadCellClass LoadCell;

