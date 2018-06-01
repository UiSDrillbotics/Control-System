/*
Name:    Circulation.ino
Created: 6/7/2017 8:39:50 PM
Author:  TeamUiS
*/

int pressureSensor = A0;
const int pump1 = 8; //original pump
const int pump2 = 10;  //Pump 2
char incomingOption;
bool pump1Active = true;
bool turnOnPump = false;

#define Stop 0
#define Pressure 1
#define Monitor 2
#define END 3
#define coolDown 6

int mode = Stop;
int m = 0;

float pump_rampUp_time = 10000; // 10 seconds initial test
float pump_start_time;
float time_since_pump_start;

float maximum_pressure = 4.5;
float minimum_pressure = 2.0; // supposed to be 2.0

float dataSampleInterval; // [s]
float filterTimeConstant = 250.0 / 1000.0; // [s]
float filtered_old;
float result;
int reading;
int ms_init; // in ms
int ms_start_loop_old; // in ms

const int timeStep = 10; // in ms
unsigned int elapsed; // in ms
unsigned int previousTime; // in ms
unsigned int currentTime; // in ms
unsigned int endOfLoopTime; // in ms

unsigned int pumpTimer = 0;
unsigned int switchTime = 1000 * 10;

unsigned int pumpSwitchTimer;

float pressure;
void setup() {
	ms_init = millis();
	ms_start_loop_old = ms_init;
	pinMode(pump1, OUTPUT);
	pinMode(pump2, OUTPUT);
	Serial.begin(9600);
	digitalWrite(pump1, LOW);
	digitalWrite(pump2, LOW);
	mode = Stop;



	filtered_old = analogRead(pressureSensor);
}


void loop()
{
	currentTime = millis();
	int ms_start_loop = millis();
	dataSampleInterval = (ms_start_loop - ms_start_loop_old) / 1000.0; // [sec]
	ms_start_loop_old = ms_start_loop;
	//result = analogRead(pressureSensor);
	result = lowpassfilter(analogRead(pressureSensor));
	pressure = ConvertPressure(result);

	//if (mode != END)
	//{


	incomingOption = Serial.read();
	switch (incomingOption)
	{
	case '0':
		mode = Stop;
		m = 0;

		break;

	case '1':
		pump_start_time = millis();
		mode = Pressure;
		m = 1;
		turnOnPump = true;
		pump1Active = false; //i.e. simulate pump2 has been on, so we always start with pump 1
		digitalWrite(pump2, LOW);
		break;

	case '2':
		mode = Monitor;
		m = 2;

		break;

	case '3':
		mode = END;
		m = 5;

		break;
	}

	if (mode == coolDown)
	{
		if ((millis() - pumpTimer) > 15000)
		{
			digitalWrite(pump1, HIGH);
			mode = Pressure;
			pump_start_time = millis();

		}
	}

	if (mode == Pressure)
	{
		if (turnOnPump == true)
		{
			if (pump1Active == true) //i.e. pump 1 was just on, so we need to turn off pump 1 and start pump 2
			{
				digitalWrite(pump1, LOW);
				digitalWrite(pump2, HIGH);
				pump1Active = false;
				turnOnPump = false;
				pumpSwitchTimer = millis();
			}
			else if (pump1Active == false) { //turn off pump2, and turn on pump 1
				digitalWrite(pump1, HIGH);
				digitalWrite(pump2, LOW);
				pump1Active = true;
				turnOnPump = false;
				pumpSwitchTimer = millis();
			}
		}

		//digitalWrite(pump1, HIGH); //see what happens if I get rid of this and move it to just under the case 1 command
		time_since_pump_start = millis() - pump_start_time;
		if (time_since_pump_start  > pump_rampUp_time)
		{
			mode = Monitor;
			m = 2;
			pumpTimer = millis();
		}

	}

	if (mode == Monitor)
	{
		if ((millis() - pumpSwitchTimer) > switchTime)
		{
			turnOnPump = true;
			mode = Pressure;
			pump_start_time = millis();
		}
		/*This is the toggle on on off for one pump
		if ((millis() - pumpTimer) > (30000 - pump_rampUp_time))
		{
		digitalWrite(pump1, LOW);
		mode = coolDown;
		pumpTimer = millis();
		return;
		}*/
		if (pressure > maximum_pressure)
		{
			digitalWrite(pump1, LOW);
			digitalWrite(pump2, LOW);
			mode = Stop;
			m = 3;
		}
		if (pressure < minimum_pressure) //(pressure < minimum_pressure)
		{
			digitalWrite(pump1, LOW);
			digitalWrite(pump2, LOW);
			//mode = Stop;
			m = 4;
		}
		else
		{
			//digitalWrite(pump1, HIGH); //again check if this is needed or if I can just do it once
			m = 2;
		}
	}
	else if (mode == Stop)
	{
		digitalWrite(pump1, LOW);
		digitalWrite(pump2, LOW);
		m = 0;
	}

	//}

	Serial.print("x");
	Serial.print(m);  //0.0 = stop. 1.0 = pressurizing; 2.0 = normal monitor mode; 3.0 = max pressure exceeded; 4.0 = min pressure not met
	Serial.print("y");
	Serial.print(pressure);
	Serial.println("z");

	endOfLoopTime = millis();
	if (endOfLoopTime > currentTime)
	{
		if ((endOfLoopTime - currentTime) < timeStep)
		{
			delay(timeStep - endOfLoopTime + currentTime);
		}
	}
}

float lowpassfilter(int measurement)
{
	float alpha = dataSampleInterval / (filterTimeConstant + dataSampleInterval);
	float filteredValue = alpha * measurement + ((1 - alpha)* filtered_old);
	filtered_old = filteredValue;
	return filteredValue;
}
float ConvertPressure(float result)
{

	float current = (float)((3.3 / 220.0) * (result - 272.8) + 4.0);
	//3.3 is arduino due voltage. 220 is resistor value. 272.8 is calibration integer. 4.0 is pressure sensors lowest mA output
	float value = 1.034 + (current - 4.0) * (10.0 / 16.0);
	return value;
}
