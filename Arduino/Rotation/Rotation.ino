/*
 Name:		Rotation.ino
 Created:	6/7/2017 8:50:42 PM
 Author:	TeamUiS
*/

#include "ArduinoSort.h"

const uint32_t samplingTimeMs = 10;
const int FILTERWINDOW = 25;
const int TORQUESET = 8;
const int MOTOR = 7;
const int TORQUESENSOR = A0;
const int RPMMONITOR = A1;
const int TORQUEMONITOR = A2;

void setup() {
	Serial.begin(9600);
	while (!Serial);

	pinMode(MOTOR, OUTPUT);

	analogWriteResolution(12);
	analogReadResolution(12);
}

void loop() {
	static boolean flagPrint = false;
	static boolean flagConversion = false;

	static int torqueInputScale = 1;                            //(percentage/100)/v
	static float motorRatedTorque = 2.86;                       //N*m
	static int motorScale = 275;                                //rpm/v
	static float arduinoVoltage = 3.188;

	int mode = 0;
	int duration;
	uint32_t preTimeStamp, curTimeStamp;
	int rpmInputInteger, torqueInputInteger, torqueSensorInteger;
	int rpmInputIntegerArray[FILTERWINDOW], torqueInputIntegerArray[FILTERWINDOW], torqueSensorIntegerArray[FILTERWINDOW];
	float rpmInputVoltage, torqueInputVoltage, torqueSensorVoltage;
	double actualRPM, torqueMotor, torqueSensor;

	while (true) {
		if (flagPrint == true) {
			Serial.write("x");
			Serial.print(mode);  //0 = stop 1 = rotating;
			Serial.write("y");
			Serial.print(actualRPM);
			Serial.write("y");
			Serial.print(torqueMotor);
			Serial.write("y");
			Serial.print(torqueSensor);
			Serial.write("y");
			Serial.println("z");
			Serial.println(duration / 1000);
			flagPrint = false;                                      // Serial.print done
		}

		if (flagConversion == true) {
			rpmInputInteger = medianFilter(rpmInputIntegerArray, rpmInputInteger);
			// rpmInputVoltage = (3.3 / 4096) * rpmInputInteger;
			// actualRPM = 269.65 * rpmInputVoltage - 21.099;
			actualRPM = (0.3704*rpmInputInteger - 59.674);
				
			if (abs(actualRPM) < 25) {
				mode = 0;
			}
			else {
				mode = 1;
			}

			torqueInputInteger = medianFilter(torqueInputIntegerArray, torqueInputInteger);
			// torqueInputVoltage = (3.3 / 4096) * torqueInputInteger;
			// torqueMotor = torqueInputVoltage * torqueInputScale * motorRatedTorque;
			torqueMotor = (0.0021*torqueInputInteger - 0.3121);

			torqueSensorInteger = medianFilter(torqueSensorIntegerArray, torqueSensorInteger);
			torqueSensorVoltage = (3.3 / 4096) * torqueSensorInteger;
			torqueSensor = (torqueSensorVoltage - 2.4882) / 0.0005;

			flagConversion = false;                                 // Conversion done
			flagPrint = true;                                       // A Serial.print is requested
		}
		float rpmInteger;
		if (Serial.available())
		{
			float rpmInput = Serial.parseFloat();

			if (rpmInput > 1400.0) {
				rpmInput = 1400.0;
			}
			if (rpmInput < 30) {
				rpmInput = 0.0;
			}
						
			rpmInteger = (2.7254*rpmInput) - 0.0817;

			// rpmInteger = (-4e-10*pow(rpmInteger, 4)) + (3e-06*pow(rpmInput, 3)) - (0.004*pow(rpmInput, 2)) + (4.3524*rpmInput) + 212.82; // 4th degree Poly, R^2 = 0.9985

			// float rpmVoltage = rpmInput / motorScale;               //required voltage to send drive
			// int rpmInteger = (4096 / arduinoVoltage) * rpmVoltage;  //integer for PWM
	
			
			// if (rpmInteger > 4095)
			// {
			//	rpmInteger = 4095;
			// }
			analogWrite(MOTOR, rpmInteger);
		}

		curTimeStamp = micros();
		if (curTimeStamp - preTimeStamp > samplingTimeMs * 1000 - 1) {
			duration = curTimeStamp - preTimeStamp;
			preTimeStamp = curTimeStamp;

			rpmInputInteger = analogRead(RPMMONITOR);
			torqueInputInteger = analogRead(TORQUEMONITOR);
			torqueSensorInteger = analogRead(TORQUESENSOR);

			flagConversion = true;                                  // A conversion is requested
		}
	}
}

int medianFilter(int dataArray[], int newData) {
	for (int i = 0; i < FILTERWINDOW - 1; i++) {
		dataArray[i] = dataArray[i + 1];
	}
	dataArray[FILTERWINDOW - 1] = newData;

	int dataArrayClone[FILTERWINDOW];
	for (int i = 0; i < FILTERWINDOW; i++) {
		dataArrayClone[i] = dataArray[i];
	}

	sortArray(dataArrayClone, FILTERWINDOW);
	int returnIndex = (FILTERWINDOW - 1) * 0.5;

	return dataArrayClone[returnIndex];
}