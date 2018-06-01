// 
// 
// 

#include "Parser.h"

int Parser::parseData(String read) {
	{
		String read = Serial.readStringUntil(END_OF_TRANSMISSION);
		int delim1Pos = read.indexOf(';');
		String CommandString = (delim1Pos == -1) ? read : read.substring(0, delim1Pos);
		int commandCode = CommandString.toInt();
		if (commandCode == 0) {
			return -1; // panic;6
		}

		commandType currentCommand = (commandType)commandCode;

		int delim2Pos = -1, delim3Pos = -1, delim4Pos = -1, delim5Pos = -1;
		switch (currentCommand) {
		case commandType::STOP:
		{
			replaceCurrentGoal(new StopGoal());
		} break;			
		case commandType::MOVE:
		{
			delim2Pos = read.indexOf(';', delim1Pos + 1);
			if (delim2Pos == -1) {
				return -1;  // panic
			}
			float tmpdistance = read.substring(delim1Pos + 1, delim2Pos).toFloat();
			HoistingDirectionCommand tmpdirection;
			delim3Pos = read.indexOf(';', delim2Pos + 1);
			if (delim3Pos == -1) {
				return -1;  // panic
			}
			switch (read.substring(delim2Pos + 1, delim3Pos).toInt())
			{
			case 1: tmpdirection = HoistingDirectionCommand::UP;
				break;
			case 2: tmpdirection = HoistingDirectionCommand::DOWN;
				break;
			default: Error = errorTypes::BAD_COMMAND_STRING;
				return -1; //panic
			}

			delim4Pos = read.indexOf(';', delim3Pos + 1);
			if (delim4Pos == -1) {
				return -1;  // panic
			}
			float tmpspeed = read.substring(delim3Pos + 1, delim4Pos).toFloat();

			delim5Pos = read.indexOf(';', delim4Pos + 1);
			if (delim5Pos == -1) {
				return -1;  // panic
			}
			float tmpactuator = read.substring(delim4Pos + 1, delim5Pos).toInt();

			constraints_t c = defaultConstraints();
			c.maxSpeed = tmpspeed;
			replaceCurrentGoal(new MoveByGoal(tmpdistance, tmpdirection, _beliefs, c));

			if (!telemetry_on) {
				Serial.print("speed:");
				Serial.print(tmpspeed);
			}
		} break;			
		case commandType::CALIBRATE:
			break;
		case commandType::BRAKES:
		{
			delim2Pos = read.indexOf(';', delim1Pos + 1);
			if (delim2Pos == -1) {
				Serial.println("panic in brake arg");
				return -1;
			} // panic
			BrakeCommand tmpbrake;
			switch (read.substring(delim1Pos + 1, delim2Pos).toInt())
			{
			case 1: tmpbrake = BrakeCommand::CLOSE;
				break;
			case 2: tmpbrake = BrakeCommand::OPEN;
				break;
			default: Error = errorTypes::BAD_COMMAND;
				return -1; //panic!
			}
		} break;
		case commandType::TOGGLE_TELEMTRY:
		{
			telemetry_on = !telemetry_on;
		} break;
		case commandType::TOGGLE_WOBCONTROL:
		{
			Serial.println("WOBControl toggle");
			delim2Pos = read.indexOf(';', delim1Pos + 1);
			if (delim2Pos == -1) {
				WOBControlenabled = !WOBControlenabled;
				return;
			} // panic
			switch (read.substring(delim1Pos + 1, delim2Pos).toInt())
			{
			case 0:
				WOBControlenabled = false;
				break;
			case 1:
				WOBControlenabled = true;
				break;
			default: Error = errorTypes::BAD_COMMAND;
				return; //panic!
			}
			if (WOBControlenabled) {
				replaceCurrentGoal(new WOBGoal( 100 /*final posistion*/, kp, ki, kd, WOBSetpoint_int, defaultConstraints()));
				//WOBControl.SetMode(AUTOMATIC);
				//WOBspeed = 300.0;
				//WOBFREQ_Hz = (1 / WOBspeed) * 1000000.0;
				//startTimer(TC1, 0, TC3_IRQn, WOBFREQ_Hz);
				//Mode = Modes::WOBCONTROL;
				Serial.println(Mode);
			}
			else {
				replaceCurrentGoal(new StopGoal());
				//WOBControl.SetMode(MANUAL);
				Mode = Modes::STOPPED;
			}
			Serial.println(WOBControlenabled);
		} break;
		case commandType::SET_PID_TUNINGS:
		{
			Serial.println("Setting PID constants");


			delim2Pos = read.indexOf(';', delim1Pos + 1);
			if (delim2Pos == -1) {
				return;  // panic
			}
			WOBSetpoint_int = read.substring(delim1Pos + 1, delim2Pos).toInt();

			delim3Pos = read.indexOf(';', delim2Pos + 1);
			if (delim3Pos == -1) {
				return;  // panic
			}
			kp = read.substring(delim2Pos + 1, delim3Pos).toDouble();

			delim4Pos = read.indexOf(';', delim3Pos + 1);
			if (delim4Pos == -1) {
				return;  // panic
			}
			ki = read.substring(delim3Pos + 1, delim4Pos).toDouble();

			delim5Pos = read.indexOf(';', delim4Pos + 1);
			if (delim5Pos == -1) {
				return;  // panic
			}
			kd = read.substring(delim4Pos + 1, delim5Pos).toDouble();

			//WOBControl.SetTunings(kp, ki, kd);

			Serial.print("P = ");
			Serial.print(kp * 1000);
			Serial.print(" * 10-3; I = ");
			Serial.print(ki * 1000);
			Serial.print(" * 10-3; D = ");
			Serial.println(kd * 1000);
		} break;
		case commandType::INTERROGATE:
		{			
			Serial.print("Goal ");
			Serial.print(_beliefs.currentGoal->describe());
			Serial.print("; sumz ");
			Serial.print(b.hookload);
			Serial.print("; loop time ");
			Serial.println(1);
			for (int i = 0; i < 3; i++) {
				//Serial.print("Button: ");
				//Serial.print(act[i].getPushButton());
				Serial.print("Error: ");
				Serial.print(0);
				Serial.print("; Position: ");
				Serial.print(act[i].getPosition());
				Serial.print(" ; Raw Z: ");
				Serial.println(lc[i].getZ());
			}

		} break;
		case commandType::ZERO_WOB:
		{
			Serial.println("zeroing WOB");
			_beliefs.freeWeight = _beliefs.hookload;
			Serial.print("WOB = ");
			Serial.print(_beliefs.freeWeight);
			Serial.print("; SumZ = ");
			Serial.println(_beliefs.hookload);
		} break;
		case commandType::RESET_STEPPERS:
		{
			act[0].resetSteppers();
			act[1].resetSteppers();
			act[2].resetSteppers();
		} break;
		case commandType::TOGGLE_HAMMER:
		{
			Serial.println("Hammer toggle");
		} break;
		case commandType::TOGGLE_AUTOMATION:
		{
			Serial.println("Automation toggle");
			goalsPush(new BrakesGoal(BrakeStatus::OPENED));
			goalsPush(new )			
			delay(500);
			taggedBottom = false;
			int position1 = act[0].getStepCounter();
			int position2 = act[1].getStepCounter();
			int position3 = act[2].getStepCounter();
			int initialPos = position1 + position2 + position3;
			moveDistance(5.0, HoistingDirectionCommand::UP, 500, 4); //raise up so we know we aren't touching the bottom
			while ((position1 + position2 + position3) > 1)
			{
				//need to detect load cells to reset age counter
				for (int i = 0; i < 3; i++) {
					lc[i].detect();
					lc[i].filter();

					//int sumZFiltered = sumZ;;
					//lowpassFilter(sumZ, &sumZFiltered);
				}

				position1 = act[0].getStepCounter();
				position2 = act[1].getStepCounter();
				position3 = act[2].getStepCounter();

				//wait for it to finish moving
				//put sendData loop in here to send data every so often so we aren't blind
				Serial.println(initialPos);
				Serial.println((position1 + position2 + position3));
			}
			Serial.println("Out");
			delay(1000); //delay just to disipate the energy from stopping
						 //record hook load
			for (int sample = 0; sample < 2 * MEDIAN_WINDOW_SIZE; sample++) {
				for (int i = 0; i < 3; i++) {
					lc[i].detect();
					lc[i].filter();
					free_weight_int = lc[0].getZ() + lc[1].getZ() + lc[2].getZ();
				}
			}
			delay(100);
			Mode = Modes::RESETTINGWOB;
			delay(50);
			sendData();
			delay(100);
			//now go tag bottom
			kp = 0.00000001;
			ki = 0.0000;
			kd = 0.0;
			WOBControl.SetTunings(kp, ki, kd);
			WOBSetpoint_int = 5.0;
			WOBSetpoint_int = 0.2288 * 5000 + 5034.3; // THIS IS WHERE WE DEFINE THE WOB SETPOINT (CONVERTED TO INTEGERS). NOW WOBSP = 5KG
													  // WOBSetpoint_int = (WOBSetpoint_int / 0.101971621) * (3.138 / 200) * (4096 / 3.3);
			WOBControlenabled = true;
			WOBControl.SetMode(AUTOMATIC);
			WOBspeed = 900.0;
			WOBFREQ_Hz = (1 / WOBspeed) * 1000000.0;
			startTimer(TC1, 0, TC3_IRQn, WOBFREQ_Hz);
			Mode = Modes::WOBCONTROL;
			tagBottom();
		} break;
		}

		Serial.println("Receive Data Success!!");
	}
}
