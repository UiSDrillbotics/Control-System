#include <PID_v1.h>


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

volatile int testError=0;

int getError() {
  return testError;
}

double Height;
#define STEPS_PER_DISTANCE 250 //steps per mm

class Actuator {
private:

  const int _directionPin, _pulsePin, _endStopperPin, _brakePin;
  volatile float _position;
  volatile bool _calibrated;
  BrakeStatus _brakeStatus;
  volatile int _StepCounter;
  volatile bool _trustedEndStopper;
  int _endStopperMeasurements;
  HoistingDirectionCommand _direction;
  volatile bool _step_wire_on;
  bool _isSetup;

public:
  Actuator(int directionPin, int pulsePin, int endStopperPin, int brakePin) :
    _directionPin(directionPin),
    _pulsePin(pulsePin),
    _endStopperPin(endStopperPin),
    _brakePin(brakePin)
  {
    _isSetup = false;

    _StepCounter = Height;
    _endStopperMeasurements = 0;
    _position = 0;
    _calibrated = false;

    _step_wire_on = false;
  };

  void setup()
  {
    pinMode(_directionPin, OUTPUT);
    pinMode(_pulsePin, OUTPUT);
    pinMode(_endStopperPin, INPUT);
    pinMode(_brakePin, OUTPUT);

    digitalWrite(_directionPin, HIGH); //high = raise and "1" is raise
    _direction = HoistingDirectionCommand::UP;

    digitalWrite(_brakePin, LOW);
    _brakeStatus = BrakeStatus::CLOSED;

    _isSetup = true;
  }
  void stop() {
    _StepCounter = 0;
  }

  void moveDistance(HoistingDirectionCommand dir, int steps)
  {
    if (!_isSetup) return;
    _StepCounter = steps;
    _direction = dir;
    switch (dir) {
    case HoistingDirectionCommand::UP:
      digitalWrite(_directionPin, HIGH);
      break;
    case HoistingDirectionCommand::DOWN:
      digitalWrite(_directionPin, LOW);
      break;
    }
  }

  void resetSteppers()
  {
    _position = (Height)*10;
  }

  void stepInterrupt() {
    if (!_isSetup) return;
    int pushButton = digitalRead(_endStopperPin);
    /*if (pushButton == HIGH) {
      _endStopperMeasurements++;
      if (_endStopperMeasurements > 5) {
        testError = 5;
        _trustedEndStopper = true;
        _position = 0.0;
      }
    }
    else {
      _endStopperMeasurements = 0;
      _trustedEndStopper = false;
    }*/

    if ((_StepCounter <= 0)
      || ((_trustedEndStopper == true) && (_direction == UP))
      || (_brakeStatus == CLOSED))
    {
      //testError = 6;
      _StepCounter = 0;
      // don't do anything
    }
    else {
      digitalWrite(_pulsePin, _step_wire_on = !_step_wire_on);
      _StepCounter--;
      if (_direction == UP)
      {
        //raising
        _position = _position - (0.5 / STEPS_PER_DISTANCE); //dividing by 2 because everytime we enter the interrupt it only executes half a pulse
      }
      else if (_direction == DOWN)
      {
        //lowering
        _position = _position + (0.5 / STEPS_PER_DISTANCE); //dividing by 2 because everytime we enter the interrupt it only executes half a pulse
      }
    }
  }

  int getStepCounter() {
    return _StepCounter;
  }

  float getPosition() {
    return _position;
  }

  BrakeStatus getBrakeStatus() {
    return _brakeStatus;
  }

  void openBrake() {
    if (!_isSetup) return;
    digitalWrite(_brakePin, HIGH);
    _brakeStatus = BrakeStatus::OPENED;
  }

  void closeBrake() {
    if (!_isSetup) return;
    digitalWrite(_brakePin, LOW);
    _brakeStatus = BrakeStatus::CLOSED;
  }

  HoistingDirectionCommand getDirection()
  {
    return _direction;
  }

};

#define MEDIAN_WINDOW_SIZE 11
#define SPACIAL_DIMENSIONS 3

class LoadCell {
private:
  int _array[SPACIAL_DIMENSIONS][MEDIAN_WINDOW_SIZE] = { { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 },
  { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 },
  { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 }
  };
  int _sortedArray[3][MEDIAN_WINDOW_SIZE];
  const int _XPin, _YPin, _ZPin;
  const int _XCorrOffset, _YCorrOffset, _ZCorrOffset;
  //const float _XCorrScale, _YCorrScale, _ZCorrScale;
  bool _isSetup;
  int _raw[SPACIAL_DIMENSIONS];
  int _age;

  int _filtered[SPACIAL_DIMENSIONS];

public:
  LoadCell(int XPin, int YPin, int ZPin, int XCorrOffset, int YCorrOffset, int ZCorrOffset) :
    _XPin(XPin),
    _YPin(YPin),
    _ZPin(ZPin),
    _XCorrOffset(XCorrOffset),
    _YCorrOffset(YCorrOffset),
    _ZCorrOffset(ZCorrOffset)
  {
    _isSetup = false;
  }

  void setup() {
    _isSetup = true;
    detect();
  }

  void detect() {
    _raw[0] = (int)(/*_XCorrScale * */analogRead(_XPin) + _XCorrOffset);
    _raw[1] = (int)(/*_YCorrScale * */analogRead(_YPin) + _YCorrOffset);
    _raw[2] = (int)(/*_ZCorrScale * */analogRead(_ZPin) + _ZCorrOffset);
    _age = 0;
  }

  void incAge() {
    _age++;
  }

  int getAge() {
    return _age;
  }

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

  void filter() {
    // this is a median filter;
    for (int i = 0; i < SPACIAL_DIMENSIONS; i++) {
      for (int q = 0; q < MEDIAN_WINDOW_SIZE; q++)
      {
        _array[i][q] = _array[i][q + 1];
      }
      _array[i][10] = _raw[i];

      for (int p = 0; p < MEDIAN_WINDOW_SIZE; p++)
      {
        _sortedArray[i][p] = _array[i][p];
      }
      sort(_sortedArray[i], MEDIAN_WINDOW_SIZE);

      _filtered[i] = _sortedArray[i][5];

    }
  }

  double y[2];
  double x[3];
  double BW = 0.0066;
  double z1x;
  double b0, b1, b2;
  double a1, a2;
  
  void lowpassFilter(int loadcell, int *output) {
    // this is a notch filter;
    int cutoff = 50; //going to try 50hz at first
    int sampleFrequency = 13; //this is looptime of 77ms

    z1x = cos(2 * PI * cutoff / sampleFrequency);
    b0 = (1 - BW) * (1 - BW) / (2 * (abs(z1x) + 1)) + BW;
    b2 = b0;
    b1 = -2 * z1x * b0;
    a1 = -2 * z1x * BW;
    a2 = BW * BW;

    *output = (b0 * x[0]) + (b1 * x[1]) + (b2 * x[2]) - (a1 * y[0]) - (a2 * y[1]);
    y[1] = y[0];
    y[0] = *output;
    x[2] = x[1];
    x[1] = x[0];
    x[0] = loadcell;

    /*  for (int i = 0; i < SPACIAL_DIMENSIONS; i++) {
    alpha = sampleFrequency / (cutoff + sampleFrequency);
    _filtered[i] = (alpha * _filtered[i]) + ((1 - alpha)*_filtered[i]);

    _filtered[i] = ema_s[i];

    Serial.print("i = ");
    Serial.print(i);
    Serial.print("; filtered[i] = ");
    Serial.println(_filtered[i]);
    }*/
  }

  double alpha = 500; //initialize alpha
  double ema_s[3];
};

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

const char END_OF_TRANSMISSION = '\n';

const int heightSensor = A7; //was A9
double heightReading;



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



enum Modes {
  START = 0,
  CALIBRATING = 1,
  MOVING = 2,
  STOPPED = 3,
  WOBCONTROL = 4,
  HAMMER = 5,
  TAGGEDBOTTOM = 6,
  RESETTINGWOB = 7,
  DONECALIBRATING = 8,
  NORMALVIBRATIONS = 9,
  DAMAGINGVIBRATIONS = 10
};
Modes Mode; //0 = Start; 1 = Calibrate; 2 = Moving; 3 = Stopped;
bool calibrated;

float start_time;
float finish_time;
float cmdDistance;
HoistingDirectionCommand cmdDirection;
BrakeCommand cmdBrakes;
float cmdSpeed;

commandType Command;
int cmdActuator;
errorTypes Error;
BrakeStatus brakeStatus;

int currentTime;
int timeStep = 10; // in ms
int endOfLoopTime;
int loopTime;
int heightSensingCounter = 0;




double WOBspeed;
double WOBFREQ_Hz;

Actuator act[3] = { Actuator(direction_wire1, step_wire1, pushButton1, Brake1),
Actuator(direction_wire2, step_wire2, pushButton2, Brake2),
Actuator(direction_wire3, step_wire3, pushButton3, Brake3)
};

LoadCell lc[3] = { LoadCell(loadcellX1, loadcellY1, loadcellZ1, 0,     0, 0),
LoadCell(loadcellX2, loadcellY2, loadcellZ2, 0, 0,    0),
LoadCell(loadcellX3, loadcellY3, loadcellZ3, 0,     0,  0)
};



//step is 0.18 degrees
//revolution = 360/1.8 = 2000
//1 REVOLUTION = 8MM MOVEMENT (= 250 STEPS = 1 MM)

int sumZ_int;
int avgZ;
int avgCnt;
int free_weight_int;

double WOB_Measured_int;
double WOBControllerOutput;
double WOBSetpoint_int;
double sumZ_double;
double WOBLog[501]; // was 50

bool WOBControlenabled = false;
bool activateHammerDrill = false;
HoistingDirectionCommand preDirection = HoistingDirectionCommand::DOWN;

PID WOBControl(&WOB_Measured_int, &WOBControllerOutput, &WOBSetpoint_int, 0.0001, 0.0001, 0, DIRECT);

bool telemetry_on = true;

double SumZLog[500]; // was 49



void setup()
{
  WOBControl.SetOutputLimits(-1, 1);
  Serial.begin(115200); //experiment more with this. changing to 115200 drastically sped things up
  analogReadResolution(12);
  act[0].setup();
  act[1].setup();
  act[2].setup();
  lc[0].setup();
  lc[1].setup();
  lc[2].setup();
  OpenBrakes();
  for (int sample = 0; sample < 2 * MEDIAN_WINDOW_SIZE; sample++) {
    for (int i = 0; i < 3; i++) {
      lc[i].detect();
      lc[i].filter();
      //the following equations are from calibrating z1 and z3 using lab scale
      free_weight_int = (lc[0].getZ() + lc[1].getZ() + lc[2].getZ());
    }
  }
  WOBSetpoint_int = 0;
  WOBControl.SetMode(MANUAL);
  WOBControl.SetSampleTime(25); // was 100 before today.
  double FREQ_Hz = 1;
  startTimer(TC1, 0, TC3_IRQn, FREQ_Hz);
  Mode = Modes::START; //Starting
  for (int i = 0; i < 500; i++) { // was 49 before today.
    SumZLog[i] = 0;
  }
  sendData();
}

volatile boolean motorsDisabled = false;


const int upperMargin = 4000;
const int lowerMargin = 2000;
double upperStepcounter = 50;
double lowerStepcounter = 850;
const int maxLoadCellAge = 1000;

volatile int loadCellAge;
int sendInterval = 10 * 1000; //



void TC3_Handler() {
  TC_GetStatus(TC1, 0);

  // may we move?
  if (motorsDisabled) {
    testError = 1;
    return;
  }
  for (int i = 0; i < 3; i++) {
    if (lc[i].getAge() > maxLoadCellAge) {
      testError = 2;
      return;
    }
    //putting something here so it doesnt get stuck

    bool shouldStop = false;
    if (lc[i].getZ() > upperMargin || act[i].getPosition() < upperStepcounter)
    {
      for (int i = 0; i < 3; i++) {
        if (act[i].getDirection() == HoistingDirectionCommand::UP) {
          testError = 3;
          shouldStop = true;
          return;
        }
        //move down immediately
        //remember how far we were supposed to move? maybe not      
      }
    }
    if (lc[i].getZ() < lowerMargin || act[i].getPosition() > lowerStepcounter)
    {
      for (int i = 0; i < 3; i++) {
        if (act[i].getDirection() == HoistingDirectionCommand::DOWN) {
          testError = 4;
          shouldStop = true;
          return;
        }
        //move down immediately
        //remember how far we were supposed to move? maybe not      
      }
    }

    //if (shouldStop == true) { for (int i = 0; i < 3; i++) { act[i].stop(); } }

    lc[i].incAge();
    for (int i = 0; i < 3; i++) {
      act[i].stepInterrupt();
    }
  }
}

void startTimer(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t frequency) {

  //Enable or disable write protect of PMC registers.
  pmc_set_writeprotect(false);
  //Enable the specified peripheral clock.
  pmc_enable_periph_clk((uint32_t)irq);

  TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK4);
  uint32_t rc = VARIANT_MCK / 128 / frequency * 10;

  TC_SetRA(tc, channel, rc / 2);
  TC_SetRC(tc, channel, rc);
  TC_Start(tc, channel);

  tc->TC_CHANNEL[channel].TC_IER = TC_IER_CPCS;
  tc->TC_CHANNEL[channel].TC_IDR = ~TC_IER_CPCS;
  NVIC_EnableIRQ(irq);

}

// #define RING_SIZE 50

// int ring[RING_SIZE];
// uint ringIdx = 0;
bool taggedBottom = false;

int sumZ_numerator;
int sumZ_denominator = 0;
int hammerTimer = 0;
int oldHammerTime = 0;

int loopCounter = 0;
int loopTimer;

double totalDepth = 0;

double rop = NAN; // mm / 15s
double rop3m = NAN;
int rop3MinMeasureTime;
double lastRopPosition3m;

int ropMeasureTime; //milli seconds
double lastRopPosition; //mm
const int ropMeasureInterval = 15000; //milli seconds
int currentMilliSec;

int sendTime = micros();
int lastWorkDoneTime = 0;
bool setAccHeight = false;
double Looping;

void loop()
{
  currentTime = micros();
  currentMilliSec = millis();

  totalDepth = (act[1].getPosition() > totalDepth) ? act[1].getPosition() : totalDepth;

  //if ((currentTime - ropMeasureTime) > ropMeasureInterval)
  //{
  //  double pos = totalDepth;
  //  rop = pos - lastRopPosition;
  //  lastRopPosition = pos;
  //}

  //if ((currentTime - rop3MinMeasureTime) > 180000)
  //{
  //  double pos3m = totalDepth;
  //  rop3m = pos3m - lastRopPosition3m;
  //  lastRopPosition3m = pos3m;
  //}


  if ((taggedBottom == false) && (WOB_Measured_int > WOBSetpoint_int)) {
    taggedBottom = true;
    sendData;
  }

  for (int i = 0; i < 3; i++) {
    lc[i].detect();
    lc[i].filter();

    //int sumZFiltered = sumZ;;
    //lowpassFilter(sumZ, &sumZFiltered);
  }

  //the following equations are from calibrating z1 and z3 using lab scale
  // sumZ unit: 60 kg / 3*4096 = 4g
  //if (sumZ_denominator > 50)
  //{
  //  sumZ_numerator = 0;
  //  sumZ_denominator = 0;
  //}
  sumZ_int = (lc[0].getZ() + lc[1].getZ() + lc[2].getZ());
  int sumZ_movingAvg = 0;
  for (int i = 0; i < 500; i++) //originally 9 // was 49 before we edited today.
  {
    SumZLog[i] = SumZLog[i + 1];
    sumZ_movingAvg += SumZLog[i];
  }
  SumZLog[500] = sumZ_int; //originally 9 // was 49 before we edited today.
  sumZ_movingAvg += SumZLog[500];
  sumZ_int = sumZ_movingAvg / 500; // was 49 before we edited today.
  sumZ_numerator += sumZ_int;
  sumZ_denominator++;
  //sumZ_int = sumZ_numerator / sumZ_denominator;
  WOB_Measured_int = free_weight_int - sumZ_int;
  //ringIdx = ringIdx++ % RING_SIZE;

  if (Serial.available())
  {
    ReceiveData();
    //Serial.print("executing ");
    //Serial.println(Command);
    if (Command == commandType::STOP) {
      motorsDisabled = true;
      Mode = Modes::STOPPED;
    }
    if (cmdBrakes == BrakeCommand::OPEN && brakeStatus == BrakeStatus::CLOSED)
    {
      OpenBrakes();
      sendData();
      //Brake = 2.0;
    }
    else if (cmdBrakes == BrakeCommand::CLOSE)
    {
      CloseBrakes();
      brakeStatus == 0;
      sendData();

    }
    if (Command == commandType::MOVE) //move distance
    {
      //Serial.println("matched MOVE");
      Mode = Modes::MOVING;
      motorsDisabled = false;
      moveDistance(cmdDistance, cmdDirection, cmdSpeed, cmdActuator);
      Command = commandType::NOTHING;
    }

    else if (Command == commandType::CALIBRATE) //calibrate
    {

      Mode = Modes::CALIBRATING;

      //calibratePosition(); //edit
      Mode = Modes::STOPPED;
    }

    else
    {
      Error = errorTypes::BAD_COMMAND;
      //sendData();
      //Serial.flush();
    }
  }

  
  heightReading = analogRead(heightSensor);
  Height = ((0.2372*heightReading) + 1.8353)/10;
  if (setAccHeight == false){
    for (int i = 0; i < 3; i++) {
      act[i].resetSteppers();
        
    }
    setAccHeight = true;

  }

  //Height = (5.0 / 3.3) * (27010.0 * pow(heightReading * 0.25, -1.14));//0.25 is because we initially calibrated with 10 bit read resolution and now we have 12 bit      //HERE!
  if ((micros() - sendTime) > sendInterval)
  {
    //putting this here so the modes in the automation command don't get reset (i.e. tagged bottom/done calibrating)
    if (act[0].getStepCounter() + act[1].getStepCounter() + act[2].getStepCounter() == 0)
    {
      Mode = Modes::STOPPED;
    }
    sendData();
    sendTime = micros();
    //loopCounter = 0;
  }
  else
  {
    loopCounter++;
  }

  if (WOBControlenabled)
  {
    for (int i = 0; i < 500; i++) //originally 9 // was 49 before today
    {
      WOBLog[i] = WOBLog[i + 1];
    }
    WOBLog[500] = WOB_Measured_int; //originally 9 // was 49 before today
    bool workDone = WOBControl.Compute();
    Looping = millis();
    if (workDone) {
      double PIDerror = WOBSetpoint_int  - WOB_Measured_int;
      if (!telemetry_on) {
        Serial.print("pid-err: ");
        Serial.println(PIDerror);
        Serial.print("Loop-time: ");
        Serial.println(Looping);
        Serial.print("WOB Controller Output: ");
        Serial.println(WOBControllerOutput);
        
      }
      if (PIDerror < -1000) {
        HoistingDirectionCommand dir = HoistingDirectionCommand::UP;
        int speed = 900; 
        int allActuators = 4;
        double StepOutput = abs(WOBControllerOutput);
        if (StepOutput > 0.04)
        {
          StepOutput = 0.04;
        }
        moveDistance(StepOutput, dir, speed, allActuators);
      }
      else if (PIDerror > 100) { 
        HoistingDirectionCommand dir = HoistingDirectionCommand::DOWN;
        int speed = 900;
        int allActuators = 4;
        double StepOutput = abs(WOBControllerOutput);
        if (StepOutput > 0.04) // was 1.0
        {
          StepOutput = 0.04; // was 1.0
        }
        moveDistance(StepOutput, dir, speed, allActuators);
       }

      else {
        // deadband, we ignore the output.
      }
    }

    //check for vibrations
    int vibrationNormalCounter = 0;
    int vibrationDamageCounter = 0;
    //int vibrationNormalLimit = ((WOBSetpoint * 1.5) / 0.101971621) * (3.138 / 200) * (4096 / 3.3);
    int vibrationNormalLimit = WOBSetpoint_int * 200.0; //already being sent as an int
    //int vibrationDamageLimit = (10.0 / 0.101971621) * (3.138 / 200) * (4096 / 3.3);
    int vibrationDamageLimit = (28 / 0.101971621) * (3.138 / 200) * (4096 / 3.3); // was WOBSetpoint_int * 4.0
      for (int i = 0; i < 49; i++)
    {
      if (WOBLog[i] > vibrationNormalLimit)
      {
        vibrationNormalCounter++;
      }
      if (WOBLog[i] > vibrationDamageLimit)
      {
        vibrationDamageCounter++;
      }
    }
    double averageWOB = 0.0;
    if (vibrationNormalCounter > 10 && vibrationNormalCounter < 35)
    {
      //vibrations
      Mode = Modes::NORMALVIBRATIONS;
      sendData();
    }
    else if (vibrationDamageCounter > 5 && vibrationDamageCounter < 35)
    {
      //damaging vibrations
      Mode = Modes::DAMAGINGVIBRATIONS;
      //moveDistance(1.0, HoistingDirectionCommand::UP, 100, 4);
      //preDirection = HoistingDirectionCommand::UP;
      sendData();
    }

    //end checking for vibrations





  }
  endOfLoopTime = micros();
  loopTime = endOfLoopTime - currentTime;
  //if (endOfLoopTime > currentTime)
  //{
  //  if ((endOfLoopTime - currentTime) < timeStep)
  //  {
  //    delay(timeStep - endOfLoopTime + currentTime);
  //  }
  //}
  //Serial.print("Loop time = ");
  //Serial.println(endOfLoopTime - currentTime);
}

void sort(int a[], int size)
{
  for (int k = 0; k < (size - 1); k++)
  {
    for (int o = 0; o < (size - (k + 1)); o++)
    {
      if (a[o] > a[o + 1])
      {
        int t = a[o];
        a[o] = a[o + 1];
        a[o + 1] = t;
      }
    }
  }
}
double kp;
double ki;
double kd;
void ReceiveData()
{
  int bytes = Serial.available();
  if (bytes > 0) {
    String read = Serial.readStringUntil(END_OF_TRANSMISSION);
    int delim1Pos = read.indexOf(';');
    String CommandString = (delim1Pos == -1) ? read : read.substring(0, delim1Pos);
    int commandCode = CommandString.toInt();
    if (commandCode == 0) {
      return; // panic;6
    }

    commandType currentCommand = (commandType)commandCode;

    float tmpdistance = cmdDistance, tmpspeed = cmdSpeed;
    HoistingDirectionCommand tmpdirection = cmdDirection;
    int tmpactuator = cmdActuator;
    BrakeCommand tmpbrake = cmdBrakes;
    int delim2Pos = -1, delim3Pos = -1, delim4Pos = -1, delim5Pos = -1;
    switch (currentCommand) {
    case commandType::STOP:
      tmpdistance = 0;
      tmpspeed = 0;
      tmpdirection = HoistingDirectionCommand::UP;
      tmpactuator = 4;
      break;
    case commandType::MOVE:
      delim2Pos = read.indexOf(';', delim1Pos + 1);
      if (delim2Pos == -1) {
        return;  // panic
      }
      tmpdistance = read.substring(delim1Pos + 1, delim2Pos).toFloat();

      delim3Pos = read.indexOf(';', delim2Pos + 1);
      if (delim3Pos == -1) {
        return;  // panic
      }
      switch (read.substring(delim2Pos + 1, delim3Pos).toInt())
      {
      case 1: tmpdirection = HoistingDirectionCommand::UP;
        break;
      case 2: tmpdirection = HoistingDirectionCommand::DOWN;
        break;
      default: Error = errorTypes::BAD_COMMAND_STRING;
        return; //panic
      }

      delim4Pos = read.indexOf(';', delim3Pos + 1);
      if (delim4Pos == -1) {
        return;  // panic
      }
      tmpspeed = read.substring(delim3Pos + 1, delim4Pos).toFloat();

      delim5Pos = read.indexOf(';', delim4Pos + 1);
      if (delim5Pos == -1) {
        return;  // panic
      }
      tmpactuator = read.substring(delim4Pos + 1, delim5Pos).toInt();
      if (!telemetry_on) {
        Serial.print("speed:");
        Serial.print(tmpspeed);
      }
      break;
    case commandType::CALIBRATE:
      break;
    case commandType::BRAKES:
      delim2Pos = read.indexOf(';', delim1Pos + 1);
      if (delim2Pos == -1) {
        Serial.println("panic in brake arg");
        return;
      } // panic
      switch (read.substring(delim1Pos + 1, delim2Pos).toInt())
      {
      case 1: tmpbrake = BrakeCommand::CLOSE;
        break;
      case 2: tmpbrake = BrakeCommand::OPEN;
        break;
      default: Error = errorTypes::BAD_COMMAND;
        return; //panic!
      }
      break;
    case commandType::TOGGLE_TELEMTRY:
      telemetry_on = !telemetry_on;
      break;
    case commandType::TOGGLE_WOBCONTROL:
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
        WOBControl.SetMode(AUTOMATIC);
        WOBspeed = 300.0;
        WOBFREQ_Hz = (1 / WOBspeed) * 1000000.0;
        startTimer(TC1, 0, TC3_IRQn, WOBFREQ_Hz);
        Mode = Modes::WOBCONTROL;
        Serial.println(Mode);
      }
      else {
        WOBControl.SetMode(MANUAL);
        Mode = Modes::STOPPED;
      }
      Serial.println(WOBControlenabled);
      break;
    case commandType::SET_PID_TUNINGS:
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

      WOBControl.SetTunings(kp, ki, kd);

      Serial.print("P = ");
      Serial.print(WOBControl.GetKp() * 1000);
      Serial.print(" * 10-3; I = ");
      Serial.print(WOBControl.GetKi()*1000);
      Serial.print(" * 10-3; D = ");
      Serial.println(WOBControl.GetKd()*1000);
      break;
    case commandType::INTERROGATE:
      Serial.print("WOB ");
      Serial.print(WOB_Measured_int);
      Serial.print("; sumz ");
      Serial.print(sumZ_int);
      Serial.print("; loop time ");
      Serial.println(loopTime);
      for (int i = 0; i < 3; i++) {
        //Serial.print("Button: ");
        //Serial.print(act[i].getPushButton());
        Serial.print("Error: ");
        Serial.print(getError());
        Serial.print("; Position: ");
        Serial.print(act[i].getPosition());
        Serial.print(" ; Raw Z: ");
        Serial.println(lc[i].getZ());
      }

      break;
    case commandType::ZERO_WOB:
      Serial.println("zeroing WOB");
      for (int sample = 0; sample < 2 * MEDIAN_WINDOW_SIZE; sample++) {
        for (int i = 0; i < 3; i++) {
          lc[i].detect();
          lc[i].filter();
          free_weight_int = lc[0].getZ() + lc[1].getZ() + lc[2].getZ();
        }
      }
      Serial.print("WOB = ");
      Serial.print(free_weight_int);
      Serial.print("; SumZ = ");
      Serial.println(sumZ_int);
      break;
    case commandType::RESET_STEPPERS:

      act[0].resetSteppers();
      act[1].resetSteppers();
      act[2].resetSteppers();
      break;
    case commandType::TOGGLE_HAMMER:

      Serial.println("Hammer toggle");
      activateHammerDrill = !activateHammerDrill;
      if (activateHammerDrill)
      {
        Mode = Modes::HAMMER;
        //Serial.println(Mode);     
      }
      break;
    case commandType::TOGGLE_AUTOMATION:

      Serial.println("Automation toggle");
      OpenBrakes();
      delay(500);
      taggedBottom = false;
      int position1 = act[0].getStepCounter();
      int position2 = act[1].getStepCounter();
      int position3 = act[2].getStepCounter();
      int initialPos = position1 + position2 + position3;
      moveDistance(5.0, HoistingDirectionCommand::UP, 500, 4); //raise up so we know we aren't touching the bottom
      while ((position1 + position2 + position3)  > 1)
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
      WOBSetpoint_int = 0.2288*5000 + 5034.3; // THIS IS WHERE WE DEFINE THE WOB SETPOINT (CONVERTED TO INTEGERS). NOW WOBSP = 5KG
      // WOBSetpoint_int = (WOBSetpoint_int / 0.101971621) * (3.138 / 200) * (4096 / 3.3);
      WOBControlenabled = true;
      WOBControl.SetMode(AUTOMATIC);
      WOBspeed = 900.0;
      WOBFREQ_Hz = (1 / WOBspeed) * 1000000.0;
      startTimer(TC1, 0, TC3_IRQn, WOBFREQ_Hz);
      Mode = Modes::WOBCONTROL;
      tagBottom();
      break;
    }

    Serial.println("Receive Data Success!!");
    cmdDistance = tmpdistance;
    cmdDirection = tmpdirection;
    cmdSpeed = tmpspeed;
    Command = currentCommand;
    cmdActuator = tmpactuator;
    cmdBrakes = tmpbrake;
  }
}

void OpenBrakes()
{
  for (int i = 0; i < 3; i++) {
    act[i].openBrake();
  }
  brakeStatus = BrakeStatus::OPENED;
}

void CloseBrakes()
{
  for (int i = 0; i < 3; i++) {
    act[i].closeBrake();
  }
  brakeStatus = BrakeStatus::CLOSED;
}

void calibratePosition()
{
  calibrated = false;
  while (!calibrated) {
    cmdDistance = 5;
    cmdSpeed = 200;
    cmdDirection = HoistingDirectionCommand::UP;
    cmdActuator = 4;
    lc[0].detect();
    lc[1].detect();
    lc[2].detect();
    moveDistance(cmdDistance, cmdDirection, cmdSpeed, cmdActuator);
    int pb1 = digitalRead(pushButton1);
    int pb2 = digitalRead(pushButton1);
    int pb3 = digitalRead(pushButton1);
    calibrated = ((pb1 == HIGH) && (pb2 = HIGH) && (pb3 == HIGH));
    sendData();
  }
  act[0].getPosition();
  act[1].getPosition();
  act[2].getPosition();
}

void tagBottom()
{
  Serial.println("In");
  taggedBottom = false;
  int tagBottomCounter = 0;
  int sendingDataCounter = 0;
  while (!taggedBottom) {
    for (int i = 0; i < 3; i++) {
      lc[i].detect();
      lc[i].filter();

      //int sumZFiltered = sumZ;;
      //lowpassFilter(sumZ, &sumZFiltered);
    }

    //if (sumZ_denominator > 50)
    //{
    //  sumZ_numerator = 0;
    //  sumZ_denominator = 0;
    //}
    sumZ_numerator += (lc[0].getZ() + lc[1].getZ() + lc[2].getZ());
    sumZ_denominator++;
    sumZ_int = sumZ_numerator / sumZ_denominator;
    WOB_Measured_int = free_weight_int - sumZ_int;

    bool workDone = WOBControl.Compute();
    if (workDone) {
      HoistingDirectionCommand dir = HoistingDirectionCommand::DOWN;
      int speed = 900;
      int allActuators = 4;
      double StepOutput = abs(WOBControllerOutput);
      if (StepOutput > 0.5)// was 1 mm
      {
        StepOutput = 0.5;//was 1 mm
      }
      moveDistance(StepOutput, dir, speed, allActuators);
      sumZ_numerator = 0;
      sumZ_denominator = 0;

      //Serial.print("PID Output = ");
      //Serial.println(WOBControllerOutput);
      if (WOBControllerOutput < 0.0)
      {
        tagBottomCounter++;
        if (tagBottomCounter > 3)
        {
          taggedBottom = true;
          WOBControl.SetMode(MANUAL);
          WOBControllerOutput = 0.0;

          //reset stepper position 
          for (int i = 0; i < 3; i++) {
            act[i].resetSteppers();
          }
          WOBControlenabled = false;
          //telemetry_on = true;
          Mode = Modes::TAGGEDBOTTOM;
          sendData();
        }
      }
      else
      {
        tagBottomCounter = 0;
      }

    }
    sendingDataCounter++;
    if (sendingDataCounter > 100)
    {
      sendData();
      sendingDataCounter = 0;
    }
  }


  act[0].getPosition();
  act[1].getPosition();
  act[2].getPosition();
  moveDistance(5.0, 1, 300, 4);
  delay(1000);
  int position1 = 5;
  int position2 = 5;
  int position3 = 5;
  while (position1 + position2 + position3 > 0)
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

    //put sendData loop in here to send data every so often so we aren't blind
  }
  delay(2000);
  Mode = Modes::DONECALIBRATING;
  sendData();
}

void moveDistance(float distance, int dir,
  int speed, int actuator)
{
  HoistingDirectionCommand direction = (HoistingDirectionCommand)dir;
  //Serial.print("moving ");
  //Serial.println(distance);
  //Serial.print(";");
  //Serial.print(direction);
  //Serial.print(";");
  //Serial.print(speed);
  //Serial.print(";");
  //Serial.print(actuator);
  //Serial.print(";");

  // Check parameters
  if ((actuator > 4) || (actuator < 1)) {
    Error = errorTypes::BAD_COMMAND;
    Serial.println("Bad Actuator");
    return;
  }

  if ((direction != HoistingDirectionCommand::UP) &&
    (direction != HoistingDirectionCommand::DOWN)) {
    Serial.println("Bad Direction");
    return;
  }

  if ((speed < 30.0) || (speed > 1100)) {
    Serial.println("Bad Speed");
    return;
  }

  // All parameters OK;
  // int steps = Distance * steps_per_distance;             edit

  if (direction == HoistingDirectionCommand::UP) {
    taggedBottom = false;
  }

  int steps = (int)(distance * STEPS_PER_DISTANCE);
  //start_time = millis();
  double FREQ_Hz = (1 / (double)speed) * 1000000.0;

  if (actuator == 4) {
    for (int i = 0; i < 3; i++) {
      act[i].moveDistance(direction, steps * 2.0); //multiply by 2 because the interrupt only does half a step
    }
  }
  else if (actuator < 4) {
    act[actuator - 1].moveDistance(direction, steps * 2.0); //multiply by 2 because the interrupt only does half a step
  }
  if (!WOBControlenabled) {
    startTimer(TC1, 0, TC3_IRQn, FREQ_Hz);
  }
  //Serial.println(FREQ_Hz);
  //Serial.println("in actuator");

  finish_time = millis() - start_time;

}

void sendData()
{
  if (!telemetry_on) {
    return;
  }

  if (WOBControlenabled && Mode != NORMALVIBRATIONS && Mode != DAMAGINGVIBRATIONS) {
    Mode = Modes::WOBCONTROL;
  }

  if (activateHammerDrill) {
    Mode = Modes::HAMMER;
  }




  Serial.write("x");
  Serial.print(Mode);
  Serial.write("y");
  Serial.write(cmdDirection + 48);
  Serial.write("y");
  Serial.write(cmdBrakes + 48);
  Serial.write("y");
  Serial.print(Height);
  for (int i = 0; i < 3; i++) {
    Serial.write("y");
    Serial.print(act[i].getPosition());
  }
  for (int i = 0; i < SPACIAL_DIMENSIONS; i++) {
    Serial.write("y");
    Serial.print(lc[i].getRawX());
  }
  for (int i = 0; i < 3; i++) {
    Serial.write("y");
    Serial.print(lc[i].getRawY());
  }

  for (int i = 0; i < 3; i++) {
    Serial.write("y");
    Serial.print(lc[i].getZ());
  }
  Serial.write("y");
  Serial.print(rop);
  Serial.write("y");
  Serial.print(rop3m);
  Serial.write("y");
  Serial.print("z"); // was: Serial.println("z");
  Serial.write("WOB_output: "); // delete if unecessary
  Serial.println(WOBControllerOutput); // delete if unecessary
}

// float sum = 0;
// for (int i=0; i<16; i++)
// {
// sum += analogRead(heightSensor);
// }
//  float volts = sum/16  * 0.00322265625;         // value from sensor * (3.3/1024) - running 3.3V
//  float Height = 65*pow(volts, -1.10);
// heightReading = analogRead(heightSensor);

// Height = 27010.0 * pow(sum/16, -1.14);

