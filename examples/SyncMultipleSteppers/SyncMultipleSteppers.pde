#include <MyStepper.h>

MyStepper stepper1(MyStepper::DRIVER, 5, 4);
MyStepper stepper2(MyStepper::DRIVER, 9, 8);

const int timeStep = 100;  // ms

// about 1600 = 1 revolution for stepper 1

// stepper 1 parameters
const long stepper1DistanceToMove = 5000;
const float stepper1TimeToComplete = 5;  // s
bool isStepper1PrintTimeStepToSerial = false;
bool isStepper1Stopped = false;

// stepper 2 parameters
const long stepper2DistanceToMove = 10000;
const float stepper2TimeToComplete = 5;  // s
bool isStepper2PrintTimeStepToSerial = false;
bool isStepper2Stopped = false;

void setup()
{
  Serial.begin(9600);
  
  stepper1.reset(stepper1DistanceToMove, stepper1TimeToComplete);
  stepper1.setTimeStepInMillis(timeStep);
  stepper1.setIsPrintTimeStepToSerial(isStepper1PrintTimeStepToSerial);

  stepper2.reset(stepper2DistanceToMove, stepper2TimeToComplete);
  stepper2.setTimeStepInMillis(timeStep);
  stepper2.setIsPrintTimeStepToSerial(isStepper2PrintTimeStepToSerial);
}

void loop()
{   
  bool isStepper1Complete = stepper1.isCompleteTotDist();
  bool isStepper2Complete = stepper2.isCompleteTotDist();
  
  if (!isStepper1Complete)
  {
    stepper1.myRun();
  }

  if (!isStepper2Complete)
  {
    stepper2.myRun();
  }

  if (isStepper1Complete && isStepper2Complete)
  {
    Serial.println("Stopper 1:");
    stepper1.printEndStatusToSerial();
    Serial.println("Stopper 2:");
    stepper2.printEndStatusToSerial();
    
    int nextStepper1Position = random(5000, 30000);
    int nextStepper2Position = random(5000, 30000);
    float nextTimespan = random(15, 40);
    Serial.println("Next Timespan: " + String(nextTimespan, DEC));
    Serial.println("Next Pos 1: " + String(nextStepper1Position, DEC));
    Serial.println("Next Pos 2: " + String(nextStepper2Position, DEC));
    stepper1.reset(nextStepper1Position, nextTimespan);
    stepper2.reset(nextStepper2Position, nextTimespan);
  }
}