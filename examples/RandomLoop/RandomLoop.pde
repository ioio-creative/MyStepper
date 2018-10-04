#include <MyStepper.h>

MyStepper stepper1(MyStepper::DRIVER, 5, 4);
//MyStepper stepper2(MyStepper::DRIVER, 9, 8);

const int timeStep = 100;  // ms

// about 1600 = 1 revolution for stepper 1

// stepper 1 parameters
const long stepper1DistanceToMove = 3000;
const float stepper1TimeToComplete = 5;  // s
bool isStepper1PrintTimeStepToSerial = false;
bool isStepper1Stopped = false;


void setup()
{
  Serial.begin(9600);
  
  stepper1.reset(stepper1DistanceToMove, stepper1TimeToComplete);
  stepper1.setTimeStepInMillis(timeStep);
  stepper1.setIsPrintTimeStepToSerial(isStepper1PrintTimeStepToSerial);
}

void loop()
{   
  if (!stepper1.isCompleteTotDist())
  {
    stepper1.myRun();
  }
  else {
    Serial.println("Stopped");
    stepper1.printEndStatusToSerial();
    int nextPosition = random(5000,10000);
    int nextTimespan = random(5, 10);
    Serial.println("Next Pos: " + numericToString(nextPosition) + ", Next Timespan: " + numericToString(nextTimespan));
    stepper1.reset(nextPosition, nextTimespan);
  }
}