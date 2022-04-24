#include <Servo.h>

#include "ASE_MotorConstants.h";


const float timestep = 1; //In microseconds

int stepperPins[4] = {4, 5, 6, 7};
int dirPins[4] = {27, 28, 29, 30};
int enablePins[4] = {23, 24, 25, 26};
int controllerMin[4] = {0, 0, 0, 0};
int controllerMax[4] = {1023, 1023, 1008, 1022};

int pausePin = 18, /*estopPin = 19,*/ zeroPin = 20, setMaxPin = 21, setMaxSelectorPin = 22,
    controlSelectPin = 16, pauseLED = 15, estopLED = 14, controlModeLED = 17;
    


unsigned int tcnt[4] = {0,0,0,0}; //Time counters for all 4 steppers in microseconds
                                  //Resets at each movement end
                                
volatile int pos[4] = {0,0,0,0};  //Step counters for all 4 steppers;
                                  //Is zeroed at upright

int targetPos[4];

bool isMoving[4] = {false, false, false, false};

int posLimit[2] = {0,0};
bool limitSet[2] = {false, false};

bool dir[4]; //True is negative, false is positive

float period[4]; //In microseconds

volatile bool paused, ESTOP, controlType; //controlType is position based when true and speed based while false

int controllerVal[4] = {0, 0, 0, 0};
int speedIn = 0;
long lastTime = 0, deltaTime = 0;


void setup()
{
  Serial.begin(115200);

  paused = true;
  ESTOP = false;
  
  for (int i = 0; i < 4; i++)
  {
    pinMode(stepperPins[i], OUTPUT);
    pinMode(dirPins[i], OUTPUT);
    pinMode(enablePins[i], OUTPUT);
    period[i] = MINPERIOD * 100;
  }
  pinMode(pauseLED, OUTPUT);                //LED
  pinMode(estopLED, OUTPUT);                //LED
  pinMode(controlModeLED, OUTPUT);                //LED
  pinMode(pausePin, INPUT_PULLUP);          //Push button to ground
  //pinMode(estopPin, INPUT_PULLUP);          //Push button to ground
  pinMode(zeroPin, INPUT_PULLUP);           //Push button to ground
  pinMode(setMaxPin, INPUT_PULLUP);         //Push button to ground
  pinMode(setMaxSelectorPin, INPUT_PULLUP);        //Toggle switch
  pinMode(controlSelectPin, INPUT_PULLUP);         //Toggle switch

  attachInterrupt(digitalPinToInterrupt(pausePin), pause, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(estopPin), estop, FALLING);
  attachInterrupt(digitalPinToInterrupt(zeroPin), zero, CHANGE);
  attachInterrupt(digitalPinToInterrupt(setMaxPin), setMax, CHANGE);

  
  digitalWrite(pauseLED, paused);

  eyeStart();
}

void loop()
{
  delayMicroseconds(timestep);
  deltaTime = micros() - lastTime;

  //Only allow for position control if limits have been set
  if (limitSet[0] && limitSet[1])
  {
    controlType = digitalRead(controlSelectPin);
  }
  else
  {
    controlType = false;
  }
  
  if (paused) //The steppers will be on HOLD while paused
  {
    //IDK, it's paused
  }
  else //If not paused, update controller inputs and motors
  {
    getControllerValues();
    checkStepTime();
  }

/*
  if (lastTime % 100 >= 92)
  {
    Serial.print(pos[0]);
    Serial.print(" ");
    Serial.print(map(controllerVal[0], 0, 1023, -posLimit[0], posLimit[0]));
    Serial.print(" ");
    Serial.print(posLimit[0]);
    Serial.print(" ");
    Serial.println(-1 * posLimit[0]);
  }
  */

  eyeLoop();
  
  lastTime = micros();
}

void getControllerValues()
{
  controllerVal[0] = map(analogRead(A0), controllerMin[0], controllerMax[0], 0, 1024);
  controllerVal[1] = map(analogRead(A1), controllerMin[1], controllerMax[1], 0, 1024);
  controllerVal[2] = map(analogRead(A2), controllerMin[2], controllerMax[2], 0, 1024);
  controllerVal[3] = map(analogRead(A3), controllerMin[3], controllerMax[3], 0, 1024);

  speedIn = analogRead(A4);

  if (controlType)
  {
    //Position control
    digitalWrite(estopLED, HIGH);
    targetPos[0] = (int)map(controllerVal[0], 0, 1023, -posLimit[0], posLimit[0]);
    targetPos[1] = (int)map(controllerVal[1], 0, 1023, -posLimit[0], posLimit[0]);
    targetPos[2] = (int)map(controllerVal[2], 0, 1023, -posLimit[1], posLimit[1]);
    targetPos[3] = (int)map(controllerVal[3], 0, 1023, -posLimit[1], posLimit[1]);
  
    for (int i = 0; i < 4; i++)
    {
      if ((!isMoving[i] && abs(targetPos[i] - pos[i]) > 2) || (isMoving[i] && targetPos[i] != pos[i]))
      {
        isMoving[i] = true;
        if (targetPos[i] > pos[i])
        {
          dir[i] = false; //Move in positive direction
        }
        else
        {
          dir[i] = true; //Move in negative direction
        }
      }
      else
      {
        isMoving[i] = false;
        tcnt[i] = 0;
      }

      period[i] = MINPERIOD / (speedIn / 1023.0);
    }
  }
  else
  {
    //Speed control
    digitalWrite(estopLED, LOW);
    for (int i = 0; i < 4; i++)
    {
      period[i] = MINPERIOD * (1023.0 / speedIn) * (511.0 / (abs(controllerVal[i] - 511)) );
      dir[i] = (controllerVal[i] < 511) ? true : false;
      isMoving[i] = true;
    }
  }
}

void checkStepTime()
{
  for (int i = 0; i < 4; i++)
  {
    if (isMoving[i])
    {
      tcnt[i] += deltaTime;
      if (tcnt[i] >= period[i]/2)
      {
        stepMotor(tcnt + i, pos + i, stepperPins + i, dir[i]);
      }
      digitalWrite(dirPins[i], dir[i]);
    }
  }
}

void zero()
{
  for (int i = 0; i < 4; i++)
  {
    pos[i] = 0;
  }
}

void pause()
{
  paused = digitalRead(pausePin);
  digitalWrite(pauseLED, paused);
  for (int i = 0; i < 4; i++)
  {
    digitalWrite(stepperPins[i], LOW);
  }
}

void estop()
{
  if (!ESTOP)
  {
   //digitalWrite(enablePin, true); //Disables the steppers when high
   ESTOP = true; 
  }
  else
  {
    ESTOP = false;
    reset();
  }
  digitalWrite(estopLED, ESTOP);
}

void setMax()
{
  if (!digitalRead(setMaxSelectorPin))
  {
    posLimit[0] = pos[0];
    limitSet[0] = true;
  }
  else
  {
    posLimit[1] = pos[2];
    limitSet[1] = true;
  }
}

void reset()
{
  return;
}

void stepMotor(int *tcnt, int *pos, int *pin, bool dir)
{
  *tcnt = 0;
  if (digitalRead(*pin))
  {
    digitalWrite(*pin, 0);
  }
  else
  {
    digitalWrite(*pin, 1);
    *pos += pow(-1, dir);
  }
}
