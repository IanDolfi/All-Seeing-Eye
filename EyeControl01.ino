//Eye Control Pins
int xPin = 11, yPin = 12 /*, b = 13;*/, eyeControlSelectPin = 32;
int xIn = A6, yIn = A7;

int xMin = 60, xMax = 120, yMin = 60, yMax = 120;


Servo x;
Servo y;

void eyeStart()
{
  x.attach(xPin);
  y.attach(yPin);
  pinMode(eyeControlSelectPin, INPUT_PULLUP);
}

void eyeLoop()
{
  if (eyeControlSelectPin)
  {
    x.write(map(analogRead(xIn), 0, 1024, xMin, xMax));
    y.write(map(analogRead(yIn), 0, 1024, yMin, yMax));
  }
  else
  {
    CVeye();
  }
}

void CVeye()
{
  x.write(90);
  y.write(90);
}
