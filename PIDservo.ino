#include <Servo.h>

double kp = 2;
double ki = 0.05;
double kd = 0.01;

Servo servo1;
int servoPin = 3;
int potPin = A3;

unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double lastError;
double input, output, setPoint;
double ierror, derror;
double val;

void setup(){
  Serial.begin(9600);
  servo1.attach(servoPin);
}

void loop(){
  val = analogRead(potPin);
  setPoint = map(val,0,1023,0,180);
  input = servo1.read();
  output = computePID(input,setPoint);
  delay(50);
  servo1.write(output);
}

int computePID(int input, int setPoint){
  currentTime = millis();
  elapsedTime = (double)(currentTime - previousTime);
  error = setPoint - input;
  delay(100);
  ierror += error*elapsedTime;
  derror = (lastError - error)/elapsedTime;
  double output = kp*error + ki*ierror + kd*derror;
  Serial.println(output);
  lastError = error;
  previousTime = currentTime;
  return output;
}
