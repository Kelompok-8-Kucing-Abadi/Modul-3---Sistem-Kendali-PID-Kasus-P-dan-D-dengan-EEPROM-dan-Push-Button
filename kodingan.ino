#include <EEPROM.h>
//motor directory
#define CW  0
#define CCW 1
 
//motor control pin
#define motorLDirPin 7
#define motorLPWMPin 9
#define motorRDirPin 5
#define motorRPWMPin 6
#define enablePin1 8
#define enablePin2 3
 
//encoder pin
#define encoder1PinA 2
#define encoder1PinB 4
#define encoder2PinA 10
#define encoder2PinB 11
 
//encoder var
int encoderPos = 0;
 
//PD control
int   targetPos   = 100;
int   error;
int   control;
int   velocityR;
int   velocityL;

int peka = 805;

int adc_sensor[6],
	pekax[6],
	sensorMax[6] = {0, 0, 0, 0, 0, 0},
	sensorMin[6] = {1023, 1023, 1023, 1023, 1023, 1023},
	sendat[6],
	robotSpeedRight,
	robotSpeedLeft,
	robotSpeed = 170,
	kp = 25,
	kd = 7,
    ki = 0,
	rate_i,
	rate_d,
	lastError = 0,
	x,
	rate,
	sensorBit,
	maxpwm = 250,
	t;
 
//external interrupt encoder
void doEncoderA()
{
  digitalRead(encoder1PinB)?encoderPos--:encoderPos++;
  digitalRead(encoder2PinB)?encoderPos--:encoderPos++;
}
 
void setup()
{
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  //setup interrupt
    pinMode(encoder1PinA, INPUT_PULLUP);
    pinMode(encoder1PinB, INPUT_PULLUP);
    pinMode(encoder2PinA, INPUT_PULLUP);
    pinMode(encoder2PinB, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(encoder1PinA), doEncoderA,RISING);
    attachInterrupt(digitalPinToInterrupt(encoder2PinA), doEncoderA,RISING);
   
    //setup motor driver
    pinMode(motorLDirPin, OUTPUT);
    pinMode(motorRDirPin, OUTPUT);
    pinMode(enablePin1, OUTPUT);
    pinMode(enablePin2, OUTPUT);
    digitalWrite(enablePin1, HIGH);
    digitalWrite(enablePin2, HIGH);
   
    Serial.begin(9600);
}
 
void loop()
{
  follow_line();
  
  kalibrasistart();
       
}


void kalibrasistart(){
adc_sensor[0] = analogRead(A0);
adc_sensor[1] = analogRead(A1);
adc_sensor[2] = analogRead(A2);
adc_sensor[3] = analogRead(A3);
adc_sensor[4] = analogRead(A4);
adc_sensor[5] = analogRead(A5);
delay(10);
for (x = 5; x >= 0; x--){
if(adc_sensor[x] > sensorMax[x]){
sensorMax[x] = adc_sensor[x];
}
if(adc_sensor[x] < sensorMin[x]){
sensorMin[x] = adc_sensor[x];
}
pekax[x] = (sensorMax[x] + sensorMin[x]) / 2;
}
}

void readSensor(){
adc_sensor[0] = EEPROM.read(A0);
adc_sensor[1] = EEPROM.read(A1);
adc_sensor[2] = EEPROM.read(A2);
adc_sensor[3] = EEPROM.read(A3);
adc_sensor[4] = EEPROM.read(A4);
adc_sensor[5] = EEPROM.read(A5);
delay(10);
for (x = 5; x >= 0; x--){
if(adc_sensor[x] > pekax[x]){
sendat[x] = 1;
}
else {
sendat[x] = 0;
}
}
sensorBit = 0;
for (x = 5; x >= 0; x--){
sensorBit += sendat[x] * (1 << x);
}
}

void pv(){
switch (sensorBit){

case 0b100000: error = -3; break;
case 0b010000: error = -2; break;
case 0b110000: error = -1; break;
case 0b001000: error = 0; break;
case 0b000100: error = 0; break;
case 0b001100: error = 0; break;
case 0b000010: error = 1; break;
case 0b000001: error = 2; break;
case 0b000011: error = 3; break;

default : error = lastError; break;
}
}

void follow_line(){
readSensor();
pv();
  rate_d = error - lastError;
  rate_i = error + lastError;
  lastError = error;

  control = (kp * error) + (ki * rate_i) + (kd * rate_d);
  error   = targetPos - encoderPos;
   
    velocityL = min(max(control, -255), 255);
    velocityR = min(max(control, -255), 255);
    
    if(velocityL >= 0)
    {
        digitalWrite(motorLDirPin, CW);
        analogWrite(motorLPWMPin, velocityL); 
    }
    else
    {
        digitalWrite(motorLDirPin, CCW);
        analogWrite(motorLPWMPin, 255+velocityL);
    }
    if(velocityR >= 0)
    {
        digitalWrite(motorRDirPin, CW);
        analogWrite(motorRPWMPin, velocityR); 
    }
    else
    {
        digitalWrite(motorRDirPin, CCW);
        analogWrite(motorRPWMPin, 255+velocityR);
    }
    Serial.println(encoderPos);  
}
