//kucing abadi

//motor directory
#define CW  0
#define CCW 1
 
//motor control pin
#define motorLDirPin 7
#define motorLPWMPin 9
#define motorRDirPin 5
#define motorRPWMPin 6
#define enablePin_1 8
#define enablePin_2 3
 
//encoder pin
#define encoderPinA_1 2
#define encoderPinB_1 4
#define encoderPinA_2 10
#define encoderPinB_2 11
 
//button pin
#define button1 12
#define button2 13

//encoder var
int encoderPos_1 = 0;
int encoderPos_2 = 0;
 
//PD control
int   targetPos_1  = 100;
int	  targetPos_2  = 150;
int   error_1;
int   error_2;
int   control_1;
int   control_2;
int   velocityR;
int   velocityL;

int peka = 805;

int adc_sensor[6],
	pekax[6],
	sensorMax[6] = {32, 32, 32, 32, 32, 32},
	sensorMin[6] = {685, 685, 685, 685, 685, 685},
	sendat[6],
	kp = 10,
	kd = 1,
	rate_i,
	rate_d,
	lastError = 0,
	x,
	rate,
	sensorBit,
	maxpwm = 250,
	t,
    lane = 0;
 
//external interrupt encoder
void doEncoder_1()
{
  digitalRead(encoderPinB_1)?encoderPos_1--:encoderPos_1++;
}

void doEncoder_2()
{
  digitalRead(encoderPinB_2)?encoderPos_2--:encoderPos_2++;
}

void setup()
{
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  //setup button
  pinMode(button1, INPUT_PULLUP);
  pinMode(button2, INPUT_PULLUP);
  //setup interrupt
    pinMode(encoderPinA_1, INPUT_PULLUP);
    pinMode(encoderPinB_1, INPUT_PULLUP);
    pinMode(encoderPinA_2, INPUT_PULLUP);
    pinMode(encoderPinB_2, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(encoderPinA_1), doEncoder_1, RISING);
    attachInterrupt(digitalPinToInterrupt(encoderPinA_2), doEncoder_2, RISING);
   
    //setup motor driver
    pinMode(motorLDirPin, OUTPUT);
    pinMode(motorRDirPin, OUTPUT);
    pinMode(enablePin_1, OUTPUT);
    pinMode(enablePin_2, OUTPUT);
    digitalWrite(enablePin_1, HIGH);
    digitalWrite(enablePin_2, HIGH);
   
    Serial.begin(9600);
}
 
void loop()
{
  if(digitalRead(button1)==LOW){
    lane=2;
  }
  
  if(digitalRead(button2)==LOW){
    lane=1;
  }
    switch(lane){
      case 1:break;
      case 2:
             follow_line();
             kalibrasistart();
      default:break;
    }
  //Serial.println();
  
}


void kalibrasistart(){
adc_sensor[0] = analogRead(A0);
adc_sensor[1] = analogRead(A1);
adc_sensor[2] = analogRead(A2);
adc_sensor[3] = analogRead(A3);
adc_sensor[4] = analogRead(A4);
adc_sensor[5] = analogRead(A5);
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
adc_sensor[0] = analogRead(A0);
adc_sensor[1] = analogRead(A1);
adc_sensor[2] = analogRead(A2);
adc_sensor[3] = analogRead(A3);
adc_sensor[4] = analogRead(A4);
adc_sensor[5] = analogRead(A5);
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

case 0b100000: error_1 = -4;
  			   error_2 = -4; break;
case 0b010000: error_1 = -3;
  			   error_2 = -3; break;
case 0b110000: error_1 = -2;
  			   error_2 = -2; break;
case 0b011000: error_1 = -1;
  			   error_2 = -1; break;
  //	||
case 0b001000: error_1 = 0;
  			   error_2 = 0; break;
case 0b000100: error_1 = 0;
  			   error_2 = 0; break;
case 0b001100: error_1 = 0;
  			   error_2 = 0; break;
  //	||
case 0b000110: error_1 = 1;
  			   error_2 = 1; break;
case 0b000010: error_1 = 2;
  			   error_2 = 2; break;
case 0b000001: error_1 = 3;
  			   error_2 = 3; break;
case 0b000011: error_1 = 4;
  			   error_2 = 4; break;

default : error_1 = lastError;
          error_2 = lastError; break;
}
}

void follow_line(){
readSensor();
pv();
  
  error_1   = targetPos_1 - encoderPos_1;
  rate_d = error_1 - lastError;
  rate_i = error_1 + lastError;
  lastError = error_1;
  
  control_1 = (kp * error_1) + (kd * rate_d);
  
  velocityL = min(max(control_1, -255), 255);
  
  
  error_2   = targetPos_2 - encoderPos_2;
  rate_d = error_2 - lastError;
  rate_i = error_2 + lastError;
  lastError = error_2;
  //Serial.println(encoderPos_2);
  control_2 = (kp * error_2) + (kd * rate_d);
   
  velocityR = min(max(control_2, -255), 255);
    
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
    Serial.print(encoderPos_1);  
    Serial.print("\t");
    Serial.println(encoderPos_2);
}
