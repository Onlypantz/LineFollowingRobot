#include <AFMotor.h> 
#include <QTRSensors.h>
  
AF_DCMotor MotorLeft(1, MOTOR12_1KHZ ); //create motor #1 using M1 output on Motor Drive Shield, set to 1kHz PWM frequency
AF_DCMotor MotorRight(2, MOTOR12_1KHZ ); //create motor #2 using M2 output on Motor Drive Shield, set to 1kHz PWM frequency
  
#define KP 3.3 
#define KD 5.7
#define ML_MIN_SPEED 150  
#define MR_MIN_SPEED 150  
#define ML_MAX_SPEED 250 
#define MR_MAX_SPEED 250 
#define MIDDLE_SENSOR 4       
#define NUM_SENSORS 5         
#define TIMEOUT 2500         
#define EMITTER_PIN 2    
#define DEBUG 0

#define BLACK_LINE    0
//#define WHITE_LINE  1

QTRSensors qtr;
  
unsigned int sensorValues[NUM_SENSORS];
  
void setup()
{
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5}, NUM_SENSORS);
  qtr.setEmitterPin(2);
  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); 
  for (uint16_t i = 0; i< 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW);
  Serial.begin(9600);
  for (uint8_t i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);
  set_motors(0,0);
}
  
int lastError = 0;
int last_proportional = 0;
int integral = 0;
int position = 0;
  
void loop()
{
  unsigned int sensors[5];
  #if BLACK_LINE
  position = qtr.readLineBlack(sensors); // Read pos if the line is black
  #else
  position = qtr.readLineWhite(sensors); // Read pos if the line is white
  #endif
  int error = position - 2000;
    
  int motorSpeed = KP * error + KD * (error - lastError);
  lastError = error;
    
  int leftMotorSpeed = ML_MIN_SPEED + motorSpeed;
  int rightMotorSpeed = MR_MIN_SPEED - motorSpeed;
    
  // set motor speeds using the two motor speed variables above
  set_motors(leftMotorSpeed, rightMotorSpeed);
}
  
void set_motors(int MotorLeftspeed, int MotorRightspeed)
{
  if (MotorLeftspeed > ML_MAX_SPEED ) MotorLeftspeed = ML_MAX_SPEED;
  if (MotorRightspeed > MR_MAX_SPEED ) MotorRightspeed = MR_MAX_SPEED;
  if (MotorLeftspeed < 0) MotorLeftspeed = 0; 
  if (MotorRightspeed < 0) MotorRightspeed = 0; 
  MotorLeft.setSpeed(MotorLeftspeed); 
  MotorRight.setSpeed(MotorRightspeed);
  MotorLeft.run(FORWARD); 
  MotorRight.run(FORWARD);
}

//calibrate for sometime by sliding the sensors across the line, or you may use auto-calibration instead
void manual_calibration() {
  
  int i;
  for (i = 0; i < 250; i++)
  {
  qtr.calibrate();
  delay(20);
}
  
if (DEBUG) {
  Serial.begin(9600);
  for (int i = 0; i < NUM_SENSORS; i++)
  {
  Serial.print(qtr.calibrationOn.minimum[i]);
  Serial.print(' ');
  }
  Serial.println();

  for (int i = 0; i < NUM_SENSORS; i++)
  {
  Serial.print(qtr.calibrationOn.maximum[i]);
  Serial.print(' ');
  }
  Serial.println();
    
}
}
