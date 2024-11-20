#include <AccelStepper.h>
#include <Arduino.h>
#include <Wire.h>

////////////////
//Pin Constants
////////////////

const int enPin = 2; //enable Pin for all steppers. Daisy chained.
const int step1Pin = 3;
const int dir1Pin = 4;
const int step2Pin = 5;
const int dir2Pin = 6;
const int step3Pin = 7;
const int dir3Pin = 8;
const int startButtonPin = 9;
const int SDA_Pin = 20;
const int SCL_Pin = 21;

///////////////////
//Motor Constants
///////////////////

const int stepsPerRev = 1600;
const int homingSteps = 100;
int T1 = 0; //M1 target
int T2 = 0; //M2 target
int T3 = 0; //M3 target
const int MAX_MOTOR_LIM = 300;
const int MIN_MOTOR_LIM = -300;

AccelStepper M1(AccelStepper::DRIVER, step1Pin, dir1Pin); //create instance of stepper
AccelStepper M2(AccelStepper::DRIVER, step2Pin, dir2Pin); //create instance of stepper
AccelStepper M3(AccelStepper::DRIVER, step3Pin, dir3Pin); //create instance of stepper

/////////////////
// PID constants
/////////////////

float kp = 1;
float kd = 0.025;
float ki = 0.5;
int output = 0; //output from PID algorithm
long prevT = 0; //previous time
float errorPrev = 0; //previous error
const float MAX_INTEGRAL = 100.0; //maximum integral limit
const float MIN_INTEGRAL = -100.0; //minimum integral limit
float integral = 0; //integral term

////////////
//Functions
////////////

int8_t convertToSignedByte(uint8_t unsignedByte) {
  int8_t signedByte;
  
  if (unsignedByte > 127) {
    signedByte = unsignedByte - 256;
  } else {
    signedByte = unsignedByte;
  }
  
  return signedByte;
}

//get position from pot reading
float getPosition(int val){
  return val;
}

int startFlag = 0;

void receiveEvent() {
  int M1_byte = convertToSignedByte(Wire.read());
  int M2_byte = convertToSignedByte(Wire.read());
  int M3_byte = convertToSignedByte(Wire.read());
  if(startFlag){
    T1 = getPosition(M1_byte);
    if(T1>MAX_MOTOR_LIM){
      T1=MAX_MOTOR_LIM;
    } else if(T1<MIN_MOTOR_LIM){
      T1 = MIN_MOTOR_LIM;
    }
    T2 = getPosition(M2_byte);
    if(T2>MAX_MOTOR_LIM){
      T2=MAX_MOTOR_LIM;
    } else if(T2<MIN_MOTOR_LIM){
      T2 = MIN_MOTOR_LIM;
    }
    T3 = getPosition(M3_byte);
    if(T3>MAX_MOTOR_LIM){
      T3=MAX_MOTOR_LIM;
    } else if(T3<MIN_MOTOR_LIM){
      T3 = MIN_MOTOR_LIM;
    }
    M1.moveTo(T1);
    M2.moveTo(T2);
    M3.moveTo(T3);
  }
}


void setup() {
  Serial.begin(9600);

  ////////////////////
  //I2C Configuration
  ////////////////////

  Wire.begin(0x8);
  
  // Call receiveEvent function when data received                
  Wire.onReceive(receiveEvent);

  // Turn off 20k-50k ohm built-in pull up resistors at pins specified
  digitalWrite(SDA_Pin, LOW);
  digitalWrite(SCL_Pin, LOW); 

  //////////////////////
  //Motor Configuration
  //////////////////////

  M1.disableOutputs();
  M2.disableOutputs();
  M3.disableOutputs();

  digitalWrite(enPin,HIGH);

  M1.setMaxSpeed(3000);
  M2.setMaxSpeed(3000);
  M3.setMaxSpeed(3000);
  M1.setCurrentPosition(0); //zero current stepper position
  M2.setCurrentPosition(0);
  M3.setCurrentPosition(0);

  ///////////////
  //STARTUP
  ///////////////
  
  pinMode(startButtonPin, INPUT_PULLUP);

  //All Motors off, wait for home
  while (digitalRead(startButtonPin) == HIGH){
    Serial.println("Move motors to where you want");
  }

  digitalWrite(enPin,LOW);
  M1.enableOutputs(); //enable outputs for motor
  M2.enableOutputs();
  M3.enableOutputs();
  M1.setAcceleration(5000);
  M2.setAcceleration(5000);
  M3.setAcceleration(5000);

  //All motors move out of the way
  M1.moveTo(homingSteps);
  M2.moveTo(homingSteps);
  M3.moveTo(homingSteps);
  
  while(M1.currentPosition() <= homingSteps-10){
    M1.run();
    M2.run();
    M3.run();
  }
  delay(15);

  Serial.println("Motors On and home!");
  M1.setCurrentPosition(0);
  M2.setCurrentPosition(0);
  M3.setCurrentPosition(0);

  //Wait until program start
  delay(5000);
  while(digitalRead(startButtonPin) == HIGH){
    M1.setCurrentPosition(0);
    M2.setCurrentPosition(0);
    M3.setCurrentPosition(0);
  }
  startFlag = 1;
}

//Program
void loop() {
  M1.run();
  M2.run();
  M3.run();
}

// void PID() {

//   //SETPOINT
//   float target = 90.0 * 1023.0 / 270; // *4 This is the target step we wish to achieve converting from degrees to POT ticks.
//   target = constrain(target, 0, 1023); //contrains target to the min/max POT ticks

//   // Find time difference between now and last measurement (for time dependent stuff)
//   long currT = micros();
//   float deltaT = ((float) (currT - prevT))/( 1.0e6 ); //determine change in time
//   prevT = currT; //reset current time


//   // PID calculation
//   int error = target - analogRead(A0);
//   integral = integral + error*deltaT;
//   float derivative = (error - errorPrev)/(deltaT);
//   errorPrev = error;
 
//   // Clamp the integral term
//   if (integral > MAX_INTEGRAL) integral = MAX_INTEGRAL;
//   if (integral < MIN_INTEGRAL) integral = MIN_INTEGRAL;

//   // Control signal
//   float output = kp*error + kd*derivative + ki*integral; //PID output (In POT ticks)

//   float stepperTarget = round(((((output * 270) / 1023) * 3200) / 360)); //converts POT ticks to steps as motor is set to 3200 steps/revolution
//   stepperTarget = (constrain(stepperTarget, -2400, 2400)); //clamps error down too maximum movable steps based on POT
//   Serial.print("stepperTarget "); //prints motor's target steps
//   Serial.println(stepperTarget);
//   stepper.move(stepperTarget);

//   // Moves motors for a certain time before repeating PID calculations
//   long currT2 = millis();
//   while (analogRead(A0) < 1023 && analogRead(A0) > 0 && (millis() - currT2) < 50) { // *5 the period of motor movement can be adjusted
//       stepper.setSpeed(2* stepperTarget); //*6 sets motor speed
//       stepper.runSpeed(); //steps the motor
//  }
// }