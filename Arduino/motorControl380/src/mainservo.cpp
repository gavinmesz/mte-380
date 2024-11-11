#include <Arduino.h>
#include <Servo.h>

//PWM Pins
servo M1;
servo M2;
servo M3;
const int M1_PIN = 2;
const int M2_PIN = 3;
const int M3_PIN = 4;

//Motor targets
const int T1 = 0;
const int T2 = 0;
const int T3 = 0;

// PID constants (students can edit these to adjust accuracy)
float kp = 1; //*1
float kd = 0.025; //*2
float ki = 0.5; //*3
int output = 0; //output from PID algorithm

long prevT = 0; //previous time
float errorPrev = 0; //previous error

// Define maximum and minimum integral limits
const float MAX_INTEGRAL = 100.0; //maximum integral limit
const float MIN_INTEGRAL = -100.0; //minimum integral limit

float integral = 0; //integral term

void setup() {
  //Configuring motor behaviour
  Serial.begin(9600);
  M1.attach(M1_PIN,0,20000); // 0 -> 20000 us pulse width SUBJECT TO CHANGE
  M2.attach(M2_PIN,0,20000);
  M3.attach(M3_PIN,0,20000);
}

void loop() {
  
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