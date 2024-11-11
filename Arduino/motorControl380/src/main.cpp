// #include <AccelStepper.h>
// #include <Arduino.h>

// const int enPin = 2; //enable Pin for all steppers. Daisy chained.
// const int step1Pin = 3;
// const int dir1Pin = 4;
// const int step2Pin = 5;
// const int dir2Pin = 6;
// const int step3Pin = 7;
// const int dir3Pin = 8; 
// const int stepperTarget = -500;

// // PID constants (students can edit these to adjust accuracy)
// float kp = 1; //*1
// float kd = 0.025; //*2
// float ki = 0.5; //*3
// int output = 0; //output from PID algorithm

// AccelStepper motor1(AccelStepper::DRIVER, step1Pin, dir1Pin); //create instance of stepper
// //AccelStepper motor2(AccelStepper::DRIVER, step2Pin, dir2Pin); //create instance of stepper
// //AccelStepper motor3(AccelStepper::DRIVER, step3Pin, dir3Pin); //create instance of stepper

// long prevT = 0; //previous time
// float errorPrev = 0; //previous error

// // Define maximum and minimum integral limits
// const float MAX_INTEGRAL = 100.0; //maximum integral limit
// const float MIN_INTEGRAL = -100.0; //minimum integral limit

// float integral = 0; //integral term

// void setup() {
//   //Configuring motor behaviour
//   Serial.begin(9600);
//   motor1.disableOutputs();
//   //motor2.disableOutputs();
//   //motor3.disableOutputs();
//   motor1.setMaxSpeed(10000);
//   //motor2.setMaxSpeed(10000);
//   //motor3.setMaxSpeed(10000);
//   motor1.setCurrentPosition(0); //zero current stepper position
//   //motor2.setCurrentPosition(0);
//   //motor3.setCurrentPosition(0);
//   motor1.enableOutputs(); //enable outputs for motor
//   //motor2.enableOutputs();
//   //motor3.enableOutputs();
//   //TODO: Acceleration
//   motor1.setAcceleration(10000);
//   //TODO: 
// }

// void loop() {
//   motor1.runToNewPosition(stepperTarget);
//   //motor2.moveTo(stepperTarget);
//   //motor3.moveTo(stepperTarget);
//   Serial.println("moving to 100");
//   Serial.println(motor1.currentPosition());

//   motor1.runToNewPosition(0);
//   //motor2.moveTo(0);
//   //motor3.moveTo(0);
//   Serial.println("moving to 0");
//   Serial.println(motor1.currentPosition());
// }

// // void PID() {

// //   //SETPOINT
// //   float target = 90.0 * 1023.0 / 270; // *4 This is the target step we wish to achieve converting from degrees to POT ticks.
// //   target = constrain(target, 0, 1023); //contrains target to the min/max POT ticks

// //   // Find time difference between now and last measurement (for time dependent stuff)
// //   long currT = micros();
// //   float deltaT = ((float) (currT - prevT))/( 1.0e6 ); //determine change in time
// //   prevT = currT; //reset current time


// //   // PID calculation
// //   int error = target - analogRead(A0);
// //   integral = integral + error*deltaT;
// //   float derivative = (error - errorPrev)/(deltaT);
// //   errorPrev = error;
 
// //   // Clamp the integral term
// //   if (integral > MAX_INTEGRAL) integral = MAX_INTEGRAL;
// //   if (integral < MIN_INTEGRAL) integral = MIN_INTEGRAL;

// //   // Control signal
// //   float output = kp*error + kd*derivative + ki*integral; //PID output (In POT ticks)

// //   float stepperTarget = round(((((output * 270) / 1023) * 3200) / 360)); //converts POT ticks to steps as motor is set to 3200 steps/revolution
// //   stepperTarget = (constrain(stepperTarget, -2400, 2400)); //clamps error down too maximum movable steps based on POT
// //   Serial.print("stepperTarget "); //prints motor's target steps
// //   Serial.println(stepperTarget);
// //   stepper.move(stepperTarget);

// //   // Moves motors for a certain time before repeating PID calculations
// //   long currT2 = millis();
// //   while (analogRead(A0) < 1023 && analogRead(A0) > 0 && (millis() - currT2) < 50) { // *5 the period of motor movement can be adjusted
// //       stepper.setSpeed(2* stepperTarget); //*6 sets motor speed
// //       stepper.runSpeed(); //steps the motor
// //  }
// // }