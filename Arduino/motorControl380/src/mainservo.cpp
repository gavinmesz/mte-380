// #include <Arduino.h>
// #include <Servo.h>
// #include <Wire.h>

// //PWM Pins
// Servo M1;
// Servo M2;
// Servo M3;
// const int M1_PIN = 2;
// const int M2_PIN = 3;
// const int M3_PIN = 4;

// //I2C pins
// const int SDA_Pin = 20;
// const int SCL_Pin = 21;
// const int ledPin = 13;

// //I2C thing
// uint16_t value;

// //Motor targets
// const int T1 = 0;
// const int T2 = 0;
// const int T3 = 0;

// // PID constants (students can edit these to adjust accuracy)
// float kp = 1; //*1
// float kd = 0.025; //*2
// float ki = 0.5; //*3
// int output = 0; //output from PID algorithm

// long prevT = 0; //previous time
// float errorPrev = 0; //previous error

// // Define maximum and minimum integral limits
// const float MAX_INTEGRAL = 100.0; //maximum integral limit
// const float MIN_INTEGRAL = -100.0; //minimum integral limit

// float integral = 0; //integral term

// int pos = 0;

// //Microsecond speeds
// const int M1_MIN_POS = 10;
// const int M1_MAX_POS = 142;
// const int M2_MIN_POS = 20;
// const int M2_MAX_POS = 169;
// const int M3_MIN_POS = 10;
// const int M3_MAX_POS = 152;
// int target;


//  float getSpeed(int val){
//   // float output;
//   // if(val==0){
//   //   return STOP;
//   // } else if(val>0){
//   //   output =  MIN_POS+(val/100.0)*(MAX_POS-MIN_POS);
//   // } else if(val<0){
//   //   output = MIN_NEG+(val/100.0)*(MIN_NEG-MAX_NEG);
//   // } else{
//   //   return -1;
//   // }
//   return val;
// }

// int8_t convertToSignedByte(uint8_t unsignedByte) {
//   int8_t signedByte;
  
//   if (unsignedByte > 127) {
//     signedByte = unsignedByte - 256;
//   } else {
//     signedByte = unsignedByte;
//   }
  
//   return signedByte;
// }

// void receiveEvent(int howMany) {
//   int M1_byte = convertToSignedByte(Wire.read());
//   int M2_byte = convertToSignedByte(Wire.read());
//   int M3_byte = convertToSignedByte(Wire.read());
  
//   Serial.print("M1: ");
//   Serial.println(round(getSpeed(M1_byte)));
//   Serial.print("M2: ");
//   Serial.println(round(getSpeed(M2_byte)));
//   Serial.print("M3: ");
//   Serial.println(round(getSpeed(M3_byte)));
//   M1.writeMicroseconds(round(getSpeed(M1_byte)));
//   M2.writeMicroseconds(round(getSpeed(M2_byte)));
//   M3.writeMicroseconds(1304);
// }

// void setup() {
//   //Configuring motor behaviour
//   Serial.begin(9600);

//   Wire.begin(0x8);
  
//   // Call receiveEvent function when data received                
//   Wire.onReceive(receiveEvent);

//   // Turn off 20k-50k ohm built-in pull up resistors at pins specified
//   digitalWrite(SDA_Pin, LOW);
//   digitalWrite(SCL_Pin, LOW); 

//   M1.attach(M1_PIN);
//   M2.attach(M2_PIN,0,20000);
//   M3.attach(M3_PIN,0,20000);
// }

// void loop() {
//   M1.write(M2_MAX_POS);
//   delay(1000);
//   M1.write(M2_MIN_POS);
//   delay(1000);
//   // for (pos = M2_MIN_POS; pos <= M1_MAX_POS; pos += 1) { // goes from 0 degrees to 180 degrees
//   //   // in steps of 1 degree
//   //   target = pos;
//   //   M1.write(target);
//   //   // M2.writeMicroseconds(pos);
//   //   // M3.writeMicroseconds(pos);              // tell servo to go to position in variable 'pos'
//   //   delay(15);                       // waits 15ms for the servo to reach the position
//   //   Serial.println(pos);
//   // }
//   // delay(1000);
//   // for (pos = M2_MAX_POS; pos >= M1_MIN_POS; pos -= 1) { // goes from 0 degrees to 180 degrees
//   //   // in steps of 1 degree
//   //   target = pos;
//   //   M1.write(target);
//   //   // M2.writeMicroseconds(pos);
//   //   // M3.writeMicroseconds(pos);              // tell servo to go to position in variable 'pos'
//   //   delay(15);                       // waits 15ms for the servo to reach the position
//   //   Serial.println(pos);
//   // }
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