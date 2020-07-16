/*
 This sketch is a test of the basic functions for the drivetrain. 
 No sensors are utilized in this sketch. 
 4 1.2A DC motors are connected to the OSEPP Motor and Servo Shield V1 and an Arduino microcontroller.
*/
#include <AFMotor.h>

AF_DCMotor motor1(1, MOTOR12_64KHZ);
AF_DCMotor motor2(2, MOTOR12_64KHZ);
AF_DCMotor motor3(3, MOTOR12_1KHZ);
AF_DCMotor motor4(4, MOTOR12_1KHZ);

void setup() {
  motor1.setSpeed(200);
  motor2.setSpeed(200);
  motor3.setSpeed(200);
  motor4.setSpeed(200);

}

void loop() {
   //test to see if rover will drive forward
   motor1.run(FORWARD);                              
   motor2.run(FORWARD);                           
   motor3.run(FORWARD);
   motor4.run(FORWARD);

   delay(2000);

   motor1.run(RELEASE);
   motor2.run(RELEASE);
   motor3.run(RELEASE);
   motor4.run(RELEASE);

}
