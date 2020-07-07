/*
SparkFun Inventor’s Kit
Circuit 5C - Autonomous Robot

This robot will drive around on its own and react to obstacles by backing up and turning to a new direction.
This sketch was adapted from one of the activities in the SparkFun Guide to Arduino.
Check out the rest of the book at
https://www.sparkfun.com/products/14326

This sketch was written by SparkFun Electronics, with lots of help from the Arduino community.
This code is completely free for any use.

View circuit diagram and instructions at: https://learn.sparkfun.com/tutorials/sparkfun-inventors-kit-experiment-guide---v40
Download drawings and code at: https://github.com/sparkfun/SIK-Guide-Code
*/
//servo
#include<Servo.h>
const int motorSignalPin = 4;

const int startingAngle = 90;
const int minimumAngle = 60;
const int maximumAngle = 180;
const int rotationSpeed = 1;

Servo motor;

//temperature sensor
#include "dht.h"
#define dht_apin A0 // Analog Pin sensor is connected to Farm water level
#define dht_apin1 A1 // Analog Pin sensor is connected to for Housing
dht DHT;
//#include <Adafruit_BME280.h>

//gyro & accelerometer
#include<Wire.h>
const int MPU=0x68; 
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

//sonar
const int trigPin = 6;
const int echoPin = 5;
// Variables for the duration and the distance
long duration;

// on/off switch
int switchPin = 7;             //switch to turn the robot on and off

//Drivetrain
//the right motor will be controlled by the motor A pins on the motor driver
const int AIN1 = 13;           //control pin 1 on the motor driver for the right motor
const int AIN2 = 12;            //control pin 2 on the motor driver for the right motor
const int PWMA = 11;            //speed control pin on the motor driver for the right motor

//the left motor will be controlled by the motor B pins on the motor driver
const int PWMB = 10;           //speed control pin on the motor driver for the left motor
const int BIN2 = 9;           //control pin 2 on the motor driver for the left motor
const int BIN1 = 8;           //control pin 1 on the motor driver for the left motor

String botDirection;           //the direction that the robot will drive in (this change which direction the two motors spin in)
String distance;               //the distance to travel in each direction

//robot behaviour variables

const int driveTime = 20;      //this is the number of milliseconds that it takes the robot to drive 1 inch
                               //it is set so that if you tell the robot to drive forward 25 units, the robot drives about 25 inches

const int turnTime = 8;        //this is the number of milliseconds that it takes to turn the robot 1 degree
                               //it is set so that if you tell the robot to turn right 90 units, the robot turns about 90 degrees
/********************************************************************************/
void setup()
{
  //servo
  motor.attach(motorSignalPin);

  //Temperature sensor
  delay(500);//Delay to let system boot
  delay(1000);//Wait before accessing Sensor
  
  //gyro & accelerometer
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B); 
  Wire.write(0);    
  Wire.endTransmission(true);
  
  //sonar
  pinMode(trigPin, OUTPUT);       //this pin will send ultrasonic pulses out from the distance sensor
  pinMode(echoPin, INPUT);        //this pin will sense when the pulses reflect back to the distance sensor

  // on/off switch
  pinMode(switchPin, INPUT_PULLUP);   //set this as a pullup to sense whether the switch is flipped


  //Drivetrain
  //set the motor control pins as outputs
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  Serial.begin(9600);                       //begin serial communication with the computer
  Serial.println("To infinity and beyond!");  //test the serial connection

}

/********************************************************************************/
void loop()
{
    if(Serial.available() > 0){
      
      botDirection = Serial.readStringUntil(' ');       //read the characters in the command until you reach the first space
      distance = Serial.readStringUntil(' ');           //read the characters in the command until you reach the second space
      
        Serial.print(botDirection);                       
        Serial.print(" ");                                
        Serial.println(distance.toInt());                 
    
        if(botDirection == "f")                          //if the entered direction is forward                          
        {
          rightMotor(200);                                //drive the right wheel forward
          leftMotor(200);                                 //drive the left wheel forward
          delay(driveTime * distance.toInt());            //drive the motors long enough travel the entered distance
          rightMotor(0);                                  //turn the right motor off
          leftMotor(0);                                   //turn the left motor off
        }
        else if(botDirection == "b")                     //if the entered direction is backward  
        {
          rightMotor(-200);                               //drive the right wheel forward
          leftMotor(-200);                                //drive the left wheel forward
          delay(driveTime * distance.toInt());            //drive the motors long enough travel the entered distance
          rightMotor(0);                                  //turn the right motor off
          leftMotor(0);                                   //turn the left motor off
        }
        else if(botDirection == "r")                      //if the entered direction is right  
        {
          rightMotor(-200);                               //drive the right wheel forward
          leftMotor(255);                                 //drive the left wheel forward
          delay(turnTime * distance.toInt());             //drive the motors long enough turn the entered distance
          rightMotor(0);                                  //turn the right motor off
          leftMotor(0);                                   //turn the left motor off
        }
        else if(botDirection == "l")                    //if the entered direction is left  
        { 
          rightMotor(255);                                //drive the right wheel forward
          leftMotor(-200);                                //drive the left wheel forward
          delay(turnTime * distance.toInt());             //drive the motors long enough turn the entered distance
          rightMotor(0);                                  //turn the right motor off
          leftMotor(0);                                   //turn the left motor off
        }
        else if(botDirection == "o")
        { 
          for(int i=0; i < distance.toInt(); i++){
            getLight();
            getPosition();
            getTemp();
            delay(2500);
          }
        }
    }
    else{
      //radar
      static int motorAngle = startingAngle;
      static int motorRotateAmount = rotationSpeed;
  
      motor.write(motorAngle);
      delay(10);
       SerialOutput(motorAngle, CalculateDistance());
      
      motorAngle += motorRotateAmount;
      if(motorAngle <= minimumAngle || motorAngle >= maximumAngle) {
          motorRotateAmount = -motorRotateAmount;
      }
      
      rightMotor(0);                                  //turn the right motor off
      leftMotor(0);                                   //turn the left motor off 
    }
}
/********************************************************************************/
void rightMotor(int motorSpeed)                       //function for driving the right motor
{
  if (motorSpeed > 0)                                 //if the motor should drive forward (positive speed)
  {
    digitalWrite(AIN1, HIGH);                         //set pin 1 to high
    digitalWrite(AIN2, LOW);                          //set pin 2 to lowf 
  }
  else if (motorSpeed < 0)                            //if the motor should drive backward (negative speed)
  {
    digitalWrite(AIN1, LOW);                          //set pin 1 to low
    digitalWrite(AIN2, HIGH);                         //set pin 2 to high
  }
  else                                                //if the motor should stop
  {
    digitalWrite(AIN1, LOW);                          //set pin 1 to low
    digitalWrite(AIN2, LOW);                          //set pin 2 to low
  }
  analogWrite(PWMA, abs(motorSpeed));                 //now that the motor direction is set, drive it at the entered speed
}

/********************************************************************************/
void leftMotor(int motorSpeed)                        //function for driving the left motor
{
  if (motorSpeed > 0)                                 //if the motor should drive forward (positive speed)
  {
    digitalWrite(BIN1, HIGH);                         //set pin 1 to high
    digitalWrite(BIN2, LOW);                          //set pin 2 to low
  }
  else if (motorSpeed < 0)                            //if the motor should drive backward (negative speed)
  {
    digitalWrite(BIN1, LOW);                          //set pin 1 to low
    digitalWrite(BIN2, HIGH);                         //set pin 2 to high
  }
  else                                                //if the motor should stop
  {
    digitalWrite(BIN1, LOW);                          //set pin 1 to low
    digitalWrite(BIN2, LOW);                          //set pin 2 to low
  }
  analogWrite(PWMB, abs(motorSpeed));                 //now that the motor direction is set, drive it at the entered speed
}

/********************************************************************************/
//RETURNS THE DISTANCE MEASURED BY THE HC-SR04 DISTANCE SENSOR
int CalculateDistance(void)
{
   
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
      long duration = pulseIn(echoPin, HIGH);
    float distance = duration * 0.017F;
    return int(distance);
}

void SerialOutput(const int angle, const int distance)
{
    String angleString = String(angle);
    String distanceString = String(distance);
        Serial.println(angleString + "," + distanceString);
}

void getLight()
{
  unsigned int AnalogValue;
  AnalogValue = analogRead(A3);
  Serial.print(AnalogValue);
  Serial.println(" Light");
}

void getTemp()
{
    DHT.read11(dht_apin);
    Serial.print("Current humidity = ");
    Serial.print(DHT.humidity);
    Serial.print("%  ");
    float celsius = DHT.temperature;
    float F=celsius*1.8+32;
    Serial.print("temperature = ");
    Serial.print(F); 
    Serial.println("F");

}

void getPosition() 
{
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,12,true);  
  AcX=Wire.read()<<8|Wire.read();    
  AcY=Wire.read()<<8|Wire.read();  
  AcZ=Wire.read()<<8|Wire.read();  
  GyX=Wire.read()<<8|Wire.read();  
  GyY=Wire.read()<<8|Wire.read();  
  GyZ=Wire.read()<<8|Wire.read();  
  
  Serial.print("Accelerometer: ");
  Serial.print("X = "); Serial.print(AcX);
  Serial.print(" | Y = "); Serial.print(AcY);
  Serial.print(" | Z = "); Serial.println(AcZ); 
  
  Serial.print("Gyroscope: ");
  Serial.print("X = "); Serial.print(GyX);
  Serial.print(" | Y = "); Serial.print(GyY);
  Serial.print(" | Z = "); Serial.println(GyZ);
  Serial.println(" ");
  //delay(333);
}
