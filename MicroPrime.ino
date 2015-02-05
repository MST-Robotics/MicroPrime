#include <XBOXRECV.h>
#include <Servo.h>
#include <math.h>

#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif

USB Usb;
XBOXRECV Xbox(&Usb);

//Declare my servos
Servo servoSpeed1; //Front
Servo servoSpeed2; //Back Left - Drivers Side
Servo servoSpeed3; //Back Right - Passengers Side
Servo servoTurn1; //Front
Servo servoTurn2; //Back Left - Drivers Side
Servo servoTurn3; //Back Right - Passengers Side

//Constants used to determine which direction the servos will spin
//Clockwise = Forward = True; Counterclockwise = Backwards = false
bool servo1Forward = true;
bool servo2Forward = true;
bool servo3Forward = true;

//constant servo offsets angle from the front of Robot
const int servo1Offset = 0;
const int servo2Offset = 120;
const int servo3Offset = 240;

//Pins that all servos are attached to
const int servoSpeed1Pin = 22; //Servo in the FRONT of robot
const int servoSpeed2Pin = 24; //Servo on the BACK Drivers Side
const int servoSpeed3Pin = 26; //Servo on the BACK Passenger Side
const int servoTurn1Pin = 23; //Servo in the FRONT of robot
const int servoTurn2Pin = 25; //Servo on the BACK Drivers Side
const int servoTurn3Pin = 27; //Servo on the BACK Passenger Side

//Set all Pins to starting values
int currentSpeedServo1 = 0;
int currentSpeedServo2 = 0;
int currentSpeedServo3 = 0;
int currentPositionServo1 = 90;
int currentPositionServo2 = 90;
int currentPositionServo3 = 90;

//Other Variables
const float pi = 3.1415936; // Define pi for radians conversion
int speedFactor = 0; //value of left Joystick y axis
float xValue = 0; //value of Right Joystick x axis
float yValue = 0; //value of Right Joystick y axis
float desiredHeading; //The angle I want the robot to go 0-360 degrees

//Function to take in the headign and offset and return a servo value
//Description: Takes the desired Heading and scales it to the proper 
//              Servo angles and direction
//Pre: Must have the servo offset, and the joystick heading
//Post:returns a value of 0-180 degrees, and a direction for the wheel
int angleCalc(float desiredHeading, int offset, bool & wheelDirection);

void setup()
{
 
//Setup the serial monitor
  //Serial.begin(115200);
  
  //Assign Pins to all servos
  servoSpeed1.attach(servoSpeed1Pin);
  servoTurn1.attach(servoTurn1Pin);
  servoSpeed2.attach(servoSpeed2Pin);
  servoTurn2.attach(servoTurn2Pin);
  servoSpeed3.attach(servoSpeed3Pin);
  servoTurn3.attach(servoTurn3Pin);
  
 //Set all Servos start position 
  servoSpeed1.writeMicroseconds(1500 + currentSpeedServo1);
  servoTurn1.write(currentPositionServo1);
  servoSpeed2.writeMicroseconds(1500 + currentSpeedServo2);
  servoTurn2.write(currentPositionServo2);
  servoSpeed3.writeMicroseconds(1500 + currentSpeedServo3);
  servoTurn3.write(currentPositionServo3);
  
  //Wait for the usb device, xbox controller, to be connected
  if (Usb.Init() == -1) {
    while (1); //halt
  }
  //wait for the serial monitor to connect
//while (!Serial);
}

void loop()
{
  Usb.Task();
  if (Xbox.Xbox360Connected[0]) 
  { 
    if (Xbox.getAnalogHat(LeftHatX, 0) > 7500 || 
        Xbox.getAnalogHat(LeftHatX, 0) < -7500 || 
        Xbox.getAnalogHat(LeftHatY, 0) > 7500 || 
        Xbox.getAnalogHat(LeftHatY, 0) < -7500 || 
        Xbox.getAnalogHat(RightHatX, 0) > 7500 || 
        Xbox.getAnalogHat(RightHatX, 0) < -7500 || 
        Xbox.getAnalogHat(RightHatY, 0) > 7500 || 
        Xbox.getAnalogHat(RightHatY, 0) < -7500)
    {
      if (Xbox.getAnalogHat(LeftHatY, 0) > 7500 || 
          Xbox.getAnalogHat(LeftHatY, 0) < -7500) 
      { 
        //Set the speed Factor to that of the y axis
        speedFactor = Xbox.getAnalogHat(LeftHatY, 0);
        
        //Scale values to a range of -500 to 500    
        if ( speedFactor > 0)
          speedFactor = map(speedFactor, 7500, 32767, 0, 500);
        else
          speedFactor = map(speedFactor, -32767, -7500, -500, 0);
      }
      else
      {
        //If no signal is recieved then dont move
        currentSpeedServo1 = 0;
        currentSpeedServo2 = 0;
        currentSpeedServo3 = 0;
      }
      
      if(Xbox.getAnalogHat(RightHatX, 0) > 7500 || 
         Xbox.getAnalogHat(RightHatX, 0) < -7500 || 
         Xbox.getAnalogHat(RightHatY, 0) > 7500 || 
         Xbox.getAnalogHat(RightHatY, 0) < -7500)
      {
        //Get values and scale them to accepted range
        if (Xbox.getAnalogHat(RightHatX, 0) > 7500 || 
            Xbox.getAnalogHat(RightHatX, 0) < -7500)
        {
           xValue = Xbox.getAnalogHat(RightHatX, 0);
            
           if ( xValue > 0)
             xValue = map(xValue, 7500, 32767, 0, 180);
           else
             xValue = map(xValue, -32767, -7500, -180, 0);
        }
        else
        {
          xValue = 1;
        }
        if (Xbox.getAnalogHat(RightHatY, 0) > 7500 || 
            Xbox.getAnalogHat(RightHatY, 0) < -7500)
        {     
           yValue = Xbox.getAnalogHat(RightHatY, 0);
              
           if ( yValue > 0)
             yValue = map(yValue, 7500, 32767, 0, 180);
           else
             yValue = map(yValue, -32767, -7500, -180, 0);
        }
        else
        {
          yValue = 1;
        }
    
        //Turn xValue and yValue into the desired angle in radians
        //atan2 returns value of 0 to pi then -pi to 0 in radians
        desiredHeading = atan2(yValue, xValue);
        //Take the values returned by atan2 function
        //Returns values of 0-
        if(desiredHeading < 0)
        {
          desiredHeading += (2*pi);
          desiredHeading = fmod(desiredHeading, (2*pi));
        }
        //convert radians to degrees
        //returns a value of 0-360 degrees
        desiredHeading = round(desiredHeading*180/pi);
 
 //Prints the desired Heading out to the serial monitor       
        /*
        Serial.print("Heading: ");
        Serial.print(desiredHeading);
        Serial.print('\t');
        */
      
      //Map Servo angles for each servo indipendantly
      currentPositionServo1 = 
              angleCalc(desiredHeading, servo1Offset, servo1Forward);
      currentPositionServo2 = 
              angleCalc(desiredHeading, servo2Offset, servo2Forward);
      currentPositionServo3 = 
              angleCalc(desiredHeading, servo3Offset, servo3Forward);
      
      }
      else
      {
        servo1Forward = true;
        servo2Forward = true;
        servo3Forward = true;
        currentPositionServo1 = 90;
        currentPositionServo2 = 90;
        currentPositionServo3 = 90;  
      }
       
      //Takes the value of directional   
      //Set direction and speed for servo 1
      if(servo1Forward)  
        currentSpeedServo1 = speedFactor;
      else
        currentSpeedServo1 = -speedFactor;
      //Set direction and speed for servo 2
      if(servo2Forward)  
        currentSpeedServo2 = speedFactor;
      else
        currentSpeedServo2 = -speedFactor; 
      //Set direction and speed for servo 3
      if(servo3Forward)  
        currentSpeedServo3 = speedFactor;
      else
        currentSpeedServo3 = -speedFactor; 
        
//Prints the desired speed for each servo out to the serial monitor
        /*
        //Servo 1
        Serial.print("Servo 1: Speed = ");
        Serial.print(currentSpeedServo1);
        Serial.print("  ");
        Serial.print("Direction = ");
        Serial.print(currentPositionServo1);
        Serial.print('\t');
        //Servo 2
        Serial.print("Servo 2: Speed = ");
        Serial.print(currentSpeedServo2);
        Serial.print("  ");
        Serial.print("Direction = ");
        Serial.print(currentPositionServo2);
        Serial.print('\t');
        //Servo 3
        Serial.print("Servo 3: Speed = ");
        Serial.print(currentSpeedServo3);
        Serial.print("  ");
        Serial.print("Direction = ");
        Serial.print(currentPositionServo3);
        Serial.print('\t');
        //proper formating
        Serial.Print('\n');
        */
      
      //Give the turning servos their values and then go
      servoTurn1.write(currentPositionServo1);
      servoTurn2.write(currentPositionServo2);
      servoTurn3.write(currentPositionServo3);
       
      //Give the speed servos their values, and then go 
      servoSpeed1.writeMicroseconds(1500 + currentSpeedServo1);
      servoSpeed2.writeMicroseconds(1500 + currentSpeedServo2);
      servoSpeed3.writeMicroseconds(1500 + currentSpeedServo3);
    }
    
    //Exicute if no command is given from the xbox controller
    else
    {
      currentPositionServo1 = 90;
      currentPositionServo2 = 90;
      currentPositionServo3 = 90;
      currentSpeedServo1 = 0;
      currentSpeedServo2 = 0;
      currentSpeedServo3 = 0;
          
      servoTurn1.write(currentPositionServo1);
      servoTurn2.write(currentPositionServo2);
      servoTurn3.write(currentPositionServo3);
         
      servoSpeed1.writeMicroseconds(1500 + currentSpeedServo1);
      servoSpeed2.writeMicroseconds(1500 + currentSpeedServo2);
      servoSpeed3.writeMicroseconds(1500 + currentSpeedServo3);
    }
  }
}


//Function to take in the headign and offset and return a servo value
int angleCalc(float desiredHeading, int offset, bool & wheelDirection)
{
  //Value to return to the program
  int servoAngle;
  
  //Heading after being adjusted based on offset
  int adjustedHeading;
  
  //Scale the desired Heading to the offset
  adjustedHeading = (desiredHeading - offset);
  if(adjustedHeading < 0)
  {
    adjustedHeading += 360;
  }
  
  if(adjustedHeading >= 90 && adjustedHeading < 270)
  {
    servoAngle = adjustedHeading - 90;
    wheelDirection = true;
  }
  else if(adjustedHeading >= 270)
  {
    servoAngle = adjustedHeading - 270;
    wheelDirection = false;
  }
  else
  {
    servoAngle = adjustedHeading + 90;
    wheelDirection = false;
  }
 
  return servoAngle; 
}
