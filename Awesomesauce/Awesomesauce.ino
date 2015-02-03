#include <XBOXRECV.h>
#include <Servo.h>
#include <math.h>

#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif

USB Usb;
XBOXRECV Xbox(&Usb);

const float pi = 3.14;

//Declare my servos
Servo servoSpeed1;
Servo servoSpeed2; 
Servo servoSpeed3;
Servo servoTurn1; //Front
Servo servoTurn2; //Back Left
Servo servoTurn3; //Back Right

//Constants used to determine Swhich direction the servos will spin
bool servo1Forward = true;
bool servo2Forward = true;
bool servo3Forward = true;

//Pins that all servos are attached to
const int servoSpeed1Pin = 22;
const int servoSpeed2Pin = 24;
const int servoSpeed3Pin = 26;
const int servoTurn1Pin = 23; //Servo in the FRONT of robot
const int servoTurn2Pin = 25; //Servo on the BACK LEFT of robot if robot is facing the same direction as you are
const int servoTurn3Pin = 27; //Servo on the BACK Right of robot if robot is facing the same direction as you are

//Set all Pins to starting values
//syncronized pin settings
int currentPositionSpeedAll = 0;
int currentPositionTurnAll = 90;
//Individual pin settings
int currentPositionSpeed1 = 0;
int currentPositionSpeed2 = 0;
int currentPositionSpeed3 = 0;
int currentPositionTurn1 = 90;
int currentPositionTurn2 = 90;
int currentPositionTurn3 = 90;

//Other Variables
int speedFactor = 0; //How fast will I go?
int turnX = 0; //X value of Joystick
int turnY = 0; //Y value of Joystick
float xValue = 0; //Scaled xValue
float yValue = 0; //Scaled yValue
float desiredHeading; //The angle I want my robot to go

void setup()
{
 
  Serial.begin(115200);
  
  //Assign Pins to all servos
  servoSpeed1.attach(servoSpeed1Pin);
  servoTurn1.attach(servoTurn1Pin);
  servoSpeed2.attach(servoSpeed2Pin);
  servoTurn2.attach(servoTurn2Pin);
  servoSpeed3.attach(servoSpeed3Pin);
  servoTurn3.attach(servoTurn3Pin);
  
 //Set all Servos start position 
  servoSpeed1.writeMicroseconds(1500 + currentPositionSpeedAll);
  servoTurn1.write(currentPositionTurnAll);
  servoSpeed2.writeMicroseconds(1500 + currentPositionSpeedAll);
  servoTurn2.write(currentPositionTurnAll);
  servoSpeed3.writeMicroseconds(1500 + currentPositionSpeedAll);
  servoTurn3.write(currentPositionTurnAll);
  
  if (Usb.Init() == -1) {
    while (1); //halt
  }
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
}

void loop()
{
  Usb.Task();
  if (Xbox.Xbox360Connected[0]) 
  { 
    if (Xbox.getAnalogHat(LeftHatX, 0) > 7500 || Xbox.getAnalogHat(LeftHatX, 0) < -7500 || Xbox.getAnalogHat(LeftHatY, 0) > 7500 || Xbox.getAnalogHat(LeftHatY, 0) < -7500 || Xbox.getAnalogHat(RightHatX, 0) > 7500 || Xbox.getAnalogHat(RightHatX, 0) < -7500 || Xbox.getAnalogHat(RightHatY, 0) > 7500 || Xbox.getAnalogHat(RightHatY, 0) < -7500)
    {
      if (Xbox.getAnalogHat(LeftHatY, 0) > 7500 || Xbox.getAnalogHat(LeftHatY, 0) < -7500) 
      {            
        int speedTemp = Xbox.getAnalogHat(LeftHatY, 0);
         
        //Scale values to a range of -500 to 500    
        if ( speedTemp > 0)
          speedFactor = map(speedTemp, 7500, 32767, 0, 500);
        else
          speedFactor = map(speedTemp, -32767, -7500, -500, 0);
        
        //Set direction for servo 1
        if(servo1Forward)  
          currentPositionSpeed1 = speedFactor;
        else
          currentPositionSpeed1 = -speedFactor;
        
        //Set direction for servo 2
        if(servo2Forward)  
          currentPositionSpeed2 = speedFactor;
        else
          currentPositionSpeed2 = -speedFactor; 
         
        //Set direction for servo 3
        if(servo3Forward)  
          currentPositionSpeed3 = speedFactor;
        else
          currentPositionSpeed3 = -speedFactor; 
      }
      else
      {
        currentPositionSpeed1 = 0;
        currentPositionSpeed2 = 0;
        currentPositionSpeed3 = 0;
      }
      
      if(Xbox.getAnalogHat(RightHatX, 0) > 7500 || Xbox.getAnalogHat(RightHatX, 0) < -7500 || Xbox.getAnalogHat(RightHatY, 0) > 7500 || Xbox.getAnalogHat(RightHatY, 0) < -7500)
      {
        //Get values and scale them to accepted range
        if (Xbox.getAnalogHat(RightHatX, 0) > 7500 || Xbox.getAnalogHat(RightHatX, 0) < -7500)
        {
           turnX = Xbox.getAnalogHat(RightHatX, 0);
            
           if ( turnX > 0)
             xValue = map(turnX, 7500, 32767, 0, 180);
           else
             xValue = map(turnX, -32767, -7500, -180, 0);
        }
        else{xValue=1;}
        if (Xbox.getAnalogHat(RightHatY, 0) > 7500 || Xbox.getAnalogHat(RightHatY, 0) < -7500)
        {     
           turnY = Xbox.getAnalogHat(RightHatY, 0);
              
           if ( turnY > 0)
             yValue = map(turnY, 7500, 32767, 0, 180);
           else
             yValue = map(turnY, -32767, -7500, -180, 0);
        }
        else{yValue=1;}
    
        //Turn xValue and yValue into the desired angle in radians
        desiredHeading = atan2(yValue, xValue);
        if(desiredHeading < 0)
        {
          desiredHeading += (2*pi);
          desiredHeading = fmod(desiredHeading, (2*pi));
        }
        
        //convert radians to degrees
        desiredHeading = round(desiredHeading*180/3.1415926);
        
        Serial.print("Heading: ");
        Serial.print(desiredHeading);
        Serial.print('\n');
        
      
      
        //Servo angles are all wrong at the moment I remaped The desired Heading to 0-360 instead of the origonal -180 to 180      
      
      
        //Turn the Angles just calculated into the proper direction for each servo
        //Servo 1 Front
        if (desiredHeading >= 0 || desiredHeading < 90)
        {
          currentPositionTurn1 = desiredHeading + 90;
          servo1Forward = false;        
        }
        else if(desiredHeading >= 90 || desiredHeading < 180)
        {
          currentPositionTurn1 = desiredHeading - 90;
          servo1Forward = true;
        }
        else if(desiredHeading >= 180 || desiredHeading < 270)
        {
          currentPositionTurn1 = desiredHeading - 90;
          servo1Forward = true;
        }     
        else
        {  
          currentPositionTurn1 = desiredHeading - 270;
          servo1Forward = false; 
        }
        
        //Servo 2 Back Left
        if (desiredHeading >= -135 || desiredHeading <= 45)
        {
          currentPositionTurn2 = desiredHeading + 135;
          servo2Forward = true;        
        }
        else if(desiredHeading > 45)
        {
          currentPositionTurn2 = desiredHeading - 45;
          servo2Forward = false;
        }
        else
        {
          currentPositionTurn2 = desiredHeading + 315;
          servo2Forward = false;
        }   
        
        //Servo 3 Back Right
        if (desiredHeading >= -45 || desiredHeading <= 135)
        {
          currentPositionTurn3 = desiredHeading + 45;
          servo3Forward = true;        
        }
        else if(desiredHeading > 135)
        {
          currentPositionTurn3 = desiredHeading - 135;
          servo3Forward = false;
        }
        else
        {
          currentPositionTurn3 = desiredHeading + 225;
          servo3Forward = false;
        }   
      }
      else
      {
        servo1Forward = true;
        servo2Forward = true;
        servo3Forward = true;
        currentPositionTurn1 = 90;
        currentPositionTurn2 = 90;
        currentPositionTurn3 = 90;  
      }
      
      //Give the turning servos their values and then go
      servoTurn1.write(currentPositionTurn1);
      servoTurn2.write(currentPositionTurn2);
      servoTurn3.write(currentPositionTurn3);
       
      //Give the speed servos their values, and then go 
      servoSpeed1.writeMicroseconds(1500 + currentPositionSpeed1);
      servoSpeed2.writeMicroseconds(1500 + currentPositionSpeed2);
      servoSpeed3.writeMicroseconds(1500 + currentPositionSpeed3);
    }
    
    //Exicute if no command is given from the xbox controller
    else
    {
      currentPositionTurnAll = 90;
      currentPositionSpeedAll = 0;
          
      servoTurn1.write(currentPositionTurnAll);
      servoTurn2.write(currentPositionTurnAll);
      servoTurn3.write(currentPositionTurnAll);
         
      servoSpeed1.writeMicroseconds(1500 + currentPositionSpeedAll);
      servoSpeed2.writeMicroseconds(1500 + currentPositionSpeedAll);
      servoSpeed3.writeMicroseconds(1500 + currentPositionSpeedAll);
    }
  }
}
