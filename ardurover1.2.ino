/*  This is my first public code, and draws heavily on code from Iain Portalupi L298N_Dual_H_Bridge

This program controls an ultrasonic sensor rover using L298N H Bridge Chip and SG04 sensor

Made with the help of http://themakerspace.co.za

          *THE MOTOR SUBROUTINES WORK AS FOLLOWS*

motorA(mode, speed)
% replace A with B to control motor B %

mode is a number 0 -> 3 that determines what the motor 
will do.
0 = coast/disable the H bridge
1 = turn motor clockwise
2 = turn motor counter clockwise
3 = brake motor

speed is a number 0 -> 100 that represents percentage of
motor speed.
0 = off
50 = 50% of full motor speed
100 = 100% of full motor speed

EXAMPLE
Say you need to have motor A turn clockwise at 33% of its
full speed.  The subroutine call would be the following...

motorA(1, 33);

Created by 
Iain Portalupi http://www.youtube.com/iainportalupi
1/2/2014

This code is in the public domain.
*/

#include <NewPing.h>
#define runEvery(t) for (static typeof(t) _lasttime;(typeof(t))((typeof(t))millis() - _lasttime) > (t);_lasttime += (t))


#define TRIGGER_PIN  A0  // Arduino pin tied to trigger pin on ping sensor.
#define ECHO_PIN     A1  // Arduino pin tied to echo pin on ping sensor.
#define MAX_DISTANCE 160 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
#define ENA 6  //enable A on pin 6 (needs to be a pwm pin)
#define ENB 5  //enable B on pin 5 (needs to be a pwm pin)
#define IN1 10  //IN1 on pin 10 conrtols one side of bridge A
#define IN2 11  //IN2 on pin 11 controls other side of A
#define IN3 8  //IN3 on pin 8 conrtols one side of bridge B
#define IN4 9  //IN4 on pin 9 controls other side of B
#define SenA A3 //current sensing from H-Bridge to pin 12 of arduino (hope it doesnt blow it)
#define SenB A2 //current sensing from H-Bridge

NewPing Roboeyes(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // distance sensor called Roboeyes
int SR04;
int cm2crash;

int currentz;

void setup()
{

  //set all of the outputs
 {
  
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  Serial.begin(9600); // Open serial monitor at 9600 baud to see ping results.
  delay(3000);
}
}


void loop() 
{
   Distance2crash();
 
    if (1 < cm2crash && cm2crash <=15) //very close to object - reverse
    {
      motorA(2, 75);  //motor A reverse 75%
      motorB(2, 50);  //motor B reverse 75%
 delay(1000);
    }
   
   else if (15 < cm2crash && cm2crash <30) //moderately close to object - tight turn left
{
 
  //motorA(1, 50);  //have motor A turn clockwise at 80% speed
  //motorB(2, 50);  //have motor B turn anticlockwise at 80% speed
    delay(500);
  
  }
  
  else if (cm2crash >= 30 && cm2crash < 50) //far from object - turn left
  
  {
  motorA(1, 66);  //have motor A turn clockwise at 80% speed
  motorB(2, 50);  //have motor B turn clockwise at 80% speed
  delay(500);
  }
  
 
  
  else // cruise
  {
  
  motorA(1, 66);  //have motor A turn clockwise at 80% speed
  motorB(1, 66);  //have motor B turn clockwise at 80% speed
 
  
  }
  

}

//******************   Current sense from H-bridge   **
/*
void currentsense()

   
{
 runEvery(100)

 {
   currentz = SenA;
 }
  Serial.println(currentz); 
}
*/

//******************   Sonar ultrasonic   *******************
void Distance2crash()
{
  runEvery(100)   //loop for ultrasonic measurement
  {
    SR04 = Roboeyes.ping();
    cm2crash = SR04 / US_ROUNDTRIP_CM;
    if (SR04 == NO_ECHO) // if the sensor did not get a ping        
    {
      cm2crash = MAX_DISTANCE;      //so the distance must be bigger then the max vaulue of the sensor
    }
    // Serial.print("Ping: "); //to check distance on the serial monitor
    // Serial.println(cm2crash); 

  }
}


//******************   Motor A control   *******************
void motorA(int mode, int percent)
{
  
  //change the percentage range of 0 -> 100 into the PWM
  //range of 0 -> 255 using the map function
  int duty = map(percent, 0, 100, 0, 255);
  
  switch(mode)
  {
    case 0:  //disable/coast
      digitalWrite(ENA, LOW);  //set enable low to disable A
      break;
      
    case 1:  //turn clockwise
      //setting IN1 high connects motor lead 1 to +voltage
      digitalWrite(IN1, HIGH);   
      
      //setting IN2 low connects motor lead 2 to ground
      digitalWrite(IN2, LOW);  
      
      //use pwm to control motor speed through enable pin
      analogWrite(ENA, duty);  
      
      break;
      
    case 2:  //turn counter-clockwise
      //setting IN1 low connects motor lead 1 to ground
      digitalWrite(IN1, LOW);   
      
      //setting IN2 high connects motor lead 2 to +voltage
      digitalWrite(IN2, HIGH);  
      
      //use pwm to control motor speed through enable pin
      analogWrite(ENA, duty);  
      
      break;
      
    case 3:  //brake motor
      //setting IN1 low connects motor lead 1 to ground
      digitalWrite(IN1, LOW);   
      
      //setting IN2 high connects motor lead 2 to ground
      digitalWrite(IN2, LOW);  
      
      //use pwm to control motor braking power 
      //through enable pin
      analogWrite(ENA, duty);  
      
      break;
  }
}
//**********************************************************


//******************   Motor B control   *******************
  void motorB(int mode, int percent)
{
  
  //change the percentage range of 0 -> 100 into the PWM
  //range of 0 -> 255 using the map function
  int duty = map(percent, 0, 100, 0, 255);
  
  switch(mode)
  {
    case 0:  //disable/coast
      digitalWrite(ENB, LOW);  //set enable low to disable B
      break;
      
    case 1:  //turn clockwise
      //setting IN3 high connects motor lead 1 to +voltage
      digitalWrite(IN3, HIGH);   
      
      //setting IN4 low connects motor lead 2 to ground
      digitalWrite(IN4, LOW);  
      
      //use pwm to control motor speed through enable pin
      analogWrite(ENB, duty);  
      
      break;
      
    case 2:  //turn counter-clockwise
      //setting IN3 low connects motor lead 1 to ground
      digitalWrite(IN3, LOW);   
      
      //setting IN4 high connects motor lead 2 to +voltage
      digitalWrite(IN4, HIGH);  
      
      //use pwm to control motor speed through enable pin
      analogWrite(ENB, duty);  
      
      break;
      
    case 3:  //brake motor
      //setting IN3 low connects motor lead 1 to ground
      digitalWrite(IN3, LOW);   
      
      //setting IN4 high connects motor lead 2 to ground
      digitalWrite(IN4, LOW);  
      
      //use pwm to control motor braking power 
      //through enable pin
      analogWrite(ENB, duty);  
      
      break;
  }
}



