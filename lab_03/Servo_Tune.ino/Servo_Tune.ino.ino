/*

 ServoTune
 Language: Arduino
 Author: Michael Naish
 Date: 15/01/26
 
 Allows a servo position to be controlled by a pot. The pot value and servo angle can be read
 via the serial monitor. Useful for determining values that correspond to servo limits.
 
 Rev 1 - Initial version
 
 */

#include <Servo.h> 
 
Servo theServo;                     
 
int potPin = A0;                      // analog pin used to connect the potentiometer
int servoPin = 11;                   // pin that servo is connected to
int val;                             // input variable 
 
void setup() 
{ 
   Serial.begin(9600);
   theServo.attach(servoPin);        // attaches the servo to the servo object 
   pinMode(potPin, INPUT);
} 
 
void loop() 
{ 
   val = analogRead(potPin);         // reads the value of the potentiometer (value between 0 and 1023) 
   Serial.print("Pot: ");
   Serial.print(val); 
   val = map(val, 0, 1023, 0, 180);  // scale it to use it with the servo (value between 0 and 180)
   Serial.print(", Servo: ");
   Serial.println(val); 
   theServo.write(val);              // sets the servo position according to the scaled value 
   delay(15);                        // wait for the servo to reach set position 
} 
