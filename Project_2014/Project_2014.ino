/* Master file for MSE 2202b Project 2014
Letter-delivering mechatronic device
By: Ashton Gobbo
    Chilla Tenga
    Josh Wintraub
    Kulani Zwane
*/

//ASHTON'S code


#include <Servo.h>
Servo rightmotor;
Servo leftmotor;
int ultin=5;
int ultout=6;
unsigned long duration;
int val=0;
int a;

void setup(){
  
  Serial.begin(9600);
  pinMode(8,OUTPUT);
  pinMode(9,OUTPUT);
  rightmotor.attach(8);
  leftmotor.attach(9);
  pinMode(5,OUTPUT);
  pinMode(6,INPUT);
  
}

void loop(){
  
  val= analogRead(A4);
  Serial.print(val);
  Serial.print(" . ");
  //rightmotor.writeMicroseconds(2000);
  //leftmotor.writeMicroseconds(2050);
  //Serial.print(Ping()/58);
  //Serial.print(" . ");
  delay (500);
}
double Ping()
{
  //Ping Ultrasonic
  //Send the Ultrasonic Range Finder a 10 microsecond pulse per tech spec
  digitalWrite(5, HIGH);
  delayMicroseconds(10); //The 10 microsecond pause where the pulse in "high"
  digitalWrite(5, LOW);
  //use command pulseIn to listen to Ultrasonic_Data pin to record the
  //time that it takes from when the Pin goes HIGH until it goes LOW 
  duration = pulseIn(6, HIGH, 10000);
  //Serial.print(duration/58);
  //Serial.print(" . ");
  return duration;
}




  
  
  

