/* Master file for MSE 2202b Project 2014
Letter-delivering mechatronic device
By: Ashton Gobbo
    Chilla Tenga
    Josh Wintraub
    Kulani Zwane
*/

//Josh'S Code


#include<Servo.h>
Servo armmotor;
Servo gripmotor;
int pos=0;

void setup()
{
  pinMode(9,OUTPUT);
  gripmotor.attach(9);
  gripmotor.write(0);
  pinMode(7,OUTPUT);
  armmotor.attach(7);
  
  Serial.begin(9600);
}

void loop()
{
  //Serial.println(analogRead(A4));
  //delay(500);
  Serial.println(analogRead(A5));
  delay(500); 
 
  

  
  if (analogRead(A4)<15)
  {
  armforward();
  delay(500);

  for(pos = 0; pos < 60; pos += 5)  // goes from 0 degrees to 180 degrees 
  {                                  // in steps of 1 degree 
    gripmotor.write(pos);              // tell servo to go to position in variable 'pos' 
    delay(35);                       // waits 15ms for the servo to reach the position 
  }
  delay(2000);
  armforward();
  delay(2000);
  
 
  for(pos = 30 ; pos>=(-40); pos-=5)     // goes from 180 degrees to 0 degrees 
  {                                
    gripmotor.write(pos);              // tell servo to go to position in variable 'pos' 
    delay(35);                       // waits 15ms for the servo to reach the position 
  } 

  
  
  armreverse();
  
  delay(500);
  }
  
 if (analogRead(A3)<15)
  {
     armforward();
     armforward();
     delay(500);
    
      for(pos = 0; pos < 60; pos += 5)  // goes from 0 degrees to 180 degrees 
        {                                  // in steps of 1 degree 
        gripmotor.write(pos);              // tell servo to go to position in variable 'pos' 
        delay(35);                       // waits 15ms for the servo to reach the position 
        } 
     
      delay(2000);
      armreverse();
      delay(2000); 
      
      
      for(pos = 30 ; pos>=(-40); pos-=5)     // goes from 180 degrees to 0 degrees 
      {                                
        gripmotor.write(pos);              // tell servo to go to position in variable 'pos' 
        delay(35);                       // waits 15ms for the servo to reach the position 
      }
        
  
        delay(500);
    
  }
  
  
}


void armreverse()
{ 
  double time=millis();
  while(1)
  {
    armmotor.writeMicroseconds(1900);
    if (millis()-time >2000)
    {
      armmotor.writeMicroseconds(1500);
      break;
    }
  }
}
void armforward()
{ 
  double time=millis();
  while(1)
  {
    armmotor.writeMicroseconds(1100);
    if (millis()-time >1000)
    {
      armmotor.writeMicroseconds(1500);
      break;
    }
  }
  
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
  int duration = pulseIn(6, HIGH, 10000);
  //Serial.print(duration/58);
  //Serial.print(" . ");
  return duration;
}
