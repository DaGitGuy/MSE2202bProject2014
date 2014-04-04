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
int incoming = 0; //KULANI
int val = 0; //KULANI 
int topindex = 0; //KULANI

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
  topindex=0;
 switch (topindex)                    //KULANI
 {                                    // "
  case 0:                             // "
  {                                   // "
    incoming = Serial.available();    // " 
         // "
    val = Serial.parseInt();          // "
                                      // "
    topindex = val;                                 // "
   break;                             // "
  }                                   // "
  
  case 2: //KULANI
 { 
  // Pick-Up
  //Serial.println(analogRead(A4));
  //delay(500);
  
  //if (Serial.available()==1)
  armforward();
  delay(500);
  for(pos = 0; pos < 55; pos += 5)  // goes from 0 degrees to 180 degrees 
  {                                  // in steps of 1 degree 
    gripmotor.write(pos);              // tell servo to go to position in variable 'pos' 
    delay(35);                       // waits 15ms for the servo to reach the position 
  }
  delay(2000);
  armforward();
  delay(2000);
  
 
  for(pos = 55 ; pos>=(0); pos-=5)     // goes from 180 degrees to 0 degrees 
  {                                
    gripmotor.write(pos);              // tell servo to go to position in variable 'pos' 
    delay(35);                       // waits 15ms for the servo to reach the position 
  } 
  
  armreverse();
  delay(500);
  val = 0; //KULANI reset
  Serial.println(0);
  break; //KULANI 
 }
 
 case 1:
 { //drop off
 
 
 
  
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
    Serial.println(0);
  
  break;
 }
 
 case 3:
 {
   int leftbar;
   int rightbar;
   int room=0;
   leftbar= analogRead(A1);
   rightbar= analogRead(A2);
    if (leftbar<= 38 && rightbar<= 38){
          room=1;
    }
    if (leftbar<= 38 && rightbar>= 650){
          room=2;
    }
     if (leftbar>= 650 && rightbar<= 38){
          room=3;
    }
   Serial.println(room);
  break; 
 }
 
 
 
 
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
  while( 1)
  {
    armmotor.writeMicroseconds(1100);
    if (millis()-time >700)
    {
      armmotor.writeMicroseconds(1500);
      break;
    }
  }
}
void armdropoff()
{
  double time= millis();
  while(1)
  {
    armmotor.writeMicroseconds(1100);
    if (millis()-time>2000)
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
