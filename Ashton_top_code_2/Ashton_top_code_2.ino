#include<Servo.h>
Servo armmotor;
Servo gripmotor;
int pos=0;
int incoming = 0; //KULANI
int val = 0; //KULANI 
int topindex = 0; //KULANI
void setup()
{
  Serial.begin(9600);
  pinMode(9,OUTPUT);
  gripmotor.attach(9);
  gripmotor.write(0);
  pinMode(8,OUTPUT);
  armmotor.attach(8);
  for(int i=2; i<5; i++)
  {
    pinMode(i,OUTPUT);
  }
  
  
  
}
void loop()
{
  
 switch (topindex)                    //KULANI
 {                                    // "
  case 0:                             // "
  {  
   incoming=Serial.available(); 
while(incoming==0)
{
// "
    incoming = Serial.available();
}    // " 
         // "
    val = Serial.parseInt();
    //Serial.print(val);
    // "
                                    // "
    topindex = val; 
    // "
   break;                             // "
  }                                   // "
  
  case 1: //KULANI
 { 
 
  //delay(500);
  
  //if (Serial.available()==1)
  armforward1();
  delay(500);
  for(pos = 0; pos < 60; pos += 5)  // goes from 0 degrees to 180 degrees 
  {                                  // in steps of 1 degree 
    gripmotor.write(pos);              // tell servo to go to position in variable 'pos' 
    delay(35);                       // waits 15ms for the servo to reach the position 
  }
  delay(500);
  
  armforward();
  
  delay(500);
  
 
  for(pos = 60 ; pos>=(0); pos-=5)     // goes from 180 degrees to 0 degrees 
  {                                
    gripmotor.write(pos);              // tell servo to go to position in variable 'pos' 
    delay(35);                       // waits 15ms for the servo to reach the position 
  } 
  
  armreverse();
  
  topindex=0;
  val = 0; //KULANI reset
  Serial.print(1);
  break; //KULANI 
 }
 
 case 2:
 { //drop off
 
 
 
  
     armforward2();
     
     
     delay(500);
    
      for(pos = 0; pos < 60; pos += 5)  // goes from 0 degrees to 180 degrees 
        {                                  // in steps of 1 degree 
        gripmotor.write(pos);              // tell servo to go to position in variable 'pos' 
        delay(35);                       // waits 15ms for the servo to reach the position 
        } 
     
      delay(500);
      
      armreverse2();
      
      delay(500); 
      
      
      for(pos = 30 ; pos>=(-40); pos-=5)     // goes from 180 degrees to 0 degrees 
      {                                
        gripmotor.write(pos);              // tell servo to go to position in variable 'pos' 
        delay(35);                       // waits 15ms for the servo to reach the position 
      }
      delay(400);
      //armreverse1();
      //delay(500);
        
  
        
    
  val=0;
  topindex=0;
  Serial.print(1);
  
  break;
 }
 
 case 3:
 {
   
  
    digitalWrite(2,HIGH);
    digitalWrite(3,HIGH);
    digitalWrite(4,HIGH);
  
   delay(1200);
   int leftbar;
   int rightbar;
   int room=0;
   leftbar= analogRead(A1);
   rightbar= analogRead(A2);
   //+-Serial.print("in case 3");
   
   while(room==0)
   {
    if (leftbar<= 99 && rightbar<= 99){
          room=1;
          digitalWrite(2,HIGH);
          digitalWrite(3,LOW);
          digitalWrite(4,LOW);
          
          Serial.println(room); //This is being send to the other Arduino
          
          break;
    }
    else if (leftbar<= 99 && rightbar>= 300){
          room=2;
          digitalWrite(3,HIGH);
          digitalWrite(2,LOW);
          digitalWrite(4,LOW);
          
          Serial.println(room); //This is being send to the other Arduino
          
          break;
    }
     else if (leftbar>= 300 && rightbar<= 99){
          room=3;
          digitalWrite(4,HIGH);
          digitalWrite(3,LOW);
          digitalWrite(2,LOW);
          
          Serial.println(room); //This is being send to the other Arduino
          
          break;
    }
    else
    {
      //Serial.println("try again");
      //Serial.println(4);
      break;
    }
   }
    //Serial.print("   left   ");
    //Serial.print(leftbar);
    //Serial.print("     right   ");
    //Serial.print(rightbar);
    //Serial.print("     room   ");
    
   //delay(400);
   val=0;
   topindex=0;
   //Serial.println(room);
   Serial.print(1);
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
    if (millis()-time >2200)
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
    armmotor.writeMicroseconds(1200);
    if (millis()-time >1800)
    {
      armmotor.writeMicroseconds(1500);
      break;
    }
  }
}
void armforward1()
{ 
  
  double time=millis();
  while( 1)
  {
    armmotor.writeMicroseconds(1200);
    if (millis()-time >600)
    {
      armmotor.writeMicroseconds(1500);
      break;
    }
  }
}
void armforward2()
{
  double time= millis();
  while(1)
  {
    armmotor.writeMicroseconds(1200);
    if (millis()-time>2200)
    {
      armmotor.writeMicroseconds(1500);
      break;
    }
  }
}
void armreverse2()
{ 
  
  double time=millis();
  while( 1)
  {
    armmotor.writeMicroseconds(1900);
    if (millis()-time >2000)
    {
      armmotor.writeMicroseconds(1500);
      break;
    }
  }
}
void armreverse1()
{ 
  
  double time=millis();
  while( 1)
  {
    armmotor.writeMicroseconds(1700);
    if (millis()-time >800)
    {
      armmotor.writeMicroseconds(1500);
      break;
    }
  }
}
