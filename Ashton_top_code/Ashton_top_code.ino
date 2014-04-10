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
 switch (topindex)                    //KULANI
 {                                    // "
  case 0:                             // "
  {                                   // "
    incoming = Serial.available();    // " 
    while(incoming == 0){             // "
      incoming = Serial.available();  // " 
    }                                 // "
    val = Serial.parseInt();          // "
                                      // "
    if(val == 1){                     // "
      topindex = 1;                   // "
    }                                 // "
   break;                             // "
  }                                   // "
  
  case 1: //KULANI
 { 
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
  break; //KULANI 
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
    if (millis()-time >1000)
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
