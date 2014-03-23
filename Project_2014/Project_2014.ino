/* Master file for MSE 2202b Project 2014
Letter-delivering mechatronic device
By: Ashton Gobbo
    Chilla Tenga
    Josh Wintraub
    Kulani Zwane
*/

// KULANI's code:

//suggestions for the ultrasonic sensor code:

const int ci_Ultrasonic_Ping = 5;   //input plug
const int ci_Ultrasonic_Data = 6;   //output plug

// set up ultrasonic
pinMode(ci_Ultrasonic_Ping, OUTPUT);
pinMode(ci_Ultrasonic_Data, INPUT);

//variables
unsigned long ul_Echo_Time;

do                                                                              
{                                                                                           
  Serial.println(Ping());
  
}while(Ping() <= 500);

int Ping() //*** B-A Idea 
{
  digitalWrite(ci_Ultrasonic_Ping, HIGH);
  delayMicroseconds(10);
  digitalWrite(ci_Ultrasonic_Ping,LOW);
  ul_Echo_Time = pulseIn (ci_Ultrasonic_Data, HIGH, 10000);
  return ul_Echo_Time;
}

