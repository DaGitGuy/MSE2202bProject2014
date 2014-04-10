/*

 MSE 2202 Project 2014: Letter-delivering mechatronic device 
 Language: Arduino
 By: Ashton Gobbo
     Chilla Tenga
     Josh Wintraub
     Kulani Zwane
 
 Alternative code for linetracking.
   Uses analog inputs from ultra sonic sensors as an error
 
*/
 
#include <Servo.h>
#include <EEPROM.h>
#include <uSTimer2.h>
#include <CharliePlexM.h>

Servo servo_LeftMotor;
Servo servo_RightMotor;
Servo servo_ArmMotor;
Servo servo_GripMotor;

// Uncomment keywords to enable debugging output
//#define DEBUG_MODE_DISPLAY
//#define DEBUG_MOTORS
//#define DEBUG_LINE_TRACKERS
//#define DEBUG_ENCODERS
//#define DEBUG_ULTRASONIC
//#define DEBUG_LINE_TRACKER_CALIBRATION
//#define DEBUG_MOTOR_CALIBRATION
//#define DEBUG_ARM

boolean bt_Motors_Enabled = true;

//port pin constants
const int ci_encoder_Pin_A[2] = {
  2,3};
const int ci_Ultrasonic_Ping = 5;   //input plug
const int ci_Ultrasonic_Data = 6;   //output plug
const int ci_Mode_Button = 7;
const int ci_Left_Motor = 8;
const int ci_Right_Motor = 9;
const int ci_Arm_Motor = 10;
const int ci_Grip_Motor = 11;
const int ci_Charlieplex_LED1 = 4;
const int ci_Charlieplex_LED2 = 7;
const int ci_Charlieplex_LED3 = 12;
const int ci_Charlieplex_LED4 = 13;
const int ci_Left_Line_Tracker = A0;
const int ci_Middle_Line_Tracker = A1;
const int ci_Right_Line_Tracker = A2;
const int ci_Motor_Speed_Pot = A3;
const int ci_Light_Sensor = A4;
const int ci_Arm_Length_Pot = A5;

// Charlieplexing LED assignments
const int ci_Left_Line_Tracker_LED = 1;
const int ci_Middle_Line_Tracker_LED = 4;
const int ci_Right_Line_Tracker_LED = 7;
const int ci_Indicator_LED = 3;
const int ci_Heartbeat_LED = 12;

//constants

// EEPROM addresses
const int ci_Right_Motor_Offset_Address_L = 0;
const int ci_Right_Motor_Offset_Address_H = 1;
const int ci_Left_Motor_Offset_Address_L = 2;
const int ci_Left_Motor_Offset_Address_H = 3;
const int ci_Left_Line_Tracker_Dark_Address_L = 4;
const int ci_Left_Line_Tracker_Dark_Address_H = 5;
const int ci_Left_Line_Tracker_Light_Address_L = 6;
const int ci_Left_Line_Tracker_Light_Address_H = 7;
const int ci_Middle_Line_Tracker_Dark_Address_L = 8;
const int ci_Middle_Line_Tracker_Dark_Address_H = 9;
const int ci_Middle_Line_Tracker_Light_Address_L = 10;
const int ci_Middle_Line_Tracker_Light_Address_H = 11;
const int ci_Right_Line_Tracker_Dark_Address_L = 12;
const int ci_Right_Line_Tracker_Dark_Address_H = 13;
const int ci_Right_Line_Tracker_Light_Address_L = 14;
const int ci_Right_Line_Tracker_Light_Address_H = 15;

const int ci_Left_Motor_Stop = 1500;   // 200 for brake mode; 1500 for stop
const int ci_Right_Motor_Stop = 1500;
const int ci_Grip_Motor_Open = 176;    // Experiment to determine appropriate value
const int ci_Grip_Motor_Zero = 90;     //  "
const int ci_Grip_Motor_Closed = 140;  //  "
const int ci_Arm_Pot_Retracted = 1020; //  "
const int ci_Arm_Pot_Extended = 400;   //  "
const int ci_Arm_Pot_Tolerance = 60;   //  "
const int ci_Display_Time = 500;
const int ci_Num_Encoders = 2;
const int ci_Encoder_Steps_Per_Revolution = 90;
const int ci_Line_Tracker_Calibration_Interval = 100;
const int ci_Line_Tracker_Cal_Measures = 20;
const int ci_Motor_Calibration_Time = 10000; //originallly was 5000

//variables
byte b_LowByte;
byte b_HighByte;
unsigned long ul_Echo_Time;
unsigned int ui_Left_Line_Tracker_Data;
unsigned int ui_Middle_Line_Tracker_Data;
unsigned int ui_Right_Line_Tracker_Data;
unsigned int ui_Motors_Speed;
unsigned int ui_Left_Motor_Speed;
unsigned int ui_Right_Motor_Speed;
unsigned int ui_Arm_Length_Data;

volatile unsigned long ul_encoder_Count[ci_Num_Encoders] = {0,0};
unsigned long ul_encoder_Pos[ci_Num_Encoders] = {0,0};
unsigned long ul_old_Encoder_Pos[ci_Num_Encoders] = {0,0};

unsigned long ul_3_Second_timer = 0;
unsigned long ul_Display_Time;
unsigned long ul_Calibration_Time;

unsigned int ui_Right_Motor_Offset = 0;
unsigned int ui_Left_Motor_Offset = 0;

unsigned int ui_Cal_Count;
unsigned int ui_Left_Line_Tracker_Dark;
unsigned int ui_Left_Line_Tracker_Light;
unsigned int ui_Middle_Line_Tracker_Dark;
unsigned int ui_Middle_Line_Tracker_Light;
unsigned int ui_Right_Line_Tracker_Dark;
unsigned int ui_Right_Line_Tracker_Light;
unsigned int ui_Line_Tracker_Tolerance = 10;

unsigned int  ui_Robot_State_Index = 0;
//0123456789ABCDEF
unsigned int  ui_Mode_Indicator[6] = {
  0x00,    //B0000000000000000,  //stop
  0x00FF,  //B0000000011111111,  //Run
  0x0F0F,  //B0000111100001111,  //Calibrate line tracker light level
  0x3333,  //B0011001100110011,  //Calibrate line tracker dark level
  0xAAAA,  //B1010101010101010,  //Calibrate motors
  0xFFFF}; //B1111111111111111};
unsigned int  ui_Mode_Indicator_Index = 0;

//display Bits 0,1,2,3, 4, 5, 6,  7,  8,  9,  10,  11,  12,  13,   14,   15
int  iArray[16] = {
  1,2,4,8,16,32,64,128,256,512,1024,2048,4096,8192,16384,65536};
int  iArrayIndex = 0;

int i_Left_Motor_Calibration = 0;
int i_Right_Motor_Calibration = 0;

boolean bt_Heartbeat = true;
boolean bt_3_S_Time_Up = false;
boolean bt_Do_Once = false;
boolean bt_Cal_Initialized = false;

//NEWLY DECLARED VARIABLES
    int robotindex = 0; //state of robot at a particular task!
    boolean turn = false; // turn indicator. robot turns when the value is set to true;
    boolean nullencoders = false;
  
    //wallvalues
    long owallvalue = 0;
    long nwallvalue = 0;
    
    int x = 0; //detect the door 3100
    long ndistance = 0; //new distance from wall
    long odistance = 0; //old distance from wall
    long deltadistance = 0;//chance in distance from wall
    int ci_encoder_last[2] = {LOW, LOW};
    
    const int runturn = 1;
    int valuecheck = 0;
    int paper = 0; 
    
    int dropmode=0;
    
    
void setup() {
  Serial.begin(9600);
  
  CharliePlexM::setBtn(ci_Charlieplex_LED1,ci_Charlieplex_LED2,ci_Charlieplex_LED3,ci_Charlieplex_LED4,ci_Mode_Button);
  
  CharliePlexM::setEncoders(3,2); //CHILLA
  
  // set up ultrasonic
  pinMode(ci_Ultrasonic_Ping, OUTPUT);
  pinMode(ci_Ultrasonic_Data, INPUT);
  
  // set up drive motors
  pinMode(ci_Left_Motor, OUTPUT);
  servo_LeftMotor.attach(ci_Left_Motor);
  pinMode(ci_Right_Motor, OUTPUT);
  servo_RightMotor.attach(ci_Right_Motor);
  
  // set up arm motors
  pinMode(ci_Arm_Motor, OUTPUT);
  servo_ArmMotor.attach(ci_Arm_Motor);
  pinMode(ci_Grip_Motor, OUTPUT);
  servo_GripMotor.attach(ci_Grip_Motor);
  servo_GripMotor.write(ci_Grip_Motor_Zero);
  
  pinMode(ci_Middle_Line_Tracker, INPUT);
  pinMode(ci_Left_Line_Tracker, INPUT);
  pinMode(ci_Right_Line_Tracker, INPUT);
  
  pinMode(ci_Motor_Speed_Pot, INPUT);
  pinMode(ci_Arm_Length_Pot, INPUT);
  
  // read saved values from EEPROM
  b_LowByte = EEPROM.read(ci_Right_Motor_Offset_Address_L);
  b_HighByte = EEPROM.read(ci_Right_Motor_Offset_Address_H);
  ui_Right_Motor_Offset = word(b_HighByte, b_LowByte);
  b_LowByte = EEPROM.read(ci_Left_Motor_Offset_Address_L);
  b_HighByte = EEPROM.read(ci_Left_Motor_Offset_Address_H);
  ui_Left_Motor_Offset = word(b_HighByte, b_LowByte);
  b_LowByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_L);
  b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
  ui_Left_Line_Tracker_Dark = word(b_HighByte, b_LowByte);
  b_LowByte = EEPROM.read(ci_Left_Line_Tracker_Light_Address_L);
  b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
  ui_Left_Line_Tracker_Light = word(b_HighByte, b_LowByte);
  b_LowByte = EEPROM.read(ci_Middle_Line_Tracker_Dark_Address_L);
  b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
  ui_Middle_Line_Tracker_Dark = word(b_HighByte, b_LowByte); 
  b_LowByte = EEPROM.read(ci_Middle_Line_Tracker_Light_Address_L);
  b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
  ui_Middle_Line_Tracker_Light = word(b_HighByte, b_LowByte);
  b_LowByte = EEPROM.read(ci_Right_Line_Tracker_Dark_Address_L);
  b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
  ui_Right_Line_Tracker_Dark = word(b_HighByte, b_LowByte);
  b_LowByte = EEPROM.read(ci_Right_Line_Tracker_Light_Address_L);
  b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
  ui_Right_Line_Tracker_Light = word(b_HighByte, b_LowByte);
  ui_Line_Tracker_Tolerance = 75;
  
  
  
 
}
void loop()
{
  Ping(); //CHILLA
  
  if((millis() - ul_3_Second_timer) > 3000)
  {
    bt_3_S_Time_Up = true;
  }
  
  // button-based mode selection
  if(CharliePlexM::ui_Btn)
  {
    if(bt_Do_Once == false)
    {
      bt_Do_Once = true;
      ui_Robot_State_Index++;
      ui_Robot_State_Index = ui_Robot_State_Index & 7;
      ul_3_Second_timer = millis();
      bt_3_S_Time_Up = false;
      bt_Cal_Initialized = false;
    }
  }
  else
  {
    bt_Do_Once = LOW;
  }
  
  
  //modes
  switch (robotindex)
  {
  case 0:
    {
      //start calibration
      if(bt_3_S_Time_Up)
      {
        if(!bt_Cal_Initialized){        
          bt_Cal_Initialized = true;
          ul_encoder_Count[0] = 0;
          ul_encoder_Count[1] = 0;
          // read pot to set top motor speed
          ui_Motors_Speed = analogRead(ci_Motor_Speed_Pot);
          ui_Motors_Speed = map(ui_Motors_Speed, 0, 1023, 1650, 1900);     // motor stalls below 1650; too fast above 1900 
          // set motor speeds         
          ui_Left_Motor_Speed = constrain(ui_Motors_Speed, 1650, 1900);
          ui_Right_Motor_Speed = constrain(ui_Motors_Speed, 1650, 1900);
          ul_Calibration_Time = millis();
          //servo_LeftMotor.writeMicroseconds(1900);
          //servo_RightMotor.writeMicroseconds(1873);
        }
          
                     
        //if((ul_encoder_Count[0] < 100)&&(ul_encoder_Count[1] < 100))
        if(analogRead(ci_Light_Sensor) > 40){
        /*
          Serial.print("Encoder 1 =  ");
          Serial.println(CharliePlexM::ul_LeftEncoder_Count);
          Serial.print("Encoder 2 =  ");
          Serial.println(CharliePlexM::ul_RightEncoder_Count);
         */
         
          //if(ul_Echo_Time > 650)
          //{
          
          /*
          servo_LeftMotor.writeMicroseconds(1800 + (680 - ul_Echo_Time));
          servo_RightMotor.writeMicroseconds(1800 - (680 - ul_Echo_Time));
          }
          else{
          */
          
          //servo_LeftMotor.writeMicroseconds(1800 + (470 - ul_Echo_Time) + deltadistance);
          //servo_RightMotor.writeMicroseconds(1800 - (470 - ul_Echo_Time) + deltadistance);
          //}
          //Serial.println("Hey");
          //check sudden drop in distance from wall
         int val;
         int incoming = Serial.available();
         val = Serial.parseInt(); //Reads integers as integer rather than ASCI. Anything else returns 0 
         
         while(incoming == 9){
           incoming = Serial.available();
           servo_LeftMotor.writeMicroseconds(1500);
           servo_RightMotor.writeMicroseconds(1500);
         }
         while(ul_Echo_Time == 0)
          {
            servo_LeftMotor.writeMicroseconds(1800);
            servo_RightMotor.writeMicroseconds(1800);
          } 
          
          if(ul_Echo_Time != 0){
            ndistance = ul_Echo_Time;
          }
          
          
          
          if((millis() - ul_Calibration_Time) > 500){
            Ping(); 
            ul_Calibration_Time = millis();
            
            if(odistance != 0){
              if(ndistance - odistance > 100)
              {
               deltadistance = 680 - 470;
               //deltadistance = ndistance - odistance;                
              }
              
              if(odistance - ndistance > 100)
              {
                deltadistance = 0;
                if(turn == true){
                 servo_LeftMotor.writeMicroseconds(1500);
                 servo_RightMotor.writeMicroseconds(1500);                 
                 delay(1000); 
                 
                 CharliePlexM::ul_LeftEncoder_Count = 0;
                 CharliePlexM::ul_RightEncoder_Count = 0; 
                 robotindex = 1;
                 turn = false;                 
               }
               
               else{
                 servo_LeftMotor.writeMicroseconds(1800);
                 servo_RightMotor.writeMicroseconds(1800);                 
                 delay(1000);                  
               }
              }               
            }       
           
            odistance = ndistance;            
            //Serial.println(ul_Calibration_Time);
            //Serial.println(ndistance);
            //Serial.println(ul_Echo_Time, DEC);
            //Serial.println(deltadistance);
         
            // Serial.println(1);
            
          }
          
          //Serial.println(analogRead(A0));
          
          
          if(analogRead(A0) < 800){
            //Serial.println(analogRead(A0));
            if(dropmode == 1){   
             dropmode = 2;
              
            if(paper == 0){
            servo_LeftMotor.writeMicroseconds(1500);
            servo_RightMotor.writeMicroseconds(1500);
            delay(1000);
            do{
             servo_LeftMotor.writeMicroseconds(1700);
             servo_RightMotor.writeMicroseconds(1700);
              
            }while(analogRead(A0) < 1000);
                       
            
            servo_LeftMotor.writeMicroseconds(1500);
            servo_RightMotor.writeMicroseconds(1500);
            delay(1000);
            Serial.println(3);
            
            robotindex = 2;
            paper++;           
                        
            }           
            }
          }
          
          if(analogRead(A3) < 40){
            if(dropmode == 0){
            servo_LeftMotor.writeMicroseconds(1500);
            servo_RightMotor.writeMicroseconds(1500);
            delay(1000);
            Serial.println(2);
            robotindex = 2;
            dropmode = 1;
            }        
            
          }
          
          servo_LeftMotor.writeMicroseconds(1800 + 0.5*((470 - ul_Echo_Time) + deltadistance));
          servo_RightMotor.writeMicroseconds(1800 - 0.5*((470 - ul_Echo_Time) + deltadistance));
          //Serial.println(analogRead(A0));
          // Serial.println(1800 - (470 - ul_Echo_Time) + deltadistance);
          
         /* if(deltadistance == 0){
          if(ul_Echo_Time > 550){            
            robotindex = 3;
          }
          }*/
        }
      
        else
        {
          //if(paper != 0) paper = 0;     
          dropmode = 0;
          servo_LeftMotor.writeMicroseconds(1500);
          servo_RightMotor.writeMicroseconds(1500);
          delay(1000);
          turn = true;
          Serial.println(1);
          robotindex = 2;
        }
 
    
      //end of calibration
      if((millis() - ul_Display_Time) > ci_Display_Time)
      {
      ul_Display_Time = millis();
#ifdef DEBUG_MODE_DISPLAY  
      Serial.print("Mode: ");
      Serial.println(ui_Mode_Indicator[ui_Mode_Indicator_Index], DEC);
#endif
      bt_Heartbeat = !bt_Heartbeat;
      CharliePlexM::Write(12, bt_Heartbeat);
      digitalWrite(13, bt_Heartbeat);
      Indicator();
      }
    }     
           
     break;
    }
  
  case 1:
  {
    //Robot Turn Case
    //Serial.println("Jesus");
    // Turn right!!
    
    /*Serial.print("Encoder 1 =  ");
    Serial.println(CharliePlexM::ul_LeftEncoder_Count);
    Serial.print("Encoder 2 =  ");
    Serial.println(CharliePlexM::ul_RightEncoder_Count);*/
    
    if(!nullencoders){
      CharliePlexM::ul_LeftEncoder_Count = 0;
      CharliePlexM::ul_RightEncoder_Count = 0; 
      nullencoders = true;
    }
    
    if(CharliePlexM::ul_LeftEncoder_Count < 430){      
       servo_LeftMotor.writeMicroseconds(1900);
       servo_RightMotor.writeMicroseconds(1300);
       
    }
    else
    {
      
       servo_LeftMotor.writeMicroseconds(1500);
       servo_RightMotor.writeMicroseconds(1500);
       delay(1000);
       
       servo_LeftMotor.writeMicroseconds(1900);
       servo_RightMotor.writeMicroseconds(1873);        
       delay(5000);
       
       robotindex = 3;
       
       //CharliePlexM::ul_LeftEncoder_Count = 0;
       //CharliePlexM::ul_RightEncoder_Count = 0;
       // nullencoders = false;          
     
      }   
          
       break;   
     }
     
     case 2:
     {
       
       //chill and wait for a command
       servo_LeftMotor.writeMicroseconds(1500);
       servo_RightMotor.writeMicroseconds(1500);
       
       //For serial communication
       int val = 0; 
       int incoming = 0;
       
       //if(paper != 0) paper = 0;     
      
       incoming = Serial.available(); 
       while(incoming == 0){
         incoming = Serial.available();         
       }
       val = Serial.parseInt(); //Reads integers as integer rather than ASCII.Anything else returns 0    
       //Serial.println(val); 
       robotindex = val-1;
       
       break;
     }
     
     case 3:
     {
       // code to re align with the wall
       //servo_LeftMotor.writeMicroseconds(1900);
       //servo_RightMotor.writeMicroseconds(1873);        
       //delay(2000);
       do{
         
       do{
         
       Ping();
       //set encoders to zero
       CharliePlexM::ul_RightEncoder_Count = 0;
       CharliePlexM::ul_RightEncoder_Count = 0;
       }while(ul_Echo_Time == 0);
       
       //turn left towards the wall for a while
       while(CharliePlexM::ul_RightEncoder_Count < (ul_Echo_Time - 460)){
         servo_LeftMotor.writeMicroseconds(1600);
         servo_RightMotor.writeMicroseconds(1900); 
         //Serial.println( CharliePlexM::ul_RightEncoder_Count);         
       }
       
       
       
       servo_LeftMotor.writeMicroseconds(1500);
       servo_RightMotor.writeMicroseconds(1500);        
       delay(2000);
       
       
       // turn away from the wall until you are 90 degrees
       do{
         
         Ping();
         if(ul_Echo_Time != 0){
            nwallvalue = ul_Echo_Time;
           // Serial.println(nwallvalue);
          }
          
          if(owallvalue == 0){
         owallvalue = nwallvalue;
         }
          
          
         if((millis() - ul_Calibration_Time) > 100){           
           ul_Calibration_Time = millis();          
           servo_LeftMotor.writeMicroseconds(1900);
           servo_RightMotor.writeMicroseconds(1500);
          
          if(owallvalue < (nwallvalue - 10)){
            Serial.println(nwallvalue - owallvalue);
            break;        
          }
          
          else{
            owallvalue = nwallvalue;
          }
          
         }               
                  
       }while(runturn > 0);
       
       
       
       valuecheck = nwallvalue;
       nwallvalue = 0;
       owallvalue = 0;
       
       servo_LeftMotor.writeMicroseconds(1500);
       servo_RightMotor.writeMicroseconds(1500);        
       delay(2000);
       
       }while(valuecheck > 460);
       
       
       robotindex = 0;
       
       break;
       
     }
     
     case 4:
     {
      //start calibration
      if(bt_3_S_Time_Up)
      {
        if(!bt_Cal_Initialized){        
          bt_Cal_Initialized = true;
          ul_encoder_Count[0] = 0;
          ul_encoder_Count[1] = 0;
          // read pot to set top motor speed
          ui_Motors_Speed = analogRead(ci_Motor_Speed_Pot);
          ui_Motors_Speed = map(ui_Motors_Speed, 0, 1023, 1650, 1900);     // motor stalls below 1650; too fast above 1900 
          // set motor speeds         
          ui_Left_Motor_Speed = constrain(ui_Motors_Speed, 1650, 1900);
          ui_Right_Motor_Speed = constrain(ui_Motors_Speed, 1650, 1900);
          ul_Calibration_Time = millis();
          //servo_LeftMotor.writeMicroseconds(1900);
          //servo_RightMotor.writeMicroseconds(1873);
        }
          
                     
        //if((ul_encoder_Count[0] < 100)&&(ul_encoder_Count[1] < 100))
        if(analogRead(ci_Light_Sensor) > 40){
        /*
          Serial.print("Encoder 1 =  ");
          Serial.println(CharliePlexM::ul_LeftEncoder_Count);
          Serial.print("Encoder 2 =  ");
          Serial.println(CharliePlexM::ul_RightEncoder_Count);
         */
         
          //if(ul_Echo_Time > 650)
          //{
          
          /*
          servo_LeftMotor.writeMicroseconds(1800 + (680 - ul_Echo_Time));
          servo_RightMotor.writeMicroseconds(1800 - (680 - ul_Echo_Time));
          }
          else{
          */
          
          //servo_LeftMotor.writeMicroseconds(1800 + (470 - ul_Echo_Time) + deltadistance);
          //servo_RightMotor.writeMicroseconds(1800 - (470 - ul_Echo_Time) + deltadistance);
          //}
          //Serial.println("Hey");
          //check sudden drop in distance from wall
          
          if(ul_Echo_Time != 0){
            ndistance = ul_Echo_Time;
          }
          
          if((millis() - ul_Calibration_Time) > 500){
            Ping(); 
            ul_Calibration_Time = millis();
            
            if(odistance != 0){
              if(ndistance - odistance > 100)
              {
               deltadistance = 680 - 470;
               //deltadistance = ndistance - odistance;                
              }
              
              if(odistance - ndistance > 100)
              {
                deltadistance = 0;
                if(turn == true){
                 servo_LeftMotor.writeMicroseconds(1500);
                 servo_RightMotor.writeMicroseconds(1500);                 
                 delay(1000); 
                 
                 CharliePlexM::ul_LeftEncoder_Count = 0;
                 CharliePlexM::ul_RightEncoder_Count = 0; 
                 robotindex = 5;
                 turn = false;                 
               }
               
               else{
                 servo_LeftMotor.writeMicroseconds(1300);
                 servo_RightMotor.writeMicroseconds(1300);                 
                 delay(1000);                  
               }
              }               
            }       
           
            odistance = ndistance;            
            //Serial.println(ul_Calibration_Time);
            //Serial.println(ndistance);
            //Serial.println(ul_Echo_Time, DEC);
            //Serial.println(deltadistance);
         
            // Serial.println(1);
            
          }
          
          //Serial.println(analogRead(A0));
          
          
          if(analogRead(A0) < 800){
            //Serial.println(analogRead(A0));
            if(dropmode == 1){   
             dropmode = 2;
              
            if(paper == 0){
            servo_LeftMotor.writeMicroseconds(1500);
            servo_RightMotor.writeMicroseconds(1500);
            delay(1000);
            do{
             servo_LeftMotor.writeMicroseconds(1300);
            servo_RightMotor.writeMicroseconds(1300);
              
            }while(analogRead(A0) < 1000);
                       
            
            servo_LeftMotor.writeMicroseconds(1500);
            servo_RightMotor.writeMicroseconds(1500);
            delay(1000);
            Serial.println(3);
            
            robotindex = 2;
            paper++;           
                        
            }           
            }
          }
          
          if(analogRead(A3) < 40){
            if(dropmode == 0){
            servo_LeftMotor.writeMicroseconds(1500);
            servo_RightMotor.writeMicroseconds(1500);
            delay(1000);
            Serial.println(2);
            robotindex = 2;
            dropmode = 1;
            } 
     
            
            
          }
          
          servo_LeftMotor.writeMicroseconds(1200 - 0.5*((470 - ul_Echo_Time) + deltadistance));
          servo_RightMotor.writeMicroseconds(1200 + 0.5*((470 - ul_Echo_Time) + deltadistance));
          //Serial.println(analogRead(A0));
          // Serial.println(1800 - (470 - ul_Echo_Time) + deltadistance);
          
         /* if(deltadistance == 0){
          if(ul_Echo_Time > 550){            
            robotindex = 3;
          }
          }*/
        }
      
        else
        {
          //if(paper != 0) paper = 0;     
          dropmode = 0;
          servo_LeftMotor.writeMicroseconds(1500);
          servo_RightMotor.writeMicroseconds(1500);
          delay(1000);
          turn = true;
          Serial.println(1);
          robotindex = 2;
        }
 
    
      //end of calibration
      if((millis() - ul_Display_Time) > ci_Display_Time)
      {
      ul_Display_Time = millis();
#ifdef DEBUG_MODE_DISPLAY  
      Serial.print("Mode: ");
      Serial.println(ui_Mode_Indicator[ui_Mode_Indicator_Index], DEC);
#endif
      bt_Heartbeat = !bt_Heartbeat;
      CharliePlexM::Write(12, bt_Heartbeat);
      digitalWrite(13, bt_Heartbeat);
      Indicator();
      }
    }     
           
     break;
    }
     case 5:
    {
    //Robot Turn Case while heading back
    //Serial.println("Jesus");
    // Turn left!!
    
    /*Serial.print("Encoder 1 =  ");
    Serial.println(CharliePlexM::ul_LeftEncoder_Count);
    Serial.print("Encoder 2 =  ");
    Serial.println(CharliePlexM::ul_RightEncoder_Count);*/
    
    /*do
    {
      servo_LeftMotor.writeMicroseconds(1100);
       servo_RightMotor.writeMicroseconds(1127);  
       
    }while(frontsensor > 200);*/
    
    if(!nullencoders){
      CharliePlexM::ul_LeftEncoder_Count = 0;
      CharliePlexM::ul_RightEncoder_Count = 0; 
      nullencoders = true;
    }
    
    if(CharliePlexM::ul_LeftEncoder_Count < 430){      
       servo_LeftMotor.writeMicroseconds(1100);
       servo_RightMotor.writeMicroseconds(1700);
       
    }
    else
    {
      
       servo_LeftMotor.writeMicroseconds(1500);
       servo_RightMotor.writeMicroseconds(1500);
       delay(1000);
       
       servo_LeftMotor.writeMicroseconds(1100);
       servo_RightMotor.writeMicroseconds(1127);        
       delay(5000);
       
       //robotindex = 3;
       
       //CharliePlexM::ul_LeftEncoder_Count = 0;
       //CharliePlexM::ul_RightEncoder_Count = 0;
       // nullencoders = false;          
     
      }   
          
       break;   
     }
     
     
    
     
  }
  
}
void Indicator()
{
  //display routine, if true turn on led
  CharliePlexM::Write(ci_Indicator_LED,!(ui_Mode_Indicator[ui_Mode_Indicator_Index] & (iArray[iArrayIndex])));
  iArrayIndex++;
  iArrayIndex = iArrayIndex & 15;
}
// read values from line trackers and update status of line tracker LEDs
void readLineTrackers()
{
  ui_Left_Line_Tracker_Data = analogRead(ci_Left_Line_Tracker);
  ui_Middle_Line_Tracker_Data = analogRead(ci_Middle_Line_Tracker);
  ui_Right_Line_Tracker_Data = analogRead(ci_Right_Line_Tracker);
  if(ui_Left_Line_Tracker_Data > (ui_Left_Line_Tracker_Dark - ui_Line_Tracker_Tolerance))
  {
    CharliePlexM::Write(ci_Left_Line_Tracker_LED, HIGH);
  }
  else
  { 
    CharliePlexM::Write(ci_Left_Line_Tracker_LED, LOW);
  }
  if(ui_Middle_Line_Tracker_Data > (ui_Middle_Line_Tracker_Dark - ui_Line_Tracker_Tolerance))
  {
    CharliePlexM::Write(ci_Middle_Line_Tracker_LED, HIGH);
  }
  else
  { 
    CharliePlexM::Write(ci_Middle_Line_Tracker_LED, LOW);
  }
  if(ui_Right_Line_Tracker_Data > (ui_Right_Line_Tracker_Dark - ui_Line_Tracker_Tolerance))
  {
    CharliePlexM::Write(ci_Right_Line_Tracker_LED, HIGH);
  }
  else
  { 
    CharliePlexM::Write(ci_Right_Line_Tracker_LED, LOW);
  }
#ifdef DEBUG_LINE_TRACKERS
  Serial.print("Trackers: Left = ");
  Serial.print(ui_Left_Line_Tracker_Data,DEC);
  Serial.print(", Middle = ");
  Serial.print(ui_Middle_Line_Tracker_Data,DEC);
  Serial.print(", Right = ");
  Serial.println(ui_Right_Line_Tracker_Data,DEC);
#endif
}
// measure distance to target using ultrasonic sensor  
void Ping()
{
  //Ping Ultrasonic
  //Send the Ultrasonic Range Finder a 10 microsecond pulse per tech spec
  digitalWrite(ci_Ultrasonic_Ping, HIGH);
  delayMicroseconds(10);  //The 10 microsecond pause where the pulse in "high"
  digitalWrite(ci_Ultrasonic_Ping, LOW);
  //use command pulseIn to listen to Ultrasonic_Data pin to record the
  //time that it takes from when the Pin goes HIGH until it goes LOW 
  ul_Echo_Time = pulseIn(ci_Ultrasonic_Data, HIGH, 10000);
  
  // Print Sensor Readings
#ifdef DEBUG_ULTRASONIC
  Serial.print("Time (microseconds): ");
  Serial.print(ul_Echo_Time, DEC);
  Serial.print(", Inches: ");
  Serial.print(ul_Echo_Time/148); //divide time by 148 to get distance in inches
  Serial.print(", cm: ");
  Serial.println(ul_Echo_Time/58); //divide time by 58 to get distance in cm 
#endif
}
