#include <STM32FreeRTOS.h>
#include <SCServo.h>
#include <Wire.h>
#include "RTClib.h"

////// library Variable
RTC_DS1307 rtc;  // RTC
DateTime startTime; // RTC variable
SMS_STS st; // Servo

////// constant
const int starter_flag = 83;
const int screen_change_flag = 101;

/////////////////////////////////////////////////////////      PIN DECLARATION   /////////////////////////////

#define S_RXD PA9 // RX1 (Serial1)
#define S_TXD PA8 // TX1 (Serial1)
#define actuator_UP PB12
#define actuator_DOWN PB13

TaskHandle_t handleTaskSILAR  = NULL;
TaskHandle_t handleTaskMOTOR  = NULL;
bool isTaskSILARPaused = false;
bool isTaskSILARCancel = false;


// Serial1 interfaces
HardwareSerial &dwinSerial = Serial2;
HardwareSerial Serial1(USART1);


///// MOTOR Pin  ///////

#define MOTOR1_PWM PB4
#define MOTOR2_PWM PB5
#define MOTOR3_PWM PB6
#define MOTOR4_PWM PB7
#define MOTOR5_PWM PB8
#define MOTOR6_PWM PB9

///////////////////////////// pin for FLAG to MCU1   ///////////////////

#define PAUSE_FLAG PA7
#define CANCEL_FLAG PA6
#define HEATER_START_FLAG PB0
#define SILAR_OVER_FLAG PA5
#define reseting_initi PB1

unsigned int decimalValue,addergd,start,beaker,speed,cycle,position,current_hour,current_min,current_sec,currentBEAKER,Dwin_beaker,stirrer,heater_flag;
static bool speed_flag = false,pause_state = false ,cancel_state = false,dwinRestarted = false,heaterStart_flag= false;
uint8_t Buffer[9];
unsigned long totalTime;


unsigned int myArray[8][6] = {
  {1, 1, 0, 0, 0, 0},  // 0. Beaker Count, 1. Cycle Count, 2. Heater, 3. Slide Depth, 4. Speed, 5. Start
  {0, 0, 0, 0, 0, 0},  // Time Row (Seconds)
  {0, 0, 0, 0, 0, 0},  // Time Row (Minutes)
  {0, 0, 0, 0, 0, 0},  // Time Row (Hours)
  {0, 0, 0, 0, 0, 0},   /// 0. S!, 1. S2 , 2. S3 , 3. S4 , 4. S5 , 5. S6
  {0, 0, 5, 0, 0, 0},  //// 0.  current_beaker  1. dwin_beaker 2. pause 3. cancel
  {0, 0, 0, 0, 0, 0}, /////  Temperature SETPOINT 
  {0, 0, 0, 0, 0, 0}, ///    Current Temperature 
};
unsigned long previousMillis = 0;



////////////////////////////////////////   Restart UART /////////////////////////////

void flushSerial() {
    while (dwinSerial.available()) {
      dwinSerial.read();
    }
}


//////////////////////////////////////   CANCEL Function ///////////////////////////////////////

void cancel_fun() {
  moveActuator(10,100); 
  st.WritePosEx(1, 1593, 1000, 20);  /// servo to 120 degree
  digitalWrite(reseting_initi, HIGH);
}

//////////////////////////////////////////////////////// IIC Data Reciving //////////////////////////////////////////

void MCUEvent(int AddBytes,int dataBYTE) {
  byte MCU2_buffer[5] = {0x5A, 0xA5, 0x00, 0x00, 0x00}; ////   header ; header ; address ; higher byte ; lower byte
  MCU2_buffer[2] = lowByte(AddBytes);  // 11   22  33  44  
  MCU2_buffer[3] = highByte(dataBYTE);
  MCU2_buffer[4] = lowByte(dataBYTE);
  dwinSerial.write(MCU2_buffer, sizeof(MCU2_buffer)); 
  delay(100 );
}

/////////////////////////////////////////////////////////////// Actuator Function  ////////////////////////////////

void moveActuator(int moveUp, int distance) {
  //.println("inside Actuator Function");
  // Total time to move full range 2500 ms
  int moveit = distance*285;  /// 1 mm = 0.285 sec
  if (moveUp == 10) {  // Move actuator UP
    digitalWrite(actuator_UP, HIGH);
    digitalWrite(actuator_DOWN, LOW);
    delay(moveit );  // Move for calculated time
  } 
  else if (moveUp == 20) {  // Move actuator DOWN
    digitalWrite(actuator_UP, LOW);
    digitalWrite(actuator_DOWN, HIGH);
    delay(moveit );  // Move for calculated time
  }

  digitalWrite(actuator_UP, LOW);
  digitalWrite(actuator_DOWN, LOW);
}

////////////////////////////////////////////////////////////////  Time Calculator ///////////////////////////

void calculateTotalTime() 
{
  beaker = myArray[0][0];
  cycle =  myArray[0][1];

  unsigned long totalTimePerCycle = 0; // Start with any base time
  for (int i = 0; i < beaker; i++) {
    totalTimePerCycle += myArray[3][i];          // Seconds
    totalTimePerCycle += myArray[2][i] * 60UL;   // Minutes
    totalTimePerCycle += myArray[1][i] * 3600UL; // Hours
  }
  
  totalTime = totalTimePerCycle * cycle;  // Multiply by number of cycles
  delay(50 );  
  // Convert to hours and minutes
  unsigned int hours = totalTime / 3600;
  unsigned int minutes = (totalTime % 3600) / 60;

  // Send to display
  MCUEvent(33,hours);///  11. Current_beaker 22. current_cycle 33. current_hour 44. current_min ////   header ; header ; address ; higher byte ; lower byte
  delay(100 ); // Wait 0.1 second before checking again 
  MCUEvent(44,minutes);
}

//////////////////////////////////////////////////////////// remaining Time Function  //////////////////////

void remainTime(int pos) {
  unsigned long remainTIME = 0;
  remainTIME += myArray[3][pos];                      // Seconds
  remainTIME += myArray[2][pos] * 60UL;               // Minutes
  remainTIME += myArray[1][pos] * 3600UL;             // Hours.
  
  totalTime = totalTime - remainTIME;
  //totalSeconds = myArray[5][2];

  unsigned int hours = totalTime / 3600;
  unsigned int minutes = (totalTime % 3600) / 60;

  MCUEvent(33,hours);///  0. Current_beaker 1. current_cycle 2. current_hour 3. current_min ////   header ; header ; address ; higher byte ; lower byte
  delay(100 ); // Wait 0.1 second before checking again 
  MCUEvent(44,minutes);
}
/////////////////////////////////////////////////////////////// Time Period RTC Function  ////////////////////////////////

void TimePeriod(int period_hour,int period_min,int period_sec) {
  DateTime startTime = rtc.now();  // Get the current time
  DateTime endTime = startTime + TimeSpan(0, period_hour, period_min, period_sec); // Calculate the end time

  while (rtc.now() < endTime) {
   TimeSpan remainingTime = endTime - rtc.now(); 
   delay(50 ); // Wait 1 second before checking again 
  }
}

////////////////////////////////////////////////////////////  Motor PIN /////////////////////////////////////////

uint8_t getMotorPin(int motorNum) {
  switch (motorNum) {
    case 0: return MOTOR1_PWM;
    case 1: return MOTOR2_PWM;
    case 2: return MOTOR3_PWM;
    case 3: return MOTOR4_PWM;
    case 4: return MOTOR5_PWM;
    case 5: return MOTOR6_PWM;
    default: return 255;  // Invalid motor
  }
}

////////////////////////////////////////////////////////   infineLOOP  

void infineLOOP(){
  bool infinite = true;
  while(infinite){
   int pause_value = myArray[5][2]; 
   pause_state = digitalRead(PAUSE_FLAG); 
   delay(50);
   if (!pause_state) {
      infinite = false;
    }
    delay(50);
  }
}

//////////////////////////////////////////////////////////////////    SETUP    ///////////////////////////////////


void setup() {
  //Serial1.begin(57600);
  Serial1.begin(1000000);  // Servo communication using Serial1
  st.pSerial = &Serial1;   // Assign Serial1 to the servo library
  
  dwinSerial.begin(115200);
  heater_flag = myArray[0][2];
  pinMode(actuator_UP,OUTPUT);
  pinMode(actuator_DOWN,OUTPUT); 

  pinMode(MOTOR1_PWM,OUTPUT);
  pinMode(MOTOR2_PWM,OUTPUT);
  pinMode(MOTOR3_PWM,OUTPUT);
  pinMode(MOTOR4_PWM,OUTPUT);
  pinMode(MOTOR5_PWM,OUTPUT);
  pinMode(MOTOR6_PWM,OUTPUT);

  pinMode(PAUSE_FLAG, INPUT_PULLDOWN);
  pinMode(CANCEL_FLAG, INPUT_PULLDOWN);
  pinMode(HEATER_START_FLAG, INPUT_PULLDOWN);
  pinMode(SILAR_OVER_FLAG, OUTPUT);
  pinMode(reseting_initi, OUTPUT);

  digitalWrite(SILAR_OVER_FLAG, LOW);
  digitalWrite(reseting_initi, LOW);

 /////////////////////////////////////////////  IIC for RTC   ///////////////

  Wire.setSCL(PB10);  // Set custom SCL pin
  Wire.setSDA(PB3);  // Set custom SDA pin
  Wire.begin();
  if (!rtc.begin()) {
    while (1);
  }

  if (!rtc.isrunning()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));  // Set RTC if not running
  }

  startTime = rtc.now();
 
} 


////////////////////////////////////////////////////////////  main loop  ///////////////////////
void loop() {
  start = myArray[0][5];
  heater_flag = myArray[0][2];
  TaskFetchData();
  if(start == starter_flag){
    TaskSILAR();
  }

}

/////////////////////////////////////////       SILAR  FUNCTION     //////////////////////////////////


void TaskSILAR() {
  start = myArray[0][5];
  heater_flag = myArray[0][2];
  beaker = myArray[0][0]; 
  cycle =  myArray[0][1] + 1;
  speed = myArray[0][4];
  int previousBeaker = 0;

  ///////////////////////////////////////////  heater_state  ///////////////////////////////////

  if(start == starter_flag && heater_flag == 1){
    heaterStart_flag = digitalRead(HEATER_START_FLAG);
  }

  if(start == starter_flag && heater_flag == 0){
    heaterStart_flag = true;
  }
  delay(50 );

  ////////////////////////////////////////////  MOTOR   //////////////////////////////////

  if (speed > 1) {
    speed = map(speed, 0, 10, 0, 255);
    speed_flag = true;
  }  
  if (start == starter_flag && speed_flag ) {
    for (int i = 0; i < beaker; i++) {  
      analogWrite(getMotorPin(i), speed);
    } 
    speed_flag = false;
  }
  delay(50 );

  //////////////////////////////////////////  START FUNCTION ////////////////////////////

  if (start == starter_flag && heaterStart_flag) {
    delay(50);
    calculateTotalTime();  ///// calculateTotalTime(int pos,int multi,int initi)
    moveActuator(10,100);
    moveActuator(20,20); /// 8 cm from ground  and 5 mm from beaker  20,20
    delay(50);
    for (int h = 1; h < cycle; h++) { 
      for (int b = 0; b < beaker; b++) {  
        if(pause_state){
          infineLOOP();
        }  
        if (start == starter_flag && speed_flag ) {
          for (int i = 0; i < beaker; i++) {
            stirrer = myArray[4][i];
            if (stirrer == 1) {
              analogWrite(getMotorPin(i), speed);
              delay(50);
              } 
            else{  
              analogWrite(getMotorPin(b), 0);
              digitalWrite(getMotorPin(i), LOW);
              delay(50);
            }
          }  
        }
        analogWrite(getMotorPin(b), 0);
        digitalWrite(getMotorPin(b), LOW);
        delay(50);

        ////////////////////////////////////////////////////////  SILAR  ////////////////


        MCUEvent(11,h);///  0. Current_beaker 1. current_cycle 2. current_hour 3. current_min ////   header ; header ; address ; higher byte ; lower byte
        delay(100 ); // Wait 0.1 second before checking again 

        //delay(500 );
        Dwin_beaker = b + 1 ;
        myArray[5][0] = b;
        MCUEvent(22,Dwin_beaker);///  0. Current_beaker 1. current_cycle 2. current_hour 3. current_min ////   header ; header ; address ; higher byte ; lower byte

        // Servo rotate
        position = b * 683 + 227;
        st.WritePosEx(1, position, 1000, 20);
        delay(50 );  // Wait for the servo to move

        // Move actuator down
        moveActuator(20,35);  // 20,35
        delay(50 );
    
        // Wait for specified time current_hour,current_min,current_sec
        current_hour = myArray[1][b];          
        current_min = myArray[2][b];          
        current_sec = myArray[3][b];         
        TimePeriod(current_hour,current_min,current_sec);
        delay(50 );
        remainTime(b);
        delay(50);
        delay(50);
        moveActuator(10,35);   
        if ( myArray[4][b] == 1) {
          analogWrite(getMotorPin(b), speed);
          delay(50);
        }   
        delay(1000);
        //TimePeriod(0,0,5);
        delay(100 );

        //////////////////////////////////////////////////////////////   PAUSED BUTTON STATE //////////////////////

        if (start == starter_flag) {
          pause_state = digitalRead(PAUSE_FLAG);
          myArray[5][2] = pause_state;
        }
        delay(50);
        
        ///////////////////////////////////////////  cancel_state  ///////////////////////////////////

        if(start == starter_flag){
          cancel_state = digitalRead(CANCEL_FLAG);
        }
        if(cancel_state){
          cancel_fun();
        }
        delay(50 );
      }
    }    
    // changeScreen(0x00);
    digitalWrite(SILAR_OVER_FLAG,HIGH);
    cancel_fun();
    myArray[0][5] = 66;
  } 
}

////////////////////////////////////////////   Fetching Data FUNCTION   ////////////////////////////////////


void TaskFetchData(){
  static bool prevPauseState = false;
  uint16_t combinedValue;
  start = myArray[0][5];
  heater_flag = myArray[0][2];
  beaker = myArray[0][0]; 
  
  ////////////////////////////////////////////   screen change state ////////////////////////

  if (start == screen_change_flag && !dwinRestarted) {
    flushSerial(); 
    delay(200 );
    dwinSerial.end();
    delay(100 );
    dwinSerial.begin(115200);
    flushSerial(); 
    myArray[0][5] = 50;
    dwinRestarted = true;  // Prevent repeated restarts
  }

  //////////////////////////////////////////////  back home page /////////////////////////
  
  delay(50 );

  /////////////////////////////////////////////////   Fetching Data ////////////////////////////////
  if(start != starter_flag){
    if( dwinSerial.available() >= 9) {
      for (int i = 0; i < 9; i++) {
        Buffer[i] = dwinSerial.read();
      }
      

      if (Buffer[0] == 0x5A && Buffer[1] == 0xA5) {  
            decimalValue = Buffer[8];
            switch (Buffer[4]) {
              case byte(0x10):myArray[0][0] = decimalValue; break;
              case byte(0x15):myArray[0][2] = decimalValue; break;
              case byte(0x11):combinedValue = (Buffer[7] << 8) | Buffer[8];myArray[0][1] = combinedValue; break;
              case byte(0x12):myArray[0][3] = decimalValue; break;
              case byte(0x13):myArray[0][4] = decimalValue; break;
              case byte(0x40):myArray[0][5] = decimalValue; break;
              case byte(0x16):myArray[1][0] = decimalValue; break;
              case byte(0x17):myArray[2][0] = decimalValue; break;
              case byte(0x18):myArray[3][0] = decimalValue; break;
              case byte(0x1B):myArray[1][1] = decimalValue; break;
              case byte(0x1C):myArray[2][1] = decimalValue; break;
              case byte(0x1D):myArray[3][1] = decimalValue; break;
              case byte(0x20):myArray[1][2] = decimalValue; break;
              case byte(0x21):myArray[2][2] = decimalValue; break;
              case byte(0x22):myArray[3][2] = decimalValue; break;
              case byte(0x25):myArray[1][3] = decimalValue; break;
              case byte(0x26):myArray[2][3] = decimalValue; break;
              case byte(0x27):myArray[3][3] = decimalValue; break;
              case byte(0x2A):myArray[1][4] = decimalValue; break;
              case byte(0x2B):myArray[2][4] = decimalValue; break;
              case byte(0x2C):myArray[3][4] = decimalValue; break;
              case byte(0x2F):myArray[1][5] = decimalValue; break;
              case byte(0x30):myArray[2][5] = decimalValue; break;
              case byte(0x31):myArray[3][5] = decimalValue; break;
              case byte(0x1A):myArray[4][0] = decimalValue; break;
              case byte(0x1F):myArray[4][1] = decimalValue; break;
              case byte(0x24):myArray[4][2] = decimalValue; break;
              case byte(0x29):myArray[4][3] = decimalValue; break;
              case byte(0x2E):myArray[4][4] = decimalValue; break;
              case byte(0x33):myArray[4][5] = decimalValue; break;
              default: break;
        }
    }

      delay(50 );  // Small delay to prevent CPU overload
    }
   } 
   delay(100 ); 
  }

