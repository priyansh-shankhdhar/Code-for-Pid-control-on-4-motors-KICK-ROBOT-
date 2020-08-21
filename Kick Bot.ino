#include <directions.h>
#include <Encoder.h>
#include <NewPing.h>

int load_motor1 = A2;
int load_motor2 = A3;
int P = 150; //  for MAX speed
int p = 0; //FOR SPEED CONTROL
int flag;
int factor = 2;
int c=0;
//int Pmax=200; //will be used for slowly speed upgrade/degrade
#define lock_n A0 //for locking

#include <PS3BT.h>
#include <usbhub.h>

// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>
USB Usb;
//USBHub Hub1(&Usb); // Some dongles have a hub inside nhnBTD Btd(&Usb); // You have to create the Bluetooth Dongle instance like so
/* You can create the instance of the class in two ways */
BTD Btd(&Usb);
PS3BT PS3(&Btd); // This will just create the instance
//PS3BT PS3(&Btd, 0x00, 0x15, 0x83, 0x3D, 0x0A, 0x57); // This will also store the bluetooth address - this can be obtained from the dongle when running the sketch

//INITIALISATION FOR FOUR ENCODERS///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

volatile long counterL1 = 0;
volatile long counterL2 = 0;
volatile long counterR1 = 0;
volatile long counterR2 = 0;

Encoder encoder1(35, 37);
Encoder encoder2(41, 39);
Encoder encoder3(45, 43);
Encoder encoder4(47, 49);


//FOR PID////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
long errorS;//L1
long error2;//L2
long error3;//L3
long error_b1;
long error_b2;

float error_sumS, error_diffS;
float error_sum2, error_diff2;
float error_sum3, error_diff3;
float error_sum_b1,error_diff_b1;
float error_sum_b2,error_diff_b2;
//for motor speed
float PL1 = P;
float PL2 = P;
float PL3 = P; //FOR R1
float PL4 = P; //for R2

float last_errorS;//L1
float last_error2;//L2
float last_error3;//L3
float last_error_b1;
float last_error_b2;

//////////////////////////////////////////////////////////PID tuning/////////////////////////////////////////////////////////////////////////////////////////////////
long xpid, PIDS, PID2, PID3, PID_b1, PID_b2;
//motor l2//***********************************************************************************************************************
float kpS = 0.001;
float kiS = 0.0;
float kdS = 0.0;
//motor r1//***********************************************************************************************************************
float kp2 = 0.0179101 ; //0.037//0.025//0.01   //0.0179101
float ki2 = 0.006; //0.01 //0.02 //0.015    //0.1 //0.005
float kd2 = 0.02; //0.05//0.008 //0.01
//motor r2//***********************************************************************************************************************
float kp3 = 0.013; //0.037//0.0108
float ki3 = 0.01; //0.01
float kd3 = 0.03; //0.05//0.07
//BACKWARD//
//motor R1///////////////
float kp_b1=0.0179101 ;
float ki_b1=0.005;
float kd_b1= 0.015;
//motor R2///////////////
float kp_b2;
float ki_b2;
float kd_b2;

int u = P + factor;
int d = P - factor;

int u1 = P + factor;
int d1 = P - (factor * 3);

int u2 = P - 1;
int d2 = P - (factor * 5.5);
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//for ps3 console
int lhx;
int lhy;
int rhx;
int rhy;

//INITIALISATION FOR MOTORS///////////////////////////////////////////////////////////////////////////////////////////////////////////////
int L1PWM = 8;
int R1PWM = 5;
int L2PWM = 6;
int R2PWM = 7;
int LX, LY;
int L1DIR = 33;
int R1DIR = 27;
int L2DIR = 29;
int R2DIR = 31;

void round_left();
void round_right();


#define TRIGGER_PIN1  36  
#define ECHO_PIN1     34 
#define MAX_DISTANCE1 200
#define TRIGGER_PIN2  32  
#define ECHO_PIN2     30  
#define MAX_DISTANCE2 200


NewPing sonar1(TRIGGER_PIN1, ECHO_PIN1, MAX_DISTANCE1); 
NewPing sonar2(TRIGGER_PIN2, ECHO_PIN2, MAX_DISTANCE2); 


//PID
void Upid();
int errorU;
int lasterrorU;
int errorsumU;
int errordiffU;
float val;

float kpU=10.0;
float kiU=0.5;    //1
float kdU=0.05;    //0

float distance1;
float distance2;

void setup()
{ pinMode(A0,OUTPUT);
  pinMode(A11,OUTPUT);
  pinMode(A1,OUTPUT);
  pinMode(A2,OUTPUT);
  pinMode(A3,OUTPUT);
  SerialUSB.begin(1000000);
#if !defined(__MIPSEL__)
  while (!SerialUSB); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
  if (Usb.Init() == -1) {
    SerialUSB.print(F("\r\nOSC did not start"));
    while (1); //halt
  }
  SerialUSB.print(F("\r\nPS3 Bluetooth Library Started"));


  pinMode(L1PWM, OUTPUT);
  pinMode(L2PWM, OUTPUT);
  pinMode(R1PWM, OUTPUT);
  pinMode(R2PWM, OUTPUT);

  pinMode(L1DIR, OUTPUT);
  pinMode(L2DIR, OUTPUT);
  pinMode(R1DIR, OUTPUT);
  pinMode(R2DIR, OUTPUT);


}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop()
{

  counterL1 = encoder1.read();
  counterL2 = encoder2.read();
  counterR2 = encoder3.read();
  counterR1 = encoder4.read();

  Usb.Task();


  if (PS3.PS3Connected)
  {

    lhx = PS3.getAnalogHat(LeftHatX);
    lhy = PS3.getAnalogHat(LeftHatY);
    rhx = PS3.getAnalogHat(RightHatX);
    rhy = PS3.getAnalogHat(RightHatY);





    //joystick functions..........................................................................

    if ((lhy < 125) && (lhy >= 0) && ((lhx > 117) && (lhx < 137)))
    {
      flag=1;
      u1 = 152;
      d1 = 135;

      u2 = 150;
      d2 = 134;

      forward(P);

      SerialUSB.print("L1: ");
      SerialUSB.print(counterL1);
      SerialUSB.print("       ");
      SerialUSB.print("L2: ");
      SerialUSB.print(counterL2);
      SerialUSB.print("       ");
      SerialUSB.print("R1: ");
      SerialUSB.print(counterR1);
      SerialUSB.print("       ");
      SerialUSB.print("R2: ");
      SerialUSB.println(counterR2);
      
    }
    else if ((lhy > 130) && (lhy <= 255) &&  (lhx > 117) && (lhx < 137))
    {
      p=150;
      backward();
      
      u1 = 160;
      d1 = 130;

      u2 = 160;
      d2 = 130;


    }
    else if ((lhx < 125) && (lhx >= 0) && ((lhy > 117) && (lhy < 137)))
    {
      p=60;
      left();

    }

    else if ((lhx <= 255) && (lhx > 130) && ((lhy > 117) && (lhy < 137)))
    {
      p=60;
      right();

    }
    else if   (PS3.getAnalogButton(LEFT) )
    {
      round_left();
    }

    else if   (PS3.getAnalogButton(RIGHT) )
    { 
      round_right();
    }


    else if(PS3.getAnalogButton(L1))
    {
      //ULTRASONIC SENSORS FOR BALL DETECTION AND PID
distance1 = sonar1.ping_cm();
distance2 = sonar2.ping_cm();
 
 if( distance1 <20 && distance2<20 )
  {
     
      Upid();
  }
    }

    else
    {

      stopp();

    }

    if   (PS3.getAnalogButton(CROSS) )
    { 
      encoder1.write(0);
      encoder2.write(0);
      encoder3.write(0);
      encoder4.write(0);

      SerialUSB.println("STOP");


      digitalWrite(L1DIR, 0);
      digitalWrite(L2DIR, 1);
      digitalWrite(R1DIR, 1);
      digitalWrite(R2DIR, 0);

      analogWrite(L1PWM, 0);
      analogWrite(R1PWM, 0);
      analogWrite(L2PWM, 0);
      analogWrite(R2PWM, 0);
      //delay(50);
    }
    if (PS3.getAnalogButton(CIRCLE)) {
      analogWrite(load_motor1, 0); //this is for the kicking
      analogWrite(load_motor2, 1023);
      SerialUSB.println("CIRCLE");
    }
    else {
      digitalWrite(load_motor1, LOW);
      digitalWrite(load_motor2, LOW);
    }
     if (PS3.getAnalogButton(UP))
    {
       analogWrite(A0, 1023);
       analogWrite(A11, 1023);
    }

      if (PS3.getAnalogButton(DOWN))
     {
        analogWrite(A0,0);
        analogWrite(A11,0);
        
     }

      

    //Square to open the locking
    if (PS3.getButtonClick(SQUARE))
    {
     
      analogWrite(A1,1023);
  
    }

    //triangle to close the locking
    if (PS3.getButtonClick(TRIANGLE))
    {
      analogWrite(A1,0);
    }

    if ((rhy < 125) && (rhy >= 0))
    { c=10000;
      stop_at_straight(c);
      SerialUSB.print("entered");
      SerialUSB.println(PIDS);
    }
    ////////////////////////////////////////////////for printing the encoder values
    /*  SerialUSB.print("L1: ");
      SerialUSB.print(counterL1);
      SerialUSB.print("       ");
      SerialUSB.print("L2: ");
      SerialUSB.print(counterL2);
      SerialUSB.print("       ");
      SerialUSB.print("R1: ");
      SerialUSB.print(counterR1);
      SerialUSB.print("       ");
      SerialUSB.print("R2: ");
      SerialUSB.println(counterR2);
      //Serial.println(error1);*/
    ////////////////////////////////////////////////PRITING THE ERROR

       SerialUSB.print(error_b1);
       SerialUSB.print("    ");
       SerialUSB.println(error_b2);
/*  SerialUSB.print("counter R1:");
    SerialUSB.print(counterR1);
    SerialUSB.print("    error2:");
    SerialUSB.print(error2);
    SerialUSB.print("    error3:");
    SerialUSB.println(error3);*/

  }
}
///////////////////////////////////////////////////////////////////////////////////////////////
// for motor R1**************************************************************************************************************************
int pid2() {

  error2 = counterL2 - counterR1;
  error_sum2 = error2 + last_error2;
  error_diff2 = error2 - last_error2;



  PID2 = (kp2 * error2) + (ki2 * error_sum2) + (kd2 * error_diff2) ;
  PID2 = abs(PID2);

  if (error2 == 0)
  {
    PL3 = PL2;
  }
  if (error2 > 0)
  {
    PL3 = PL3 + PID2;

  }
  if (error2 < 0)
  {
    PL3 = PL3 - PID2;

  }
  if (PL3 < d1)
    PL3 = d1;
  if (PL3 > u1)
    PL3 = u1;
  last_error2 = error2;
  // SerialUSB.println(PL3);
  return PL3;
}

//for motor R2****************************************************************************************************************************

int pid3() {
  error3 = counterL2 - counterR2;
  error_sum3 = error3 + last_error3;
  error_diff3 = error3 - last_error3;
  PID3 = (kp3 * error3) + (ki3 * error_sum3) + (kd3 * error_diff3) ;
  PID3 = abs(PID3);

  if (error3 == 0)
  {
    PL4 = PL2;
  }
  if (error3 < 0)
  {
    PL4 = PL4 - PID3;

    if (PL4 < d2)
      PL4 = d2;
    if (PL4 > u2)
      PL4 = u2;
  }
  if (error3 > 0)
  {
    PL4 = PL4 + PID3;

    if (PL4 < d2)
      PL4 = d2;
    if (PL4 > u2)
      PL4 = u2;
  }
  last_error3 = error3;
  return PL4;
}
////////////////////////////////////////////////////////////////STOPPING PID/////////////////////////////////////////
void stop_at_straight(int dist)
{

  errorS = dist - counterL2;
  error_sumS = errorS + last_errorS;
  error_diffS = errorS - last_errorS;
  PIDS = kpS * errorS + kiS * error_sumS + (kdS * error_diffS) ;
  PIDS = abs(PIDS);
  if (PIDS > P)
    PIDS = P;
  if (PIDS < 0)
    PIDS = 0;

  if (errorS < 0)
  { 
    backward();
  }
  else if (errorS > 0)
  {
    int y=140+PIDS;
    if (y>150)
    y=150;
    forward(y);
  }
  else if (errorS == 0)
  {
    analogWrite(L1PWM, 0);
    analogWrite(R1PWM, 0);
    analogWrite(L2PWM, 0);
    analogWrite(R2PWM, 0);
  }
  last_errorS = errorS;

}
//for turning////////////////////////////////////////////////////////////// 
int turning()
{
  if(p>0)
  {
    p=p-5;
  }
  if(p<0)
  {
    p=0;
  }

  PL2=p;
  PL3=p;
  PL4=p;
  PL1=p;
   digitalWrite(L1DIR, 1);
    digitalWrite(L2DIR, 0);
    digitalWrite(R1DIR, 1);
    digitalWrite(R2DIR, 0);

    analogWrite(L1PWM, PL1);
    analogWrite(R1PWM, PL2);
    analogWrite(L2PWM, PL3);
    analogWrite(R2PWM, PL4);
  
}


void Upid()
{ 
  errorU = distance1 - distance2;
  //SerialUSB.println(errorU);
  errorsumU = errorU + lasterrorU;
  errordiffU = errorU - lasterrorU;
  val = kpU*errorU + kiU*errorsumU + kdU*errordiffU;
  val = abs(val);
   SerialUSB.println(errorU);
  
  if(val<10)
  {
    val=10;
  }
  if(val>50)
  {
    val= 50;
  }
  if(errorU>0)
  {
    
    

      analogWrite(L1PWM, val);
      analogWrite(R1PWM, val);
      analogWrite(L2PWM, val);
      analogWrite(R2PWM, val);
       digitalWrite(L1DIR, 1);
      digitalWrite(L2DIR, 0);
      digitalWrite(R1DIR, 1);
      digitalWrite(R2DIR, 0);
   
  }
  if(errorU<0) 
  {
    //SerialUSB.print("aaaaaaa ");
    

      analogWrite(L1PWM, val);
      analogWrite(R1PWM, val);
      analogWrite(L2PWM, val);
      analogWrite(R2PWM, val);
       digitalWrite(L1DIR, 0);
      digitalWrite(L2DIR, 1);
      digitalWrite(R1DIR, 0);
      digitalWrite(R2DIR, 1);
   
  }
  if(errorU=0)
  {

      analogWrite(L1PWM, 0);
      analogWrite(R1PWM, 0);
      analogWrite(L2PWM, 0);
      analogWrite(R2PWM, 0);
  }
  lasterrorU =errorU;
 
}

int pid_back1()
{

  error_b1 = counterL2 - counterR1;
  error_sum_b1 = error_b1 + last_error_b1;
  error_diff_b1 = error_b1 - last_error_b1;
   
  PID_b1 = (kp_b1 * error_b1) + (ki_b1 * error_sum_b1) + (kd_b1 * error_diff_b1) ;
  PID_b1 =abs(PID_b1);
  
  if (error_b1==0)  
   {
    PL3 = PL2;
  }
  if (error_b1>0)
  {
    PL3 = PL3 - PID_b1;

  }
  if (error_b1<0)
  {
    PL3 = PL3 + PID_b1;
  }
  if (PL3 < d1)
      PL3 = d1;
  if (PL3 > u1)
      PL3 = u1;
  last_error_b1 = error_b1;
  
  return PL3;
  
}
/////////////////////////////////////////////////////////////////////////////////////
int pid_back2()
{
  error_b2 = counterL2 - counterR2;
  error_sum_b2 = error_b2 + last_error_b2;
  error_diff_b2 = error_b2 - last_error_b2;
  
  PID_b2 = (kp_b2 * error_b2) + (ki_b2 * error_sum_b2) + (kd_b2 * error_diff_b2) ;
  PID_b2 =abs(PID_b2);
  
  if (error_b2==0)  
   {
    PL4 = PL2;
  }
  if (error_b2>0)
  {
    PL4 = PL4 - PID_b2;

  }
  if (error_b2<0)
  {
    PL4 = PL4 + PID_b2; 
  }
  
  if (PL4 < 0)
      PL4 = 0;
  if (PL4 > u2)
      PL4 = u2;
      
  last_error_b2 = error_b2;
 
  return PL4;
  
}
