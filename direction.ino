void forward(int P)///////////////////////////////////////////////////////// FORWARD/////////////////////////////////////////////////////////
{
flag=1;
  if (p < P)
  {
    p = p + 5;

    PL2 = p;    //MASTER MOTOR L2
    PL3 = p; //pid
    PL4 = p; //pid
    PL1 = p;  //COPYING MASTER MOTOR in L1 MOTOR


    if (PL1 < 0)
      PL1 = 0;
    if (PL1 > 150)
      PL1 = 150;
    if (PL2 < 0)
      PL2 = 0;
    if (PL2 > 150)
      PL2 = 150;
    if (PL3 < 0)
      PL3 = 0;
    if (PL3 > 150)
      PL3 = 150;
    if (PL4 < 0)
      PL4 = 0;
    if (PL4 > 150)
      PL4 = 150;
    digitalWrite(L1DIR, 0);
    digitalWrite(L2DIR, 1);
    digitalWrite(R1DIR, 1);
    digitalWrite(R2DIR, 0);

    analogWrite(L1PWM, PL1);
    analogWrite(R1PWM, PL3);
    analogWrite(L2PWM, PL2);
    analogWrite(R2PWM, PL4);
    delay(30);
  }

  else  {

    PL2 = P;    //MASTER MOTOR L2
    PL3 = pid2(); //pid
    PL4 = pid3(); //pid
    PL1 = P+11;//PL3 + 1.5; //COPYING MASTER MOTOR in L1 MOTOR
    // SerialUSB.println(PL3);
    if (PL1 < 0)
      PL1 = 0;
    if (PL2 < 0)
      PL2 = 0;
    if (PL3 < 0)
      PL3 = 0;
    if (PL4 < 0)
      PL4 = 0;

    digitalWrite(L1DIR, 0);
    digitalWrite(L2DIR, 1);
    digitalWrite(R1DIR, 1);
    digitalWrite(R2DIR, 0);

    analogWrite(L1PWM, PL1);
    analogWrite(R1PWM, PL3);
    analogWrite(L2PWM, PL2);
    analogWrite(R2PWM, PL4);

  }
}

void backward() //////////////////////////////////////////////////////BACKWARD///////////////////////////////////////////////////
{
   //Serial.println("backward");
   PL2=P;
   PL3=pid_back1();
   PL4=pid_back2();
   PL1=P;

  /*  if (PL1 < 0)
      PL1 = 0;
    if (PL2 < 0)
      PL2 = 0;
    if (PL3 < 0)
      PL3 = 0;
    if (PL4 < 0)
      PL4 = 0;
    if(PL1>160)
    PL1=160;
    if(PL2>160)
    PL2=160;
    if(PL3>160)
    PL3=160;
    if(PL4>160)
    PL4=160;*/
   
  digitalWrite(L1DIR, 1);
  digitalWrite(L2DIR, 0);
  digitalWrite(R1DIR, 0);
  digitalWrite(R2DIR, 1);

  analogWrite(L1PWM, PL1);
  analogWrite(R1PWM, PL3);
  analogWrite(L2PWM, PL2);
  analogWrite(R2PWM, PL4);
}

void left()//////////////////////////////////////////////////////////left/////////////////////////////////////////////
{
  SerialUSB.println("left");

  digitalWrite(L1DIR, 1);
  digitalWrite(L2DIR, 1);
  digitalWrite(R1DIR, 1);
  digitalWrite(R2DIR, 1);

  analogWrite(L1PWM, P);
  analogWrite(R1PWM, P);
  analogWrite(L2PWM, P);
  analogWrite(R2PWM, P);
}
void right()////////////////////////////////////////RIGHT/////////////////////////////////////////////
{
  SerialUSB.println("right");
  digitalWrite(L1DIR, 0);
  digitalWrite(L2DIR, 0);
  digitalWrite(R1DIR, 0);
  digitalWrite(R2DIR, 0);

  analogWrite(L1PWM, P);
  analogWrite(R1PWM, P);
  analogWrite(L2PWM, P);
  analogWrite(R2PWM, P);
}
void stopp()///////////////////////////////////////////////////////STOPPING ////////////////////////////////////////////////////
{
if (PS3.getAnalogButton(L2))
 {
  if (flag==1)
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
    delay(30);
    }}
 
else {
  if (p>0){
   p=p-5;
  if (p < 0)
   {  p = 0;}
    
    analogWrite(L1PWM, p);
    analogWrite(R1PWM, p);
    analogWrite(L2PWM, p);
    analogWrite(R2PWM, p);
    delay(15);
    
}
else 
{
    analogWrite(L1PWM, 0);
    analogWrite(R1PWM, 0);
    analogWrite(L2PWM, 0);
    analogWrite(R2PWM, 0);

  
  }}
  if (p==0)
  flag=0;
}
void round_right()
{
  digitalWrite(L1DIR, 0);
  digitalWrite(L2DIR, 1);
  digitalWrite(R1DIR, 0);
  digitalWrite(R2DIR, 1);

  analogWrite(L1PWM, 50);
  analogWrite(R1PWM, 50);
  analogWrite(L2PWM, 50);
  analogWrite(R2PWM, 50);

}

void round_left()
{
  digitalWrite(L1DIR, 1);
  digitalWrite(L2DIR, 0);
  digitalWrite(R1DIR, 1);
  digitalWrite(R2DIR, 0);

  analogWrite(L1PWM, 50);
  analogWrite(R1PWM, 50);
  analogWrite(L2PWM, 50);
  analogWrite(R2PWM, 50);

}
