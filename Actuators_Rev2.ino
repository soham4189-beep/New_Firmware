#include <SoftwareSerial.h>
#define RR_A 9
#define RR_B 10
#define RL_A 7
#define RL_B 8
#define FL_A 1
#define FL_B 5
#define FR_A 4
#define FR_B 6
#define BT_RX 2
#define BT_TX 3   
SoftwareSerial bt(BT_RX, BT_TX);
int A=1;
void setup() 
{
pinMode(RR_A, OUTPUT);
pinMode(RR_B, OUTPUT);
pinMode(RL_A, OUTPUT);
pinMode(RL_B, OUTPUT);
pinMode(FL_A, OUTPUT);
pinMode(FL_B, OUTPUT);
pinMode(FR_A, OUTPUT);
pinMode(FR_B, OUTPUT);
bt.begin(9600);     // Bluetooth baud rate
All_Stop();
}
void loop() 
{
  if (bt.available())
  {
    char cmd = bt.read();

    if (cmd == 'X')        // Home position
    {
      Home();
    }
    else if (cmd == 'Z')   // All actuators OUT
    {
      All_In();
    }
  }
}
void RR_IN()  { digitalWrite(RR_A, HIGH); digitalWrite(RR_B, LOW); }
void RR_OUT() { digitalWrite(RR_A, LOW);digitalWrite(RR_B, HIGH); }
void RR_STOP(){ digitalWrite(RR_A, HIGH); digitalWrite(RR_B, HIGH); }
void RL_OUT() { digitalWrite(RL_A, HIGH);digitalWrite(RL_B, LOW); }
void RL_IN()  { digitalWrite(RL_A, LOW); digitalWrite(RL_B, HIGH); }
void RL_STOP(){ digitalWrite(RL_A, HIGH); digitalWrite(RL_B, HIGH); }
void FL_IN()  { digitalWrite(FL_A, HIGH); digitalWrite(FL_B, LOW); }
void FL_OUT() { digitalWrite(FL_A, LOW);digitalWrite(FL_B, HIGH); }
void FL_STOP(){ digitalWrite(FL_A, HIGH); digitalWrite(FL_B, HIGH); }
void FR_IN()  { digitalWrite(FR_A, HIGH); digitalWrite(FR_B, LOW); }
void FR_OUT() { digitalWrite(FR_A, LOW);digitalWrite(FR_B, HIGH); }
void FR_STOP(){ digitalWrite(FR_A, HIGH); digitalWrite(FR_B,HIGH); }
void All_Out()
{
if(A)
{
RR_OUT();
RL_OUT();
FR_OUT();
FL_OUT();
A=0;
}
}
void All_In()
{
 RR_IN();
 RL_IN();
 FL_IN();
 FR_IN();
}
void Home()
{
All_Out();

delay(23000);
RR_STOP();
delay(1000);
RL_STOP();
delay(6250);
FL_STOP();
delay(4500);
FR_STOP();
}
void All_Stop()
{ 
RR_STOP();
RL_STOP();
FL_STOP();
FR_STOP(); 
}