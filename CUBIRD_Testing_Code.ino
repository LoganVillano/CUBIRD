#include <rcc.h>
//motor A
#define DIRA 12
#define PWMA 3
#define BRAKEA 9
//motor B
#define DIRB 13
#define PWMB 11
#define BRAKEB 8
//encoders
#define ENCA 18
#define ENCB 19
#define ENCC 20
#define ENCD 21
struct DataFrame{
  unsigned long time;
  double motorSpeedA;
  double motorSpeedB;
  double pwmA;
  double pwmB;
  double uffA;
  double uffB;
};
//set up position variables for encoder A and B
volatile int posiA = 0;
volatile int posiB = 0;
//variables setup
unsigned long cur,prev;
unsigned long dt = 10000;
bool state = 0;
double derivative;
double derivativeB;
//motor power setup
float pwra = 100;
float pwrb = 100;
float sigma = 0.05;
Differentiator diff(sigma, (dt/1000000.0));
Differentiator diffb(sigma, (dt/1000000.0));
//PID on motor A
float kpa = 2;
float kia = 12;
float kda = 0;
PID_control ctrl(kpa, kia, kda, 0, 170, sigma, dt/1e6);
//PID on motor B
float kpb = 2;
float kib = 12;
float kdb = 0;
PID_control ctrlb(kpb, kib, kdb, 0, 170, sigma, dt/1e6);
//positional PID between motors A and B
float kpp = 3;
float kip = 0;
float kdp = 0;
PID_control ctrlp(kpp, kip, kdp, 0, 170, sigma, dt/1e6);

int rep = 0;
float a = 0;
float b = 0;
//button setup
int buttonStatus = 1;
int buttonStop = 1;
int t = -1;
void setup() {
  ctrl.antiWindupEnabled = true;
  ctrlb.antiWindupEnabled = true;
  // put your setup code here, to run once:
  Serial.begin(115200);
  //motor A
  pinMode(DIRA, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BRAKEA, OUTPUT);
  pinMode(ENCA,INPUT_PULLUP);
  pinMode(ENCB,INPUT);
  //motor B
  pinMode(DIRB, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BRAKEB, OUTPUT);
  pinMode(ENCC,INPUT_PULLUP);
  pinMode(ENCD,INPUT);
  //encoders
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoderA,CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCB),readEncoderA,CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCC),readEncoderB,CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCD),readEncoderB,CHANGE);
  //Button
  pinMode(53, INPUT_PULLUP);
  pinMode(51, INPUT_PULLUP);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  digitalWrite(6, LOW);
}
void loop()
{
  //take current time
  cur = micros();
  //read value of both buttons
  int pinValue = digitalRead(53);
  int stopValue = digitalRead(51);
  //if start button pressed, send start signal to DAQ and set timt (t) to 0
  if (buttonStatus != pinValue)
  {
    buttonStatus = pinValue;
    if (buttonStatus == 0);
    {
      t =0;
      digitalWrite(6, HIGH);
    }
  }
  //if stop button pressed, cut run loop by setting t high. Send stop signal to DAQ
  if (buttonStop != stopValue)
  {
    buttonStop = stopValue;
    if (buttonStop == 0);
    {
      t = 100000;
      digitalWrite(6,LOW);
    }
  }
  //if time between 0-10 sec after button is pressed, run code
  if (t>=0 && t<1000)
  {
    //data taken at each dt
    if((cur-prev)>= dt)
    {
      //send signal to DAQ
      digitalWrite(7, state);
      //set motor A target  
      int target = 7;
      //set feedforward value for A
      float uff = (target-0.3105)/1.0829;
      //Get current counts
      //Convert to radians
      //Do dirty derivative on radians
      float delT = (cur - prev) / 1.0e6;
      //determine frequency of motor A and B
      derivative = diff.differentiate(posiA/12.0);
      derivativeB = diffb.differentiate(posiB/12.0);
      //set feedforward value for B
      float uffb = (derivative-0.3105)/1.0829;
      //int deltacount = posiA-posiA_prev;
      //find u value for all 3 PID loops
      float u = (uff*6) + ctrl.pid(target, derivative);
      float up = derivative + ctrlp.pid(posiA, posiB);
      float uB = (uffb *6)+ ctrlb.pid(up, derivativeB);
      //set power of motors based on u values
      pwra = (int)u;
      pwrb = (int)uB;
      //limit power to be between 0 and 170
      if (pwra > 170)
      {
         pwra = 170;
      }
      if (pwra < 0)
      {
         pwra = 0;
      }
      if (pwrb > 170)
      {
        pwrb = 170;
      }
      if (pwrb < 0)
      {
        pwrb = 0;
      }
      //run motors
      setMotor(pwra,1,pwrb,1);
      //Reset prev vals
      prev = cur;
      state = !state;
      //print time averages to clean up data
      a = a + derivative;
      b = b + derivativeB;
      rep = rep +1;
      if (rep > 9)
      {
        a = a/10;
        b = b/10;
        rep = 0;
        a = 0;
        b = 0;
      }
      //print values to track data
      //Serial.print('$');
      //Serial.print(uff);
      //Serial.print(',');
      Serial.print(derivative);
      Serial.print(',');
      Serial.print(derivativeB);
      Serial.print(',');
      Serial.print(pwra);
      Serial.print(',');
      Serial.print(pwrb);
      //Serial.print(',');
      Serial.println();
      t++;
    }
    //Print data from ctrl loop over serial
  }
  //if not running loop, stop motors
  else
  {
    pwra = 0;
    pwrb = 0;
    setMotor(pwra,1,pwrb,1);
    digitalWrite(6,LOW);
  }
}
//function to run motors
void setMotor(int pwma, int dira, int pwmb, int dirb)
{
  if(dira){
  digitalWrite(BRAKEA, LOW);
  digitalWrite(DIRA, HIGH);
  analogWrite(PWMA, pwma);
  digitalWrite(BRAKEB, LOW);
  digitalWrite(DIRB, HIGH);
  analogWrite(PWMB, pwmb);
  }
}
//track position via encoders for A and B 
void readEncoderA(){
  posiA++;
}
void readEncoderB(){
  posiB++;
}