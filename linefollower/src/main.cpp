#include <Arduino.h>
// #include <SparkFun_TB6612.h>
// #include <QTRSensor.h>

#define AIN1 5
#define BIN1 7
#define AIN2 4
#define BIN2 8
#define PWMA 3
#define PWMB 9
#define STBY 6

const int offsetA=1;
const int offsetB=1;

#define sw1 10
#define sw2 11
#define led 12
int s1;
int s2;
int chr=0;
char dis;
int obs1;
int obs=2;

#define NUM_SENSORS 8
unsigned int sensor1[8];
int thr[8];

// #define MaxSpeed ***
// #define BaseSpeed ***
int lastError=0;
// float kp= ***;
// float kd= ***;
// int last_pos= ****;

int num=0;
char path[100];
int path_length=0;

// Motor motor1=Motor(AIN1,AIN2,PWMA,offsetA,STBY);
// Motor motor2=Motor(BIN1,BIN2,PWMB,offsetB,STBY);
// QTRSensors qtra;

void setup() {
  // put your setup code here, to run once:
  //qtra.setTypeAnalog();
  //qtra.setSensorPins((const uint8_t[]) {A0,A1,A2,A3,A4,A5,A6,A7},NUM_SENSOR);
  pinMode(sw1,INPUT);
  pinMode(sw2,INPUT);
  pinMode(led,OUTPUT);
  pinMode(13,OUTPUT);
  pinMode(obs,INPUT);
  digitalWrite(13,LOW);
  Serial.begin(9600);
  while (!Serial)
  {
    ;
  }
  s1=digitalRead(sw1);
  while (s1==HIGH)
  {
    s1=digitalRead(sw1);
  }
  calibration();
  while (1)
  {
    int s1=digitalRead(sw1);
    int s2=digitalRead(sw2);
    if (s1==LOW)
    {
      digitalWrite(13,HIGH);
      chr=1;
      break;
    }
    if (s2==LOW)
    {
      digitalWrite(13,LOW);
      chr=2;
      break;
    }
  }
  Serial.println(chr);
  delay(900);
  s2=digitalRead(sw2);
  while (s2==HIGH)
  {
    s2=digitalRead(sw2);
  }
  delay(800);
}

void loop() {
  // put your main code here, to run repeatedly:
}
