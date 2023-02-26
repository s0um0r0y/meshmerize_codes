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
  forward(motor1,motor2,60);
  delay(40);
  forward(motor1,motor2,80);
  delay(40);
  forward(motor1,motor2,100);
  delay(40);
  maze();
}

void calibration(){
  for (int i=0;i<=100;i++){
    if (i<25 || i>=75){
      left(motor1,motor2,100);
    }
    else{
      right(motor1,motor2,100);
    }
    qtra.calibrate();
    delay(10);
  }
  brake(motor1,motor2);
  for (int i=0;i<NUM_SENSORS;i++){
    Serial.print(qtra.calibrationOn.minimum[i]);
    Serial.print(" ");
    thr[i]=(qtra.calibrationOn.minimum[i]+qtra.calibrationOn.maximum[i])/2;
  }
  Serial.println();
  for (int i=0;i<NUM_SENSORS;i++){
    Serial.print(qtra.calibrationOn.maximum[i]);
    Serial.print(" ");
  }
  Serial.println();
  Serial.println(thr[0]);
  Serial.println(thr[7]);
}

void follow_segment(){
  while (1)
  {
    digitalWrite(led,LOW);
    int position=qtra.readLineWhite(sensor1);
    int error=3500-position;
    int motorSpeed=kp*error+kd*(error-lastError);
    lastError=error;
    int rightMotorSpeed=BaseSpeed-motorSpeed;
    int leftMotorSpeed=BaseSpeed+motorSpeed;
    if (rightMotorSpeed> MaxSpeed ) rightMotorSpeed=MaxSpeed;
    if (leftMotorSpeed> MaxSpeed ) leftMotorSpeed=MaxSpeed;
    if (rightMotorSpeed< 0) rightMotorSpeed=0;
    if (leftMotorSpeed< 0) leftMotorSpeed=0;
    moto1.drive(rightMotorSpeed);
    moto2.drive(leftMotorSpeed);
    if ((sensor1[0] < thr[0]) || (sensor1[7]<thr[7])){
      return;
    }
    else if (sensor1[1]>thr[1] && sensor1[2]>thr[2] && sensor1[3]>thr[3] && sensor1[4]>thr[4] && sensor1[5]>thr[5] && sensor1[6]>thr[6]){
      return;
    }
    delay(5);
  } 
}
void maze(){
  while (1)
  {
    follow_segment();
    digitalWrite(led,HIGH);
    brake(motor1,motor2);
    unsigned char found_left=0;
    unsigned char found_straight=0;
    unsigned char found_right=0;
    qtra.readLineWhite(sensor1);
    if (sensor1[7]<thr[7])
    {
      found_left=1;
      if (sensor1[1]<thr[1]){
        found_right=1;
        if (sensor1[6]<thr[6]){
          found_left=1;
        }
      }
    forward(motor1,motor2,60);
    delay(200);
    brake(motor1,motor2);
    qtra.readLineWhite(sensor1);
    if (sensor1[1]<thr[1] && sensor1[2]<thr[2] && sensor1[3]<thr[3] && sensor1[4]<thr[4] && sensor1[5]<thr[5] && sensor1[6]<thr[6])  
      break;
    if (chr==1) dir=select_turnL(found_left,found_straight,found_right);
    else if (chr==2) dir=select_turnR(found_right,found_left);
    Serial.println(dir);
    turn(dir);
    path[path_length]=dir;
    path_length++;
    simplify_path();
    }
    brake(motor1,motor2);
    forward(motor1,motor2,80);
    delay(400);
    brake(motor1,motor2);
    for (int w=0;w < path_length; w++)
    {
      Serial.print(path[w]);
      Serial.print(' ');
    }
    digitalWrite(led,HIGH);
    delay(4000);
    digitalWrite(led,LOW);
    s2=digitalRead(sw2);
    while (s2==HIGH)
    {
      s2=digitalRead(sw2);
    }
    delay(800);
    forward(motor1,motor2,60);
    delay(40);
    forward(motor1,motor2,80);
    delay(40);
    forward(motor1,motor2,100);
    delay(40);
    while (1)
    {
      int k;
      for (k=0;k<path_length;k++){
        follow_segment();
        forward(motor1,motor2,50);
        delay(50);
        forward(motor1,motor2,60);
        delay(200);
        brake(motor1,motor2);
        delay(5);
        turn(path[k]);
      }
      follow_segment();
      forward(motor1,motor2,50);
      delay(50);
      forward(motor1,motor2,60);
      delay(200);
      brake(motor1,motor2);
      delay(5);
      turn(path[k]);
    }
    follow_segment();
    brake(motor1,motor2);
    forward(motor1,motor2,80);
    delay(400);
    brake(motor1,motor2);
    digitalWrite(led,HIGH);
    delay(4000);
    digitalWrite(led,LOW);
  }
  char select_turnL(char found_left,char found_straight,char found_right){
    if (found_left) return 'L';
    else if (found_straight) return 'S';
    else if (found_right) return 'R';
    else return 'B';
  }
  char select_turnR(char found_right,char found_straight,char found_left){

  }
}
