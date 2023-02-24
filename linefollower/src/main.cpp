#include <Arduino.h>
// #include <SparkFun_TB6612.h>
// #include <QTRSensor.h>

#define AIN1 5
#define BIN1 7
#define AIN2 4
#define BIN2 8
#define PWMA 3
#define PWMB 9

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

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:
}