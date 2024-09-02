

#include <QTRSensors.h>
// #include <SparkFun_TB6612.h>


#define IN1 5
#define IN2 7
#define IN3 4
#define IN4 8
#define ENA 3
#define ENB 9
// #define STBY 6



void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
}

void loop() {
  // Example to move both motors forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 200);  // Speed for motor 1
  
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 200);  // Speed for motor 2
  
  delay(2000);

  // Stop motors
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  delay(1000);

  // Example to move both motors backward
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, 200);  // Speed for motor 1
  
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, 200);  // Speed for motor 2

  delay(2000);

  // Stop motors
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  delay(1000);
}


QTRSensors qtr;


const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
int max_speed = 255;   // 225
int turn = 120;
int L = 0;
int R = 0;
int error = 0;
int adj = 0;

float Kp = 0.0728; //0.0728 @volttage 
float Ki =0.0090;  //0.0090
float Kd = 1;//1

int P;
int I;
int D;
int lastError = 0;

uint16_t position;
void setup() {
  brake(motor1, motor2);

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){
  A0, A1, A2, A3, A4, A5, A6, A7 },
  SensorCount);
  qtr.setEmitterPin(2);
  for (uint16_t i = 0; i < 200; i++) {
  qtr.calibrate();
  }
  Serial.println("Done");
  Serial.begin(9600);
}

void loop() {
  // forward(225,225);
   PID_control();
  // checksensor();
}
void PID_control() {
  position = qtr.readLineWhite(sensorValues);
  Serial.print("QTR::");
  Serial.println(position);
  error = 3500 - position;
//  if (position > 1000 && position < 6000) {
    P = error;
    I = I + error;
    D = error - lastError;
 lastError = error;

    adj = P * Kp + I * Ki + D * Kd;

    L = max_speed + adj;
    R = max_speed - adj;

    if (L > max_speed) {
      L = max_speed;
    }
    if (R > max_speed) {
      R = max_speed;
    }
    if (L < 0) {
      L = 0;
    }
    if (R < 0) {
      R = 0;
    }
    forward(L, R);
  }
  //  else{
  //   sharp_left();
  // }

  //  else if (position >=6000 && position <= 7000) {
  //   sharp_right();
  // }
  // //else{
  //   //sharp_right();
  // //}
  




void forward(int L, int R) {
  motor1.drive(L);
  motor2.drive(R);
}
void sharp_right() {
  motor1.drive(-255);
  motor2.drive(255);
}
void sharp_left() {
  motor1.drive(255);
  motor2.drive(-255);
}
