#include <QTRSensors.h>

QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];


float Kp = 0.06; 
int Ki = 0;
int Kd = 5;

int P;
int I;
int D;


int lastError = 0;
boolean onoff = false;


const uint8_t maxspeeda = 255;
const uint8_t maxspeedb = 255;
const uint8_t basespeeda = 160;
const uint8_t basespeedb = 160;


int IR1 = A0;
int IR2 = A1;
int IR3 = 7;
int IR4 = 6;
int IR5 = 5;
int IR6 = 4;
int IR7 = 3;
int IR8 = 2;
int IN1 = 8;
int IN2 = 9;
int IN3 = 12;
int IN4 = 13;
int ENA = 10;
int ENB = 11;



void setup() {
  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);
  pinMode(IR3, INPUT);
  pinMode(IR4, INPUT);
  pinMode(IR5, INPUT);
  pinMode(IR6, INPUT);
  pinMode(IR7, INPUT);
  pinMode(IR8, INPUT);
  pinMode(IN1, OUTPUT); 
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // put your setup code here, to run once:
  Serial.begin(9600);

  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){A0, A1 ,7, 6, 5, 4, 3, 2}, SensorCount);

  delay(5000);
  calibration();

}

void loop() {
  PID_control();
  // put your main code here, to run repeatedly:

}

void calibration() {
  digitalWrite(LED_BUILTIN, HIGH);
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW);
  delay(3000);
}


void forward_brake(int speedA, int speedB) {

  if (speedA < 0) {
    speedA  = 0 - speedA;
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
  else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  if (speedB < 0) {
    speedB = 0 - speedB;
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }
  else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
  analogWrite(ENA, speedA);
  analogWrite(ENB, speedB);
}

void PID_control() {
  
  uint16_t position = qtr.readLineBlack(sensorValues); //read the current position
  int error = 3500 - position; //3500 is the ideal position (the centre)
  int da = 1;
  int db = 1;

  P = error;
  I = I + error;
  D = error - lastError;
  lastError = error;
  
  int P_value = P*Kp;
  int I_value = I*Ki;
  int D_value = D*Kd;
  int motorspeed = P*Kp + I*Ki + D*Kd; //calculate the correction
  
  int motorspeeda = basespeeda + motorspeed;
  int motorspeedb = basespeedb -motorspeed;


//  String plus = " + ";
//  String PID_calculation = P_value  + plus +  I_value + plus + D_value;

//  Serial.println(PID_calculation);
  
  if (motorspeeda > maxspeeda) {
    motorspeeda = maxspeeda;
  }
  if (motorspeedb > maxspeedb) {
    motorspeedb = maxspeedb;
  }
  if  (motorspeeda < -255) {
    motorspeeda = -255;
  }
  
  if  (motorspeedb < -255) {
    motorspeedb = -255;
  }
  forward_brake(motorspeeda, motorspeedb);
}