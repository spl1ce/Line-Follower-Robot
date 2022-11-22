#include <QTRSensors.h>

QTRSensors qtr;
const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];


float Kp = 0.123;
float Ki = 0.00005;
float Kd = 2;

int P;
int I;
int D;


int lastError = 0;
boolean onoff = false;


const uint8_t maxspeeda = 255;
const uint8_t maxspeedb = 255;
const uint8_t basespeeda = 255;
const uint8_t basespeedb = 255;



int IR2 = 2;
int IR3 = 3;
int IR4 = 4;
int IR5 = 5;
int IR6 = 6;
int IR7 = 7;
int IN1 = 10;
int IN2 = 11;
int IN3 = 12;
int IN4 = 13;
int ENA = 8;
int ENB = 9;



void setup() {
  pinMode(IR2, INPUT);
  pinMode(IR3, INPUT);
  pinMode(IR4, INPUT);
  pinMode(IR5, INPUT);
  pinMode(IR6, INPUT);
  pinMode(IR7, INPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // put your setup code here, to run once:
  Serial.begin(9600);

  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){2, 3, 4, 5, 6, 7}, SensorCount);

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
  //set the appropriate values for aphase and bphase so that the robot goes straight
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, speedA);
  analogWrite(ENB, speedB);
}

void PID_control() {
  uint16_t position = qtr.readLineBlack(sensorValues); //read the current position
  int error = 3500 - 1000 - position; //3500 is the ideal position (the centre)

  P = error;
  I = I + error;
  D = error - lastError;
  lastError = error;
  int motorspeed = P*Kp + I*Ki + D*Kd; //calculate the correction
                                       //needed to be applied to the speed
  
  int motorspeeda = basespeeda - motorspeed;
  int motorspeedb = basespeedb + motorspeed;

  if (motorspeeda > maxspeeda) {
    motorspeeda = maxspeeda;
  }
  if (motorspeedb > maxspeedb) {
    motorspeedb = maxspeedb;
  }
  if (motorspeeda < 0) {
    motorspeeda = 0;
  }
  if (motorspeedb < 0) {
    motorspeedb = 0;
  }
  Serial.println(motorspeeda);
  Serial.println("\t");
  Serial.println(motorspeedb);
  forward_brake(motorspeeda, motorspeedb);
}