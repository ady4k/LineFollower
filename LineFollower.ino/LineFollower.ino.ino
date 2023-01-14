#include <QTRSensors.h>
const int m11Pin = 7;
const int m12Pin = 6;
const int m21Pin = 5;
const int m22Pin = 4;

const int m1Enable = 11;
const int m2Enable = 10;

int m1Speed = 0;
int m2Speed = 0;

// increase kp’s value and see what happenss
float kp = 40;
float ki = 0;
float kd = 90;

int p = 1;
int i = 0;
int d = 0;

int error = 0;
int lastError = 0;

const int minError = -10;
const int maxError = 10;

const int minCalibError = -50;
const int maxCalibError = 50;

const int maxSpeed = 255;
const int minSpeed = -255;
const int baseSpeed = 200;

QTRSensors qtr;

const int sensorCount = 6;
int sensorValues[sensorCount];
int sensors[sensorCount] = { 0, 0, 0, 0, 0, 0 };
void setup() {
  // pinMode setup
  pinMode(m11Pin, OUTPUT);
  pinMode(m12Pin, OUTPUT);
  pinMode(m21Pin, OUTPUT);
  pinMode(m22Pin, OUTPUT);
  pinMode(m1Enable, OUTPUT);
  pinMode(m2Enable, OUTPUT);

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){ A0, A1, A2, A3, A4, A5 },
                    sensorCount);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  // turn on Arduino's LED to indicate we are in calibration mode

  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
    int error = map(qtr.readLineBlack(sensorValues), 0, 5000, minCalibError, maxCalibError);

    if (error < minCalibError + 1) {
      setMotorSpeed(minSpeed, maxSpeed);
    } else if (error > maxCalibError - 1) {
      setMotorSpeed(maxSpeed, minSpeed);
    }
  }

  digitalWrite(LED_BUILTIN, LOW);
  Serial.begin(9600);
}

void loop() {
  // inefficient code, written in loop. You must create separate functions
  int error = map(qtr.readLineBlack(sensorValues), 0, 5000, minError, maxError);

  p = error;
  i = i + error;
  d = error - lastError;

  int motorSpeed = kp * p + ki * i + kd * d;  // = error in this case

  m1Speed = baseSpeed;
  m2Speed = baseSpeed;
  // a bit counter intuitive because of the signs
  // basically in the first if, you substract the error from m1Speed (you add the negative)
  // in the 2nd if you add the error to m2Speed (you substract thenegative)
  // it's just the way the values of the sensors and/or motors lined up

  if (error < 0) {
    if (error > -3) {
      kd = 0;
      kp = 60;
      m1Speed += motorSpeed;
    } else {
      kd = 40;
      kp = 90;
      m2Speed -= motorSpeed;
      m1Speed += motorSpeed;
    }

  } else if (error > 0) {
    if (error < 3) {
      kd = 0;
      kp = 60;
      m1Speed += motorSpeed;
    } else {
      kd = 40;
      kp = 90;
      m2Speed -= motorSpeed;
      m1Speed += motorSpeed;
    }
  }
  // make sure it doesn't go past limits. You can use -255 instead of 0 ifcalibrated programmed properly.
  // making sure we don't go out of bounds
  // maybe the lower bound should be negative, instead of 0? This of what happens when making a steep turn
  m1Speed = constrain(m1Speed, -75, maxSpeed);
  m2Speed = constrain(m2Speed, -75, maxSpeed);
  setMotorSpeed(m1Speed, m2Speed);
  // DEBUGGING
  // Serial.print("Error: ");
  // Serial.println(error);
  // Serial.print("M1 speed: ");
  // Serial.println(m1Speed);
  //
  // Serial.print("M2 speed: ");
  // Serial.println(m2Speed);
  //
  // delay(250);
}

int calculateError() {
  return map(qtr.readLineBlack(sensorValues), 0, 5000, minError, maxError);
}

// calculate PID value based on error, kp, kd, ki, p, i and d.
void pidControl(float kp, float ki, float kd) {
  // TODO
}
// each arguments takes values between -255 and 255. The negative values represent the motor speed in reverse.
void setMotorSpeed(int motor1Speed, int motor2Speed) {
  // remove comment if any of the motors are going in reverse
  // motor1Speed = -motor1Speed;
  // motor2Speed = -motor2Speed;
  if (motor1Speed == 0) {
    digitalWrite(m11Pin, LOW);
    digitalWrite(m12Pin, LOW);
    analogWrite(m1Enable, motor1Speed);
  } else {
    if (motor1Speed > 0) {
      digitalWrite(m11Pin, HIGH);
      digitalWrite(m12Pin, LOW);
      analogWrite(m1Enable, motor1Speed);
    }
    if (motor1Speed < 0) {
      digitalWrite(m11Pin, LOW);
      digitalWrite(m12Pin, HIGH);
      analogWrite(m1Enable, -motor1Speed);
    }
  }
  if (motor2Speed == 0) {
    digitalWrite(m21Pin, LOW);
    digitalWrite(m22Pin, LOW);
    analogWrite(m2Enable, motor2Speed);
  } else {
    if (motor2Speed > 0) {
      digitalWrite(m21Pin, HIGH);
      digitalWrite(m22Pin, LOW);
      analogWrite(m2Enable, motor2Speed);
    }
    if (motor2Speed < 0) {
      digitalWrite(m21Pin, LOW);
      digitalWrite(m22Pin, HIGH);
      analogWrite(m2Enable, -motor2Speed);
    }
  }
}