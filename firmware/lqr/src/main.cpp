#include <AS5600.h>
#include <Encoder.h>
#include <math.h>
#include <BasicLinearAlgebra.h>

using namespace BLA;

AS5600 as5600;
Encoder motorEnc(2, 3);

float power;
int critAngle = 20; // angle which system starts
Matrix <1, 4> K = {20, -5, 222.6, 10}; // from python simulation!!!

float getSensorAngle(void) {
  float offset = 119;
  float raw = as5600.getCumulativePosition();
  float scaled = raw/11.375 + offset;

  return scaled;
}

float getMotorAngle(void) {
  float offset = 0;
  float angle = motorEnc.read()/2797.0*-360.0 + offset;

  return angle;
}

float getMotorVelocity(void) {
  static unsigned long lastTime = 0;
  static float lastAngle = 0;
  static float velocity = 0;

  unsigned long currentTime = millis();
  unsigned long deltaTime = currentTime - lastTime;

  if (deltaTime >= 20) {
    float currentAngle = getMotorAngle();
    float deltaAngle = currentAngle - lastAngle;

    velocity = deltaAngle/deltaTime*1000.0;

    lastAngle = currentAngle;
    lastTime = currentTime;
  }

  return velocity;
}

float getSensorVelocity(void) {
  static unsigned long lastTime = 0;
  static float lastAngle = 0;
  static float velocity = 0;

  unsigned long currentTime = millis();
  unsigned long deltaTime = currentTime - lastTime;

  if (deltaTime >= 20) {
    float currentAngle = getSensorAngle();
    float deltaAngle = currentAngle - lastAngle;

    velocity = deltaAngle/deltaTime*1000.0;

    lastAngle = currentAngle;
    lastTime = currentTime;
  }

  return velocity;
}

void driveMotor(int power) {
  power = constrain(power, -255, 255);

  if (power > 0) {
    analogWrite(11, 0);
    delayMicroseconds(250);
    analogWrite(10, power);
  }
  else if (power < 0) {
    analogWrite(10, 0);
    delayMicroseconds(250);
    analogWrite(11, abs(power));
  }
   else {
    analogWrite(10, 0);
    analogWrite(11, 0);
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);

}

void loop() {
  if (getSensorAngle() < critAngle && getSensorAngle() > -critAngle) {
    Matrix <4, 1> x = {getMotorAngle(), getMotorVelocity(), getSensorAngle(), getSensorVelocity()}; // state vector calculation
    power = -0.03*(K*x)(0, 0); // u = -Kx
  } else {
    // keyboard controls
    char key = Serial.read();

    if (key == 'a') {
      driveMotor(20);
      delay(500);
    }
    else if (key == 'd') {
      driveMotor(-20);
      delay(500);
    } 
    else {
      power = 0;
    }
  }

  driveMotor(power);

  Serial.print("Sensor: ");
  Serial.print(getSensorAngle());
  Serial.print("   ");
  Serial.println(power);
}