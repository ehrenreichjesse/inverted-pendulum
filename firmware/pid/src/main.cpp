#include <Arduino.h>
#include <ESP32Encoder.h>
#include <Bluepad32.h>

ESP32Encoder pendulumEncoder;
ESP32Encoder linearEncoder;

float power;
float KpAngle = 1720;
float KdAngle = 172; 
float KiAngle = 0; 
float KpLinear = 15;
float KdLinear = 11; 

float k = -0.15;

float PIDcritAngle = 0.52;
float swingUpCritAngle = 1;

enum State {
  STARTUP,
  SWINGUP,
  BALANCE,
  STOPPED
};

State state = STARTUP;

void driveMotor(int power) {
  power = constrain(power, -255, 255);

  if (power > 0) {
    ledcWrite(0, 0);
    delayMicroseconds(250);
    ledcWrite(1, abs(power));
  }
  else if (power < 0) {
    ledcWrite(1, 0);
    delayMicroseconds(250);
    ledcWrite(0, abs(power));
  }
   else {
    ledcWrite(0, 0);
    ledcWrite(1, 0);
  }
}

float getAngle(void) {
  long count = pendulumEncoder.getCount();

  float theta = count/381.99;

  while (theta >= 3.1415) {
    theta -= 6.283;
  }
  while (theta < -3.1415) {
    theta += 6.283;
  }

  return theta;
}

float getAngularVelocity(void) {
  static unsigned long lastTime = 0;
  static float lastCount = 0;
  static float velocity = 0;

  unsigned long currentTime = millis();
  unsigned long deltaTime = currentTime - lastTime;

  if (deltaTime >= 20) {
    float currentCount = pendulumEncoder.getCount();
    float deltaCount = currentCount - lastCount;

    float deltaAngle = deltaCount/381.99;
    velocity = deltaAngle/deltaTime*1000.0;

    lastCount = currentCount;
    lastTime = currentTime;
  }

  return velocity;
}

float getFilteredAngularVelocity(void) {
  static float filteredAngularVelocity = 0;
  filteredAngularVelocity = 0.75*filteredAngularVelocity + 0.25*getAngularVelocity();

  return filteredAngularVelocity;
}

float getAngularIntegral(void) {
  static unsigned long lastTime = 0;
  static float integral = 0;

  unsigned long currentTime = millis()/1000.0;
  unsigned long deltaTime = currentTime - lastTime;

  if (deltaTime > 0.1) {
    deltaTime = 0.001;
  }

  float currentAngle = getAngle();
  integral += currentAngle*deltaTime;

  lastTime = currentTime;

  return integral;
}

float getPosition(void) {
  float x = -linearEncoder.getCount()/159.2;

  return x;
}

float getLinearVelocity(void) {
  static unsigned long lastTime = 0;
  static float lastAngle = 0;
  static float velocity = 0;

  unsigned long currentTime = millis();
  unsigned long deltaTime = currentTime - lastTime;

  if (deltaTime >= 20) {
    float currentAngle = getPosition();
    float deltaAngle = currentAngle - lastAngle;

    velocity = deltaAngle/deltaTime*1000.0;

    lastAngle = currentAngle;
    lastTime = currentTime;
  }

  return velocity;
}

void setup() {
  Serial.begin(115200);

  // enable pull up resistors
  ESP32Encoder::useInternalWeakPullResistors = puType::up;

  // initiate both encoders
  pendulumEncoder.attachFullQuad(33, 32);
  linearEncoder.attachFullQuad(16, 17);
  
  // encoder calibration
  pendulumEncoder.setCount(1200);
  linearEncoder.setCount(0);

  // set up motor pins
  ledcSetup(0, 20000, 8);
  ledcSetup(1, 20000, 8);
  ledcAttachPin(26, 0);
  ledcAttachPin(25, 1);

  delay(3000);
}

void loop() {
  bool positionOK = abs(getPosition()) < 15;

  float theta = getAngle();
  float omega = getAngularVelocity();

  if (positionOK) {
    switch(state) {
      case STARTUP:
        if (omega > 0) {
          power = -190*cos(theta);
        }
        else {
          power = 190*cos(theta);
        }

        if (abs(theta) < swingUpCritAngle) {
          state = SWINGUP;
        }

        break;
    
      case SWINGUP: {
        float energy = 0.5f*omega*omega + (1 + cos(theta));
        float energyError = energy - 2.0f;
        power = k*energyError*omega*cos(theta);

        if (abs(theta) < PIDcritAngle) {
          state = BALANCE;
        }

        break;
      }

      case BALANCE:
        power = KpAngle*getAngle() + KdAngle*getAngularVelocity() + KiAngle*getAngularIntegral() + KpLinear*getPosition() + KdLinear*getLinearVelocity();

        if (abs(theta) > PIDcritAngle) {
          state = STOPPED;
        }

        break;

      case STOPPED:
        power = 0;

        break;
    }
  }
  else {
    power = 0;
  }

  Serial.print(power);
  Serial.print("   ");
  Serial.print(theta);
  Serial.print("   ");
  Serial.println(omega);
  driveMotor(power);
  delay(2);
}