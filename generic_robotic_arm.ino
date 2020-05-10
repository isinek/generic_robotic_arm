/* ********************
 * Generic robotic arm
 * ********************
 */

#define debug 0

#include <Servo.h>

struct Sensor {
  uint8_t pin;
  uint16_t minVal;
  uint16_t maxVal;
};

class Motor {
  private:
    Servo servo;
    uint8_t pin;
    uint8_t speedDg;
    uint16_t newPositionDg = 0;
    struct Sensor *sensor = NULL;

  public:
    Motor(uint8_t pin, uint16_t startPosition = 0, uint8_t speedDg = 5) {
      this->pin = pin;
      this->servo.attach(pin);
      this->speedDg = speedDg;
      this->servo.write(startPosition);
    }

    void setPosition(uint16_t newPositionDg) {
      this->newPositionDg = newPositionDg > 180 ? 180 : newPositionDg;
#if debug
  Serial.print("New position:\t");
  Serial.print(this->newPositionDg);
  Serial.println();
#endif
    }

    void updatePosition() {
      uint16_t currPos = this->servo.read();
      uint16_t newPos = 0;
      if (this->sensor)
        this->newPositionDg = map(analogRead(this->sensor->pin), this->sensor->minVal, this->sensor->maxVal, 0, 180);
      if (currPos < this->newPositionDg) {
        newPos = currPos + this->speedDg > this->newPositionDg ? this->newPositionDg: currPos + this->speedDg;
        this->servo.write(newPos);
      } else if (currPos > this->newPositionDg) {
        newPos = currPos - this->speedDg < this->newPositionDg ? this->newPositionDg: currPos - this->speedDg;
        this->servo.write(newPos);
      }
#if debug
  Serial.print(this->pin);
  Serial.print("\t");
  Serial.print(this->servo.read());
  Serial.print("\t");
  Serial.print(this->newPositionDg);
  Serial.println();
#endif
    }

    uint16_t getPosition() {
      return this->servo.read();
    }

    void linkSensor(uint8_t sensorPin, uint16_t minVal, uint16_t maxVal) {
      if (this->sensor)
        delete(this->sensor);
      this->sensor = new struct Sensor;
      this->sensor->pin = sensorPin;
      this->sensor->minVal = minVal;
      this->sensor->maxVal = maxVal;
    }
};

class GenericRoboticArm {
  private:
    uint8_t noMotors;
    Motor **motors = NULL;
    uint16_t updateDelay = 10;

  public:
    GenericRoboticArm(uint8_t noMotors, uint8_t *motors) {
      this->noMotors = noMotors;
      this->motors = new Motor*[noMotors];
      for (uint8_t i = 0; i < noMotors; ++i)
        this->motors[i] = new Motor(motors[i], 0, 15);
      delay(1000);
    }

    ~GenericRoboticArm() {
      free(this->motors);
    }

    void linkSensor(uint8_t motorId, uint8_t sensorPin, uint16_t minVal = 0, uint16_t maxVal = 1023) {
      this->motors[motorId]->linkSensor(sensorPin, minVal, maxVal);
    }

    void updatePosition() {
      for (uint8_t i = 0; i < noMotors; ++i)
        this->motors[i]->updatePosition();
      delay(updateDelay);
    }
};

GenericRoboticArm *arm = NULL;
uint16_t dg = 0;

void setup() {
  uint8_t motors[] = { 3 };
  arm = new GenericRoboticArm(1, motors);
  arm->linkSensor(0, A1);

#if debug
  Serial.begin(9600);
#endif
}

void loop() {
  arm->updatePosition();
}
