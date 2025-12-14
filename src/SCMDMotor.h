#ifndef SCMDMotor_h
#define SCMDMotor_h

#include <Wire.h>
#include <Arduino.h>
#include <SCMD.h>
#include <SCMD_config.h>

// Forward declarations
class SCMDMotor;
SCMDMotor* globalMotorInstance = nullptr;  // Global pointer for ISR
void encoderISR();                          // Forward declare ISR

class SCMDMotor {
public:
    uint8_t _encoderA;
    uint8_t _encoderB;
    volatile int32_t _encoderCount;

    SCMDMotor(uint8_t motorId, uint8_t encoderA, uint8_t encoderB)
        : _motorId(motorId), _encoderA(encoderA), _encoderB(encoderB),
          _encoderCount(0), _integral(0), _lastError(0),
          _kp(5.0), _ki(0.1), _kd(1.0),
          _maxIntegral(500.0), _positionTolerance(5), _minMotorPower(50),
          _isMoving(false), _targetPosition(0),
          _isHoming(false), _limitSwitchPin(0), _homeSpeed(0),
          _lastPIDUpdate(0), _pidInterval(5)
    {}

    void begin() {
        globalMotorInstance = this;

        Wire.setClock(400000);
        _driver.settings.commInterface = I2C_MODE;
        _driver.settings.I2CAddress = 0x5D;

        while (_driver.begin() != 0xA9) delay(100);
        while (!_driver.ready()) delay(5);
        _driver.enable();
        delay(20);

        pinMode(_encoderA, INPUT_PULLUP);
        pinMode(_encoderB, INPUT_PULLUP);

        attachInterrupt(digitalPinToInterrupt(_encoderA), encoderISR, CHANGE);
        attachInterrupt(digitalPinToInterrupt(_encoderB), encoderISR, CHANGE);
    }

    // --- Non-blocking movement ---
    void startMoveTo(int32_t target) {
        _targetPosition = target;
        _integral = 0;
        _lastError = 0;
        _isMoving = true;
    }

    void startHome(uint8_t limitSwitchPin, int16_t homeSpeed = 255) {
        _limitSwitchPin = limitSwitchPin;
        _homeSpeed = homeSpeed;
        pinMode(limitSwitchPin, INPUT_PULLDOWN);
        _isHoming = true;
    }

    // Must be called repeatedly in loop()
    void update() {
        unsigned long now = millis();

        // PID update for moveTo
        if (_isMoving && now - _lastPIDUpdate >= _pidInterval) {
            _lastPIDUpdate = now;

            int32_t current = _encoderCount;
            int16_t speed = calculatePID(current, _targetPosition);
            setMotor(speed);

            if (abs(_targetPosition - _encoderCount) <= _positionTolerance) {
                setMotor(0);
                _isMoving = false;
            }
        }

        // Homing update
        if (_isHoming) {
            if (digitalRead(_limitSwitchPin) == LOW) {
                setMotor(-abs(_homeSpeed)); // move in reverse
            } else {
                setMotor(0);
                _encoderCount = 0;
                _isHoming = false;
            }
        }
    }

    int32_t getPosition() { return _encoderCount; }
    void resetEncoder() { _encoderCount = 0; }

private:
    uint8_t _motorId;
    SCMD _driver;
    float _integral;
    int32_t _lastError;
    float _kp, _ki, _kd;
    float _maxIntegral;
    int32_t _positionTolerance;
    int16_t _minMotorPower;

    // Non-blocking movement state
    bool _isMoving;
    int32_t _targetPosition;

    bool _isHoming;
    uint8_t _limitSwitchPin;
    int16_t _homeSpeed;

    unsigned long _lastPIDUpdate;
    const unsigned long _pidInterval;

    void setMotor(int16_t speed) {
        if (speed > 255) speed = 255;
        if (speed < -255) speed = -255;

        if (speed > 0) {
            if (speed < _minMotorPower) speed = _minMotorPower;
            _driver.setDrive(_motorId, 1, speed);
        } else if (speed < 0) {
            if (speed > -_minMotorPower) speed = -_minMotorPower;
            _driver.setDrive(_motorId, 0, -speed);
        } else {
            _driver.setDrive(_motorId, 0, 0);
        }
    }

    int16_t calculatePID(int32_t current, int32_t target) {
        int32_t error = target - current;
        float p = _kp * error;

        _integral += error;
        if (_integral > _maxIntegral) _integral = _maxIntegral;
        if (_integral < -_maxIntegral) _integral = -_maxIntegral;
        float i = _ki * _integral;

        int32_t derivative = error - _lastError;
        float d = _kd * derivative;
        _lastError = error;

        float output = p + i + d;

        if (abs(error) < _positionTolerance) {
            _integral = 0;
            return 0;
        }

        if (output > 255) output = 255;
        if (output < -255) output = -255;

        return (int16_t)output;
    }
};

// --- Global ISR function ---
void encoderISR() {
    if (!globalMotorInstance) return;

    static int lastEncoded = 0;
    int a = digitalRead(globalMotorInstance->_encoderA);
    int b = digitalRead(globalMotorInstance->_encoderB);
    int encoded = (a << 1) | b;
    int sum = (lastEncoded << 2) | encoded;

    if (sum == 0b0001 || sum == 0b0111 || sum == 0b1110 || sum == 0b1000)
        globalMotorInstance->_encoderCount++;
    if (sum == 0b0010 || sum == 0b1011 || sum == 0b1101 || sum == 0b0100)
        globalMotorInstance->_encoderCount--;

    lastEncoded = encoded;
}

#endif
