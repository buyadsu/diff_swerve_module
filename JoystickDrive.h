#ifndef JOYSTICKDRIVE_H
#define JOYSTICKDRIVE_H

#include <Arduino.h>
#include <iostream>

// Define the joystick input structure
struct JoystickInput {
    float directionX;  // X-axis (-1.0 to 1.0)
    float directionY;  // Y-axis (-1.0 to 1.0)
    float rotation;    // Rotation (-1.0 to 1.0)
    bool grab;         // Grab button state
    bool throwAction;  // Throw button state
};

class JoystickDrive {
private:
    Stream &serialStream;

    const int SIGNAL_TIMEOUT = 1000;
    const unsigned long BAUDRATE = 115200;
    const float DEADBAND = 0.1;  // [0-1]
    const float MAXRATE = 0.5f;  // Maximum rate of change per second
    const float DELTATIME = 0.1f; // Time step (0.1 seconds)

    unsigned long lastRecvTime;

    // Slew Rate Limiters for joystick axes
    class SlewRateLimiter {
    public:
        SlewRateLimiter(float maxRate) : maxRate(maxRate), lastValue(0.0f) {}

        float update(float targetValue, float deltaTime) {
            float maxChange = maxRate * deltaTime;
            float delta = targetValue - lastValue;

            if (delta > maxChange) {
                delta = maxChange;
            } else if (delta < -maxChange) {
                delta = -maxChange;
            }

            lastValue += delta;
            return lastValue;
        }

    private:
        float maxRate;
        float lastValue;
    };

    SlewRateLimiter limiterX{MAXRATE};
    SlewRateLimiter limiterY{MAXRATE};
    SlewRateLimiter limiterR{MAXRATE};

    // Define the raw joystick input structure
    struct RawInput {
        int16_t directionX;
        int16_t directionY;
        int16_t rotation;
        uint16_t controls;
    };

    JoystickInput input;

public:
    JoystickDrive(Stream &stream) : serialStream(stream), lastRecvTime(0) {}

    void initJoystick() {
        serialStream.begin(BAUDRATE);
        serialStream.println("Joystick initialized and waiting for input...");
    }

    bool readData() {
        if (serialStream.available() >= sizeof(RawInput)) {
            RawInput raw;
            serialStream.readBytes((uint8_t*)&raw, sizeof(RawInput));

            // Convert raw data to normalized values
            input.directionX = (float)raw.directionX / 512.0f;
            input.directionY = (float)raw.directionY / -512.0f; // Y is inverted
            input.rotation = (float)raw.rotation / 512.0f;
            input.grab = (raw.controls & 0x01) != 0;
            input.throwAction = (raw.controls & 0x02) != 0;

            lastRecvTime = millis();
            return true;
        }
        return false;
    }

    void execute() {
        if (!readData()) {
            return;
        }

        float axisX = applyDeadband(input.directionX, DEADBAND);
        float axisY = applyDeadband(input.directionY, DEADBAND);
        float rot = applyDeadband(input.rotation, DEADBAND);

        axisX = limiterX.update(axisX, DELTATIME);
        axisY = limiterY.update(axisY, DELTATIME);
        rot = limiterR.update(rot, DELTATIME);

        // Debug output to verify joystick inputs
        serialStream.print("Driving with X: ");
        serialStream.print(axisX, 3);
        serialStream.print(", Y: ");
        serialStream.print(axisY, 3);
        serialStream.print(", Rotation: ");
        serialStream.println(rot, 3);

        // Replace with actual drive logic for your subsystem
        // m_subsystem.drive(axisX, axisY, rot);
    }

    void periodic() {
        if (millis() - lastRecvTime > SIGNAL_TIMEOUT) {
            serialStream.println("Joystick signal lost...");
            // Replace with actual stop logic for your subsystem
            // m_subsystem.drive(0, 0, 0);
        } else {
            execute();
        }
    }

    float applyDeadband(float value, float deadband) {
        if (fabs(value) > deadband) {
            return (value > 0 ? 1 : -1) * (fabs(value) - deadband) / (1.0f - deadband);
        } else {
            return 0.0f;
        }
    }
};

#endif
