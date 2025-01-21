#ifndef ENCODERCONTROLLER_H
#define ENCODERCONTROLLER_H

#include <Encoder.h>

class EncoderController {
private:
    // Encoder pins
    int8_t yawENCApin;
    int8_t yawENCBpin;
    int8_t driveENCApin;
    int8_t driveENCBpin;

    // Encoder objects
    Encoder yawEncoder;
    Encoder driveEncoder;

    // Positions
    long yawPosition = 0;
    long drivePosition = 0;

public:
    // Constructor
    EncoderController(int8_t yawA, int8_t yawB, int8_t driveA, int8_t driveB)
        : yawENCApin(yawA), yawENCBpin(yawB),
          driveENCApin(driveA), driveENCBpin(driveB),
          yawEncoder(yawA, yawB), driveEncoder(driveA, driveB) {}

    // Initialize encoders
    void initialize() {
        Serial.begin(9600);
        Serial.println("Encoder Test Initialized");
    }

    // Reset both encoders to zero
    void setZero() {
        Serial.println("Resetting both encoders to zero...");
        yawEncoder.write(0);
        driveEncoder.write(0);
    }

    // Periodic update for position
    void periodic() {
        long newYawPosition = yawEncoder.read();
        long newDrivePosition = driveEncoder.read();

        if (newYawPosition != yawPosition || newDrivePosition != drivePosition) {
          /* Debug
            Serial.print("Yaw Position = ");
            Serial.print(newYawPosition);
            Serial.print(", Drive Position = ");
            Serial.println(newDrivePosition);
          */
            yawPosition = newYawPosition;
            drivePosition = newDrivePosition;
        }
    }

    // Get yaw position
    long getYawPosition() {
        return yawEncoder.read();
    }

    // Get drive position
    long getDrivePosition() {
        return driveEncoder.read();
    }
};

#endif
