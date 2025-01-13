#ifndef MOTOR_SPEED_CONTROLLER_H
#define MOTOR_SPEED_CONTROLLER_H

#include <cmath> 
#include "IMotor.h"  // Include the base class

class MotorSpeedController : public IMotor {
private:
    float kp, kd, ki, umax;     // PID parameters
    float eprev, eintegral;     // Storage for previous error and integral
    double targetRPM;           // Target RPM for the motor
    double currentRPM = 500.0;  // Current RPM (dummy value)

public:
    MotorSpeedController();  // Constructor
    void setGoalRPM(double goalRPM) override;  // Set target RPM
    double getVelocityRPM() override;          // Get current RPM
    void periodic() override;                  // Periodic logic (empty for now)
    void setParams(float kpIn, float kdIn, float kiIn, float umaxIn);  // Set PID parameters
    void evalu(int value, int target, float deltaT, int &pwr, int &dir);  // Compute control signal
};

#endif  // MOTOR_SPEED_CONTROLLER_H
