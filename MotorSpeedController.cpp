#include "MotorSpeedController.h"  // Include the header for the class definition

MotorSpeedController::MotorSpeedController()
    : kp(1), kd(0), ki(0), umax(255), eprev(0.0), eintegral(0.0), targetRPM(0) {}

void MotorSpeedController::setGoalRPM(double goalRPM) {
    targetRPM = goalRPM;
}

double MotorSpeedController::getVelocityRPM() {
    return currentRPM;  // Return current RPM (dummy value for now)
}

void MotorSpeedController::periodic() {
    // Periodic logic for updating motor state can go here
}

void MotorSpeedController::setParams(float kpIn, float kdIn, float kiIn, float umaxIn) {
    kp = kpIn;
    kd = kdIn;
    ki = kiIn;
    umax = umaxIn;
}

void MotorSpeedController::evalu(int value, int target, float deltaT, int &pwr, int &dir) {
    int e = target - value;  // Calculate error
    float dedt = (e - eprev) / deltaT;  // Calculate derivative
    eintegral = eintegral + e * deltaT;  // Calculate integral
    float u = kp * e + kd * dedt + ki * eintegral;  // Control signal

    pwr = (int)fabs(u);
    if (pwr > umax) {
        pwr = umax;  // Limit motor power to umax
    }

    dir = (u < 0) ? -1 : 1;  // Set direction based on control signal
    eprev = e;  // Store previous error for the next iteration
}
