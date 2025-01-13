/*
    SwerveModule.h 
 swerve module udirdlagin hesg
*/
#ifndef SWERVEMODULE_H
#define SWERVEMODULE_H

#include <iostream>
#include <string>
#include <cmath>
// #include "Arduino.h"
#include "IYawEncoder.h"
#include "IMotor.h"
#include "MotorSpeedController.h"

class SwerveModule {
    private:
        // Gear ratio for yaw. This may not include all pairs of gears.
        double             GEAR_RATIO_YAW;

        // Gear ratio for wheel speed. Typically, this includes all pairs of gears.
        double             GEAR_RATIO_WHEEL_SPEED;

        // Maximum yaw speed in RPM
        double            MAX_YAW_SPEED_RPM;

        // Wheel diameter in meters
        double            WHEEL_DIAMETER_METERS;
        
        // Devices, Sensors, Actuators
        IMotor* m_driveMotorA;
        IMotor* m_driveMotorB;
        IYawEncoder* m_yawEncoder;
        
        // Shuffleboard-ROS node name ntr geed data ywuulhad ashiglah bhda
        std::string       name;
        std::string       shuffleboardTabName;
        // String            name;
        // String            shuffleboardTabName;

        // Block periodic until we're initialized
        bool           bInitialized = false;   

    public:
        // Desired state -- velocity of wheel in RPM, angle in degrees
        // Angles are measured counter-clockwise, with zero being "robot forward"
        double             desiredWheelSpeedRPM = 0.0;
        double             desiredYawDegrees = 0.0;

        // Constructor to initialize motor and encoder
        SwerveModule();

        void initialize(double gearRatioYaw, double gearRatioWheelSpeed, double maxYawSpeedRpm,
                        IMotor* motorA, IMotor* motorB, IYawEncoder* encoder, double wheelDiameterMeters);

        // Converts desired wheel speed (in m/s) to RPM
        double mpsToRpm(double speedMPS);

        // Set the desired module state
        void setModuleState(double speedMetersPerSecond, double angleDegrees);

        // Set motor speeds for both motors
        void setMotorSpeedsRPM(double wheelRPM, double yawRPM);

        // Periodic update for motor control
        void periodic();
};

#endif

/*
#include "IMotor.h"
#include <iostream>

class MotorController : public IMotor {
private:
    double currentRPM;
    double goalRPM;

public:
    MotorController() : currentRPM(0.0), goalRPM(0.0) {}

    void periodic() override {
        // Example: Adjust currentRPM towards goalRPM
        if (currentRPM < goalRPM) {
            currentRPM += 1.0; // Simple simulation of motor acceleration
        } else if (currentRPM > goalRPM) {
            currentRPM -= 1.0; // Simple simulation of motor deceleration
        }
        std::cout << "Current RPM: " << currentRPM << std::endl;
    }

    void setGoalRPM(double goalRPM) override {
        this->goalRPM = goalRPM;
    }

    double getVelocityRPM() const override {
        return currentRPM;
    }
};

#include "MotorController.h"
#include <vector>
#include <memory>

int main() {
    // Create multiple motors
    std::vector<std::unique_ptr<IMotor>> motors;

    // Add motors to the collection
    motors.push_back(std::make_unique<MotorController>());
    motors.push_back(std::make_unique<MotorController>());
    motors.push_back(std::make_unique<MotorController>());

    // Set individual RPM goals
    motors[0]->setGoalRPM(100.0);
    motors[1]->setGoalRPM(200.0);
    motors[2]->setGoalRPM(150.0);

    // Simulate periodic updates
    for (int i = 0; i < 10; ++i) {
        std::cout << "Periodic Update " << i + 1 << ":" << std::endl;
        for (auto& motor : motors) {
            motor->periodic();
        }
        std::cout << std::endl;
    }

    // Print final velocities
    for (size_t i = 0; i < motors.size(); ++i) {
        std::cout << "Motor " << i + 1 << " Final Velocity: " 
                  << motors[i]->getVelocityRPM() << " RPM" << std::endl;
    }

    return 0;
}

int main() {
    // Create instances of Motor
    Motor motorA;
    Motor motorB;

    // Create a dummy encoder (you'll need a real implementation)
    class DummyYawEncoder : public IYawEncoder {
    public:
        void setZero() override {}
        double getOutputSignedPercent(double targetAngle) const override { return 0.5; } // Dummy behavior
        void periodic() override {}
    };
    DummyYawEncoder yawEncoder;

    // Create the SwerveModule and initialize it
    SwerveModule module;
    module.initialize(1.0, 1.0, 100.0, &motorA, &motorB, &yawEncoder, 0.1);

    // Set module state
    module.setModuleState(1.0, 90.0); // Set speed to 1 m/s and angle to 90 degrees

    // Simulate periodic updates
    for (int i = 0; i < 10; ++i) {
        module.periodic();
    }

    return 0;
}
*/
/*
Output
    Periodic Update 1:
    Motor RPM: 1
    Motor RPM: 1
    Motor RPM: 1

    Periodic Update 2:
    Motor RPM: 2
    Motor RPM: 2
    Motor RPM: 2
    ...

    Motor 1 Final Velocity: 10 RPM
    Motor 2 Final Velocity: 20 RPM
    Motor 3 Final Velocity: 15 RPM
*/