#include "SwerveModule.h"   

// Constructor to initialize motor and encoder
SwerveModule::SwerveModule() : m_driveMotorA(nullptr), m_driveMotorB(nullptr), m_yawEncoder(nullptr),
                     bInitialized(false), desiredWheelSpeedRPM(0), desiredYawDegrees(0) {}

void SwerveModule::initialize(double gearRatioYaw, double gearRatioWheelSpeed, double maxYawSpeedRpm,
                    IMotor* motorA, IMotor* motorB, IYawEncoder* encoder, double wheelDiameterMeters) {
    m_driveMotorA = motorA;
    m_driveMotorB = motorB;
    m_yawEncoder = encoder;
    GEAR_RATIO_YAW = gearRatioYaw;
    GEAR_RATIO_WHEEL_SPEED = gearRatioWheelSpeed;
    MAX_YAW_SPEED_RPM = maxYawSpeedRpm;
    WHEEL_DIAMETER_METERS = wheelDiameterMeters;

    m_yawEncoder->setZero();
    bInitialized = true;
}

// Converts desired wheel speed (in m/s) to RPM
double SwerveModule::mpsToRpm(double speedMPS) {
    return (speedMPS * 60) / (M_PI * WHEEL_DIAMETER_METERS); // Using formula: RPM = (MPS * 60) / (Pi * Diameter)
}

// Set the desired module state
void SwerveModule::setModuleState(double speedMetersPerSecond, double angleDegrees) {
    desiredWheelSpeedRPM = mpsToRpm(speedMetersPerSecond);
    desiredYawDegrees = angleDegrees;
}

// Set motor speeds for both motors
void SwerveModule::setMotorSpeedsRPM(double wheelRPM, double yawRPM) {
    double aRPM = (yawRPM / GEAR_RATIO_YAW) + (wheelRPM / GEAR_RATIO_WHEEL_SPEED);
    double bRPM = (yawRPM / GEAR_RATIO_YAW) - (wheelRPM / GEAR_RATIO_WHEEL_SPEED);

    m_driveMotorA->setGoalRPM(aRPM);
    m_driveMotorB->setGoalRPM(bRPM);
}

// Periodic update for motor control
void SwerveModule::periodic() {
    if (!bInitialized) {
        return;
    }

    double calculatedYawRPM = m_yawEncoder->getOutputSignedPercent(desiredYawDegrees) * MAX_YAW_SPEED_RPM;
    setMotorSpeedsRPM(desiredWheelSpeedRPM, calculatedYawRPM);

    m_driveMotorA->periodic();
    m_driveMotorB->periodic();
    m_yawEncoder->periodic();
}