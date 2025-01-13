#ifndef IMOTOR_H
#define IMOTOR_H

/**
 * This is the interface expected to be implemented for a motor/motor
 * controller to be used with Bionic Swerve. Each motor is expected to
 * implement a PID that keeps the motor at a specified setpoint.
 *
 * Many motor controllers have internal PIDs, which may be used. If the motor
 * controller does not have internal PIDs, the implementing class should
 * provide a software-based PID to maintain constant motor speed.
 *
 * The constructor might include the following:
 * - Instantiate an appropriate motor controller instance
 * - Set factory defaults
 * - Set initial motor PID values
 */
class IMotor {
public:
    /**
     * Virtual destructor to ensure proper cleanup of derived objects.
     */
    virtual ~IMotor() {}

    /**
     * Function that should be called periodically, typically from the using
     * swerve module's periodic function.
     */
    virtual void periodic() = 0; // no-op

    /**
     * Set the RPM goal, in revolutions per minute. The implementing class is
     * expected to cause the motor to maintain this RPM, through the use of
     * PIDs or similar mechanism.
     *
     * @param goalRPM The requested RPM to be maintained
     */
    virtual void setGoalRPM(double goalRPM) = 0;

    /**
     * Get the current encoder-ascertained velocity of the motor, in RPM.
     *
     * @return The current motor velocity in RPM
     */
    virtual double getVelocityRPM() = 0;  
};

#endif // IMOTOR_H
