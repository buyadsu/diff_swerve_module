#ifndef IYAWENCODER_H
#define IYAWENCODER_H

class IYawEncoder {
public:
    virtual ~IYawEncoder() {}
    virtual void setZero() = 0;
    virtual double getOutputSignedPercent(double targetAngle) const = 0;
    virtual void periodic() {}
};

#endif // IYAWENCODER_H