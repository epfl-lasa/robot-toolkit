#ifndef TORQUECHECKER_H
#define TORQUECHECKER_H


#include "MathLib/Vector.h"

#include "MathLib/Matrix.h"
using namespace MathLib;

/**
  This is a little class that can be used to check if a torque vector will yield torque error on the WAM robot. It is useful to check torques before sending them, and if
    it turns out that exessive torque has been computed, the robot can be put in gravity compensation or similar safe mode.
  */

class TorqueChecker
{
    Vector TorqueLimits;
    Vector CableLimits;
    Vector MotorTorque;
    Vector CL2MT;
    Matrix Joint2MotorTorque;

public:
    /**
    constructor
    */
    TorqueChecker();
    /**
    The checking function. This guy returns -1 if the torque vector is ok, and returns the ID of the violating motor if the torque error is detected.
    */
    int CheckTorqueValue(Vector &mJointTorques);
};

#endif // TORQUECHECKER_H
