#ifndef WAMROBOT_H
#define WAMROBOT_H

#include "Robot.h"
#include "ros/ros.h"


typedef enum{HANDCLOSE,HANDOPEN,HANDINIT,HANDIDLE} HandCommand;

class WAMRobot : public Robot
{
    //WAM-specific stuff
    SpatialForce ForceSensorSpatialForce;
    HandCommand HC;



public:
    WAMRobot();
    void GetSensedEEForce(Vector3 & res);
    void GetSensedEETorque(Vector3 & res);
    void GetSpatialForce(SpatialForce & res);
    void SetSensedEEForce(double x, double y, double z);
    void SetSensedEETorque(double wx,double wy,double wz);

    void HandOpen();
    void HandClose();
    void HandInit();
    HandCommand GetHandState();
};

#endif // WAMROBOT_H
