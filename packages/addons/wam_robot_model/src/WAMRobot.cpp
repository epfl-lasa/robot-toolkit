#include "WAMRobot.h"

WAMRobot::WAMRobot():Robot::Robot(false)
{
    HC = HANDIDLE;
    Vector3 temp3;
    temp3 *=0;
    ForceSensorSpatialForce.SetLinearComponent(temp3);
    ForceSensorSpatialForce.SetAngularComponent(temp3);
}

void WAMRobot::GetSensedEEForce(Vector3 &res)
{
    res = ForceSensorSpatialForce.GetLinearComponent();
}

void WAMRobot::GetSensedEETorque(Vector3 &res)
{
    res = ForceSensorSpatialForce.GetAngularComponent();
}

void WAMRobot::GetSpatialForce(SpatialForce &res)
{
    res = ForceSensorSpatialForce;
}

void WAMRobot::SetSensedEEForce(double x,double y,double z)
{
    ForceSensorSpatialForce.SetLinearComponent(x,y,z);
}

void WAMRobot::SetSensedEETorque(double wx,double wy,double wz)
{
    ForceSensorSpatialForce.SetAngularComponent(wx,wy,wz);
}

void WAMRobot::HandOpen()
{
    HC = HANDOPEN;
}

void WAMRobot::HandClose()
{
    HC = HANDCLOSE;
}

void WAMRobot::HandInit()
{
    HC = HANDINIT;
}

HandCommand WAMRobot::GetHandState()
{
    return HC;
}
