#include "TorqueChecker.h"

TorqueChecker::TorqueChecker()
{

    TorqueLimits.Resize(7);
    TorqueLimits(0)=15;
    TorqueLimits(1)=25;
    TorqueLimits(2)=8;
    TorqueLimits(3)=8;
    TorqueLimits(4)=2;
    TorqueLimits(5)=3;
    TorqueLimits(6)=2;
    TorqueLimits*=5;

    Joint2MotorTorque.Resize(7,7);
    Joint2MotorTorque(0,0) = -0.0238095;
    Joint2MotorTorque(1,1) =  0.0176991;Joint2MotorTorque(1,2) = -0.0297345;
    Joint2MotorTorque(2,1) = -0.0176991;Joint2MotorTorque(2,2) = -0.0297345;
    Joint2MotorTorque(3,3) = -0.0555556;
    Joint2MotorTorque(4,4) =  0.0527426;Joint2MotorTorque(4,5) = -0.0527426;
    Joint2MotorTorque(5,4) =  0.0527426;Joint2MotorTorque(5,5) =  0.0527426;
    Joint2MotorTorque(6,6) = -0.0669792;

    MotorTorque.Resize(7);
    MotorTorque(0) = 4860;
    MotorTorque(1) = 4860;
    MotorTorque(2) = 4860;
    MotorTorque(3) = 4320;
    MotorTorque(4) = 3900;
    MotorTorque(5) = 3900;
    MotorTorque(6) = 3200;

    CableLimits.Resize(7);
    CableLimits(0) = 1.8;
    CableLimits(1) = 1.8;
    CableLimits(2) = 1.8;
    CableLimits(3) = 1.6;
    CableLimits(4) = 0.6;
    CableLimits(5) = 0.6;
    CableLimits(6) = 0.1813;

    CL2MT.Resize(7);
    CL2MT = CableLimits/MotorTorque;

}



int TorqueChecker::CheckTorqueValue(Vector & mJointTorques){
  Joint2MotorTorque.Mult(mJointTorques,MotorTorque);
  int result = -1;
  for (int i=0; i<7; i++){
    if (fabs(MotorTorque[i]) > 5500*CL2MT[i]){
      result = i+1;
      break;
    }
  }
  return result;
}
