
/*
 * IIWARobot.cpp
 *
 *  Created on: Mar 14, 2012
 *      Author: sina
 */

#include "IIWARobot.h"




void IIWARobot::SetEstimatedExternalCartForces(const Vector & cartEF){
/*  Not Supported yet
 *
 *  if(cartEF.Size()==NB_CART){
    EstimatedExternalCartForces= cartEF;
  }*/
}

void IIWARobot::SetCommandedJointTorques(Vector & jointT){
/* Not Supported yet
 *
 *  if(jointT.Size() == NB_JOINTS){
    CommandedJointTorques = jointT;
  }*/
}

void IIWARobot::SetCompleteIIWAState(const Vector& s){
  CompleteIIWAState = s;
}

Vector IIWARobot::GetCompleteIIWAState(){
  return CompleteIIWAState;
}

void IIWARobot::InitializeIIWAComponents(){


  CommandedJointTorques.Resize(NB_JOINTS);
  massMatrix.Resize(NB_JOINTS,NB_JOINTS);

  SelectedGravComp.Resize(7);
  SelectedGravComp.One();
  JointStiffness.Resize(NB_JOINTS);
  JointDamping.Resize(NB_JOINTS);

  EstimatedExternalJointTorques.Resize(NB_JOINTS);
  MeasuredJointTorques.Resize(NB_JOINTS);

  CompleteIIWAState.Resize(IIWA_state_size);

  for(int i=0;i<NB_JOINTS;i++){
    JointStiffness(i) = FRI_JOINT_STIFFNESS;
    JointDamping(i) = FRI_JOINT_DAMPING;
    CommandedJointTorques(i) = 0.;
  }

  for(int i=0;i<3;i++){
    CartStiffness(i) = FRI_CART_STIFFNESS_POS;
    CartStiffness(3+i) = FRI_CART_STIFFNESS_ORIENT;
    CartDamping(i) = FRI_CART_DAMPING_POS;
    CartDamping(3+i) = FRI_CART_DAMPING_ORIENT;
  }

}
