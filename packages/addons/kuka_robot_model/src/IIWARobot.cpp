
/*
 * IIWARobot.cpp
 *
 *  Created on: Mar 14, 2012
 *      Author: sina
 */

#include "IIWARobot.h"

IIWARobot::IIWARobot():Robot::Robot(false){


  InitializeIIWAComponents();


}

IIWARobot::~IIWARobot() {
  // TODO Auto-generated destructor stub
}



float * IIWARobot::GetCartCommandAsFloat(){

  float * float_cartCommand;
  float_cartCommand = (float*)malloc(12*sizeof(float));
  for(int i=0;i<3;i++){
    float_cartCommand[i*4+3] = CommandedCartPosition(i);
  }
  for(int i=0;i<3;i++){
    for(int j=0;j<3;j++){
      float_cartCommand[i*4+j] = CommandedCartOrientation(i,j);
    }
  }


  // float_cartCommand is ok here

  return float_cartCommand;
}

void IIWARobot::SetCartCommand(const Vector3 & cartCommandPos, const Matrix3 & cartCommandOrient) {
  CommandedCartPosition = cartCommandPos;
  CommandedCartOrientation = cartCommandOrient;
}

void IIWARobot::SetCartCommand(const Matrix4 & cartCommandHomMatrix){
  CommandedCartPosition = cartCommandHomMatrix.GetTranslation();
  CommandedCartOrientation = cartCommandHomMatrix.GetOrientation();
}

Vector IIWARobot::GetCartDamping() {
  return CartDamping;
}

void IIWARobot::SetCartDamping(const Vector & cartDamping) {
  CartDamping = cartDamping;
}

Vector IIWARobot::GetCartStiffness() {
  return CartStiffness;
}

void IIWARobot::SetCartStiffness(const Vector & cartStiffness) {
  CartStiffness = cartStiffness;
}

Vector IIWARobot::GetJointDamping() {
  return JointDamping;
}

void IIWARobot::SetJointDamping(const Vector & jointDamping) {
  JointDamping = jointDamping;
}

Vector IIWARobot::GetJointStiffness()  {
  return JointStiffness;
}

void IIWARobot::SetJointStiffness(const Vector & jointStiffness) {
  JointStiffness = jointStiffness;
}

void IIWARobot::SetAlive(bool bb){
  bAlive = bb;
}

bool IIWARobot::IsAlive(){
  return bAlive;
}

void IIWARobot::SetEstimatedExternalJointTorques(const Vector & jointEJT){
  if(jointEJT.Size()==NB_JOINTS){
    EstimatedExternalJointTorques = jointEJT;
  }
}
void IIWARobot::SetEstimatedExternalCartForces(const Vector & cartEF){
/*  Not Supported yet
 *
 *  if(cartEF.Size()==NB_CART){
    EstimatedExternalCartForces= cartEF;
  }*/
}

void IIWARobot::SetMeasuredJointTorques(const Vector & jointT){
  if(jointT.Size()==NB_JOINTS){
    MeasuredJointTorques = jointT;
  }
}
void IIWARobot::SetCommandedJointTorques(Vector & jointT){
/* Not Supported yet
 *
 *  if(jointT.Size() == NB_JOINTS){
    CommandedJointTorques = jointT;
  }*/
}

Vector IIWARobot::GetEstimatedExternalJointTorques(){
  return EstimatedExternalJointTorques;
}
Vector IIWARobot::GetMeasuredJointTorques(){
  return MeasuredJointTorques;
}
Vector IIWARobot::GetEstimatedExternalCartForces(){
  return EstimatedExternalCartForces;
}
Vector IIWARobot::GetCommandedJointTorques(){
  return CommandedJointTorques;
}
void IIWARobot::SetCompleteIIWAState(const Vector& s){
  CompleteIIWAState = s;
}

Vector IIWARobot::GetCompleteIIWAState(){
  return CompleteIIWAState;
}

void IIWARobot::SetMassMatrix(float** massMatrixPointer)
{
  for (int i=0; i<NB_JOINTS; i++)
    for (int j=0;j<NB_JOINTS;j++)
      massMatrix(i,j) = massMatrixPointer[i][j];
}

Matrix IIWARobot::GetMassMatrix()
{
  return massMatrix;
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

void IIWARobot::GetMeasuredCartPose(Vector3 & resultPos, Matrix3 & resultMat)
{
  resultPos = MeasuredCartPosition;
  resultMat = MeasuredCartOrientation;
}

float IIWARobot::GetSamplingTime() //s
{
  float current_time = MeasuredSamplingTime;
  return current_time;
}

void IIWARobot::SetSamplingTime(float measured_samp_time) //s
{
  MeasuredSamplingTime=measured_samp_time;
}

void IIWARobot::SetMeasuredCartPose(float * float_cartMeasured)
{
  for(int i=0;i<3;i++){
    MeasuredCartPosition(i) = float_cartMeasured[i*4+3];
  }
  for(int i=0;i<3;i++){
    for(int j=0;j<3;j++){
      MeasuredCartOrientation(i,j)= float_cartMeasured[i*4+j];
    }
  }
}


float * IIWARobot::GetCartStiffnessAsFloat()
{
  float * float_cartStiffness;
  float_cartStiffness = (float*)malloc(6*sizeof(float));
  for(int i=0;i<6;i++){
    float_cartStiffness[i] = CartStiffness(i);
  }

  // ok here.

  return float_cartStiffness;
}

float * IIWARobot::GetCartDampingAsFloat()
{
  float * float_cartDamping;
  float_cartDamping = (float*)malloc(6*sizeof(float));
  for(int i=0;i<6;i++){
    float_cartDamping[i] = CartDamping(i);
  }
  return float_cartDamping;
}

void IIWARobot::SetCartForce(const Vector & desForce)
{
  CartDesiredForce = desForce;

  //cout<<"CartDesidredForceeeeeee";
  //CartDesiredForce.Print();

}

float * IIWARobot::GetDesiredForceAsFloat()
{
  float * float_cartForce;
  float_cartForce = (float*)malloc(6*sizeof(float));
  for(int i=0;i<6;i++){
    float_cartForce[i] = CartDesiredForce(i);
  }
  return float_cartForce;
}

void IIWARobot::SetGravComp(int joint_ind, bool select)
{
  if(joint_ind >=0 && joint_ind <= 6)
    {
      SelectedGravComp(joint_ind) = (int)select;
    }
  else
    {
      cout<<"Bad joint index!"<<endl;
    }
}

bool IIWARobot::GetGravComp(int joint_ind)
{
  if(joint_ind >=0 && joint_ind <= 6)
    {
      return (bool)SelectedGravComp(joint_ind);
    }
  else
    {
      cout<<"Bad joint index!"<<endl;
      return false;
    }
}
