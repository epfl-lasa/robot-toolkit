/*
 * LWRRobot.cpp
 *
 *  Created on: Mar 14, 2012
 *      Author: klas
 */

#include "LWRRobot.h"

LWRRobot::LWRRobot():Robot::Robot(false){


  InitializeLWRComponents();


}

LWRRobot::~LWRRobot() {
  // TODO Auto-generated destructor stub
}



float * LWRRobot::GetCartCommandAsFloat(){

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

void LWRRobot::SetCartCommand(const Vector3 & cartCommandPos, const Matrix3 & cartCommandOrient) {
  CommandedCartPosition = cartCommandPos;
  CommandedCartOrientation = cartCommandOrient;
}

void LWRRobot::SetCartCommand(const Matrix4 & cartCommandHomMatrix){
  CommandedCartPosition = cartCommandHomMatrix.GetTranslation();
  CommandedCartOrientation = cartCommandHomMatrix.GetOrientation();
}

Vector LWRRobot::GetCartDamping() {
  return CartDamping;
}

void LWRRobot::SetCartDamping(const Vector & cartDamping) {
  CartDamping = cartDamping;
}

Vector LWRRobot::GetCartStiffness() {
  return CartStiffness;
}

void LWRRobot::SetCartStiffness(const Vector & cartStiffness) {
  CartStiffness = cartStiffness;
}

Vector LWRRobot::GetJointDamping() {
  return JointDamping;
}

void LWRRobot::SetJointDamping(const Vector & jointDamping) {
  JointDamping = jointDamping;
}

Vector LWRRobot::GetJointStiffness()  {
  return JointStiffness;
}

void LWRRobot::SetJointStiffness(const Vector & jointStiffness) {
  JointStiffness = jointStiffness;
}

void LWRRobot::SetAlive(bool bb){
  bAlive = bb;
}

bool LWRRobot::IsAlive(){
  return bAlive;
}

void LWRRobot::SetEstimatedExternalJointTorques(const Vector & jointEJT){
  if(jointEJT.Size()==NB_JOINTS){
    EstimatedExternalJointTorques = jointEJT;
  }
}
void LWRRobot::SetEstimatedExternalCartForces(const Vector & cartEF){
  if(cartEF.Size()==NB_CART){
    EstimatedExternalCartForces= cartEF;
  }
}

void LWRRobot::SetMeasuredJointTorques(const Vector & jointT){
  if(jointT.Size()==NB_JOINTS){
    MeasuredJointTorques = jointT;
  }
}
void LWRRobot::SetCommandedJointTorques(Vector & jointT){
  if(jointT.Size() == NB_JOINTS){
    CommandedJointTorques = jointT;
  }
}

Vector LWRRobot::GetEstimatedExternalJointTorques(){
  return EstimatedExternalJointTorques;
}
Vector LWRRobot::GetMeasuredJointTorques(){
  return MeasuredJointTorques;
}
Vector LWRRobot::GetEstimatedExternalCartForces(){
  return EstimatedExternalCartForces;
}
Vector LWRRobot::GetCommandedJointTorques(){
  return CommandedJointTorques;
}
void LWRRobot::SetCompleteLWRState(const Vector& s){
  CompleteLWRState = s;
}

Vector LWRRobot::GetCompleteLWRState(){
  return CompleteLWRState;
}

void LWRRobot::SetMassMatrix(float** massMatrixPointer)
{
  for (int i=0; i<NB_JOINTS; i++)
    for (int j=0;j<NB_JOINTS;j++)
      massMatrix(i,j) = massMatrixPointer[i][j];
}

Matrix LWRRobot::GetMassMatrix()
{
  return massMatrix;
}

void LWRRobot::InitializeLWRComponents(){


  CommandedJointTorques.Resize(NB_JOINTS);
  massMatrix.Resize(NB_JOINTS,NB_JOINTS);

  SelectedGravComp.Resize(7);
  SelectedGravComp.One();
  JointStiffness.Resize(NB_JOINTS);
  JointDamping.Resize(NB_JOINTS);

  EstimatedExternalJointTorques.Resize(NB_JOINTS);
  EstimatedExternalCartForces.Resize(NB_CART);
  MeasuredJointTorques.Resize(NB_JOINTS);

  CompleteLWRState.Resize(LWR_state_size);

  CartStiffness.Resize(NB_CART);
  CartDamping.Resize(NB_CART);
  CartDesiredForce.Resize(NB_CART);

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

void LWRRobot::GetMeasuredCartPose(Vector3 & resultPos, Matrix3 & resultMat)
{
  resultPos = MeasuredCartPosition;
  resultMat = MeasuredCartOrientation;
}

float LWRRobot::GetSamplingTime() //s
{
  float current_time = MeasuredSamplingTime;
  return current_time;
}

void LWRRobot::SetSamplingTime(float measured_samp_time) //s
{
  MeasuredSamplingTime=measured_samp_time;
}

void LWRRobot::SetMeasuredCartPose(float * float_cartMeasured)
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


float * LWRRobot::GetCartStiffnessAsFloat()
{
  float * float_cartStiffness;
  float_cartStiffness = (float*)malloc(6*sizeof(float));
  for(int i=0;i<6;i++){
    float_cartStiffness[i] = CartStiffness(i);
  }

  // ok here.

  return float_cartStiffness;
}

float * LWRRobot::GetCartDampingAsFloat()
{
  float * float_cartDamping;
  float_cartDamping = (float*)malloc(6*sizeof(float));
  for(int i=0;i<6;i++){
    float_cartDamping[i] = CartDamping(i);
  }
  return float_cartDamping;
}

void LWRRobot::SetCartForce(const Vector & desForce)
{
  CartDesiredForce = desForce;

  //cout<<"CartDesidredForceeeeeee";
  //CartDesiredForce.Print();

}

float * LWRRobot::GetDesiredForceAsFloat()
{
  float * float_cartForce;
  float_cartForce = (float*)malloc(6*sizeof(float));
  for(int i=0;i<6;i++){
    float_cartForce[i] = CartDesiredForce(i);
  }
  return float_cartForce;
}

void LWRRobot::SetGravComp(int joint_ind, bool select)
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

bool LWRRobot::GetGravComp(int joint_ind)
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
