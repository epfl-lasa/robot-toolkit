/*
 * LWRRobot.h
 *
 *  Created on: Mar 14, 2012
 *      Author: klas
 */

#ifndef LWRROBOT_H_
#define LWRROBOT_H_

#define FRI_JOINT_STIFFNESS 1000
#define FRI_JOINT_DAMPING   0.7

#define FRI_CART_STIFFNESS_POS 		200
#define FRI_CART_STIFFNESS_ORIENT 	20
#define FRI_CART_DAMPING_POS 		0.7
#define FRI_CART_DAMPING_ORIENT 	0.7

#define NB_JOINTS 7
#define NB_CART 6

#define LWR_state_size 7+12+7+7+6+7+12


#include <Robot.h>

class LWRRobot: public Robot {
 public:
  LWRRobot();
  virtual ~LWRRobot();

  Vector GetJointStiffness();
  Vector GetJointDamping();

  void SetJointStiffness(const Vector &);
  void SetJointDamping(const Vector &);

  Vector GetCartStiffness();
  Vector GetCartDamping();



  void SetCartCommand(const Vector3 &, const Matrix3 &);
  void SetCartCommand(const Matrix4 &);

  void SetCartStiffness(const Vector&);
  void SetCartDamping(const Vector &);
  void SetCartForce(const Vector &);

  void SetEstimatedExternalJointTorques(const Vector&);
  void SetEstimatedExternalCartForces(const Vector&);
  void SetMeasuredJointTorques(const Vector&);
  void SetCommandedJointTorques(Vector&);
  void SetMeasuredCartPose(float *);

  void SetCompleteLWRState(const Vector&);

  void SetGravComp(int, bool);
  bool GetGravComp(int);
  Vector GetCompleteLWRState();
  Vector GetEstimatedExternalJointTorques();
  Vector GetMeasuredJointTorques();
  Vector GetEstimatedExternalCartForces();
  Vector GetCommandedJointTorques();
  void GetMeasuredCartPose(Vector3 &,Matrix3 &);

  void SetSamplingTime(float); //s
  float GetSamplingTime();  //s





  bool IsAlive();
  void SetAlive(bool);

  float * GetCartCommandAsFloat();

  Matrix GetMassMatrix();
  void SetMassMatrix(float **massMatrixPointer);

  float * GetCartStiffnessAsFloat();
  float * GetCartDampingAsFloat();
  float * GetDesiredForceAsFloat();

private:
  //LWR-specific controls:
  Vector JointStiffness;
  Vector JointDamping;
  Vector CartStiffness;
  Vector CartDamping;
  Vector CartDesiredForce;
  Vector3 CommandedCartPosition;
  Matrix3 CommandedCartOrientation;
  Vector  CommandedJointTorques;
  //LWR-specific sensing:
  Vector EstimatedExternalJointTorques;
  Vector EstimatedExternalCartForces;
  Vector MeasuredJointTorques;
  Vector MeasuredCartForces;
  Vector CompleteLWRState;
  Vector3 MeasuredCartPosition;
  Matrix3 MeasuredCartOrientation;
  Matrix massMatrix;

  float MeasuredSamplingTime; //s

  Vector SelectedGravComp;
  void InitializeLWRComponents();
  bool bAlive;
};

#endif /* LWRROBOT_H_ */
