/*
 * IIWARobot.h
 *
 *  Created on: Mar 14, 2012
 *      Author: klas
 */

#ifndef IIWAROBOT_H_
#define IIWAROBOT_H_

#define FRI_JOINT_STIFFNESS 1000
#define FRI_JOINT_DAMPING   0.7

#define FRI_CART_STIFFNESS_POS 		200
#define FRI_CART_STIFFNESS_ORIENT 	20
#define FRI_CART_DAMPING_POS 		0.7
#define FRI_CART_DAMPING_ORIENT 	0.7

#define NB_JOINTS 7

#define IIWA_state_size 7+7+7+7


#include <Robot.h>

class IIWARobot: public Robot {
 public:
  IIWARobot();
  virtual ~IIWARobot();

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

  void SetCompleteIIWAState(const Vector&);

  void SetGravComp(int, bool);
  bool GetGravComp(int);
  Vector GetCompleteIIWAState();
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
  //IIWA-specific controls:
  Vector JointStiffness;
  Vector JointDamping;
  Vector CartStiffness;
  Vector CartDamping;
  Vector CartDesiredForce;
  Vector3 CommandedCartPosition;
  Matrix3 CommandedCartOrientation;
  Vector  CommandedJointTorques;
  //IIWA-specific sensing:
  Vector EstimatedExternalJointTorques;
  Vector EstimatedExternalCartForces;
  Vector MeasuredJointTorques;
  Vector MeasuredCartForces;
  Vector CompleteIIWAState;
  Vector3 MeasuredCartPosition;
  Matrix3 MeasuredCartOrientation;
  Matrix massMatrix;

  float MeasuredSamplingTime; //s

  Vector SelectedGravComp;
  void InitializeIIWAComponents();
  bool bAlive;
};

#endif /* IIWAROBOT_H_ */
