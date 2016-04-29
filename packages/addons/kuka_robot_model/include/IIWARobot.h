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


#include <LWRRobot.h>

class IIWARobot: public LWRRobot {
 public:

  virtual void SetEstimatedExternalCartForces(const Vector&);
  virtual  void SetCommandedJointTorques(Vector&);

  void SetCompleteIIWAState(const Vector&);
  Vector GetCompleteIIWAState();


private:
  Vector CompleteIIWAState;
  void InitializeIIWAComponents();
};

#endif /* IIWAROBOT_H_ */
