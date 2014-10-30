/*
 * Copyright (C) 2010 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
 * Author: Eric Sauser
 * email:   eric.sauser@a3.epf.ch
 * website: lasa.epfl.ch
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#ifndef ACTUATOR_H_
#define ACTUATOR_H_

#include <string>
#include <vector>
using namespace std;

#include "MathLib/MathLib.h"
using namespace MathLib;
#include "StdTools/BaseObject.h"



class Actuator;
typedef Actuator *pActuator;
typedef vector<pActuator> ActuatorsList;

BASEOBJECT_TOPCLASS(Actuator)

public:
string  mName;
public:
Actuator();
virtual ~Actuator();

void      SetName(string name);
string    GetName();

virtual void      Zero();
virtual void      Set(const Actuator& actuator);
virtual int       GetDOFCount();
};



BASEOBJECT_CLASS(Generic1DOFJointActuator,Actuator)

public:
REALTYPE  mPosition;
REALTYPE  mVelocity;
REALTYPE  mAcceleration;
REALTYPE  mTorque;

public:
Generic1DOFJointActuator();
Generic1DOFJointActuator(const Generic1DOFJointActuator& actuator);
virtual ~Generic1DOFJointActuator();

virtual void      Zero();
virtual void      Set(const Actuator& actuator);
virtual int       GetDOFCount();

void      Set(REALTYPE position, REALTYPE velocity, REALTYPE acceleration, REALTYPE torque);
};
typedef Generic1DOFJointActuator *pGeneric1DOFJointActuator;


typedef vector<pGeneric1DOFJointActuator> Generic1DOFJointActuatorsList;
class Generic1DOFJointActuatorGroup
{
  friend class Generic1DOFJointSensorGroup;
 protected:
  Generic1DOFJointActuatorsList       mActuatorList;
  int                                 mActuatorCount;

  Vector mPosition;
  Vector mVelocity;
  Vector mAcceleration;
  Vector mTorque;
 public:
  Generic1DOFJointActuatorGroup();
  ~Generic1DOFJointActuatorGroup();

  void        SetActuatorsList(const ActuatorsList& list);
  void        WriteActuators();
  void        ReadActuators();

  int         GetCount();
  void        SetJointAngles(const Vector& vector);
  void        SetJointPositions(const Vector& vector);
  void        SetJointVelocities(const Vector& vector);
  void        SetJointAccelerations(const Vector& vector);
  void        SetJointTorques(const Vector& vector);

  Vector&     GetJointAngles();
  Vector&     GetJointPositions();
  Vector&     GetJointVelocities();
  Vector&     GetJointAccelerations();
  Vector&     GetJointTorques();

  void     GetJointAngles(Vector& result);
  void     GetJointPositions(Vector& result);
  void     GetJointVelocities(Vector& result);
  void     GetJointAccelerations(Vector& result);
  void     GetJointTorques(Vector& result);

};

#define RevoluteJointActuator           Generic1DOFJointActuator
#define pRevoluteJointActuator          pGeneric1DOFJointActuator
#define RevoluteJointActuatorsList      Generic1DOFJointActuatorsList
#define RevoluteJointActuatorGroup      Generic1DOFJointActuatorGroup

#define SliderJointActuator           Generic1DOFJointActuator
#define pSliderJointActuator          pGeneric1DOFJointActuator

#endif /*ACTUATOR_H_*/
