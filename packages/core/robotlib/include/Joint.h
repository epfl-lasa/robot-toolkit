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

#ifndef __JOINT_H__
#define __JOINT_H__

#include <string>
#include <vector>
using namespace std;

#include "MathLib/MathLib.h"
using namespace MathLib;
#include "StdTools/XmlTree.h"
#include "StdTools/BaseObject.h"

#include "Sensor.h"
#include "Actuator.h"


class Joint;
typedef Joint *pJoint;
typedef vector<pJoint> JointsList;

class Link;

BASEOBJECT_TOPCLASS(Joint)
public:
enum JointControlMode {
  JCTRLMODE_NONE = 0,
  JCTRLMODE_POSITION,
  JCTRLMODE_VELOCITY,
  JCTRLMODE_ACCELERATION,
  JCTRLMODE_TORQUE
};

public:
bool                bIsLocked;

ReferenceFrame      mBaseFrame;
ReferenceFrame      mJointFrame;

ReferenceFrame     *mRefFrame;

Link               *mParentLink;
Link               *mChildLink;

Sensor             *mSensor;
Actuator           *mActuator;

JointControlMode    mControlMode;

public:
Joint();
Joint(const pXmlTree tree);
virtual ~Joint();


void    SetParentLink(Link *parent);
void    SetChildLink(Link *child);
void    SetBaseFrame(const ReferenceFrame & base);

void    Lock(bool lock=true);

virtual void    Update();

virtual void    Load(const pXmlTree tree);

virtual void    Print();

void    MapActuatorToSensor();

bool    IsCoupled();

void                SetControlMode(JointControlMode mode);
JointControlMode    GetControlMode();

protected:
virtual void    ConstructorInit();
virtual void    Free();
};



#define MAX_FRICTION_COEFS 16

BASEOBJECT_CLASS(RevoluteJoint,Joint)

public:
enum Axis {NONE, EX, EY, EZ, NEG_EX, NEG_EY, NEG_EZ, ARBITRARY};

public:

Axis      mAxis;
Vector3   mAxisVector;
REALTYPE  mZero;
REALTYPE  mRange[2];
REALTYPE  mFrictionCoefs[MAX_FRICTION_COEFS];


public:
RevoluteJoint();
RevoluteJoint(const pXmlTree tree);
virtual ~RevoluteJoint();


void              SetAxis(Axis axis);
void              SetAxis(Vector3 & axisVector);
void              SetAngle(REALTYPE angle);
const Vector3&    GetAxis();
virtual void              Update();

virtual void          Load(const pXmlTree tree);

virtual void          Print();

void GetJointLimits(REALTYPE & l, REALTYPE & h);



protected:
virtual void    ConstructorInit();
virtual void    Free();
};

typedef RevoluteJoint *pRevoluteJoint;


BASEOBJECT_CLASS(SliderJoint,Joint)

public:
enum Axis {NONE, EX, EY, EZ, NEG_EX, NEG_EY, NEG_EZ, ARBITRARY};

public:

Axis      mAxis;
Vector3   mAxisVector;
REALTYPE  mZero;
REALTYPE  mRange[2];
REALTYPE  mFrictionCoefs[MAX_FRICTION_COEFS];


public:
SliderJoint();
SliderJoint(const pXmlTree tree);
virtual ~SliderJoint();


void              SetAxis(Axis axis);
void              SetAxis(Vector3 & axisVector);
void              SetPosition(REALTYPE position);
const Vector3&    GetAxis();
virtual void              Update();

virtual void          Load(const pXmlTree tree);

virtual void          Print();

protected:
virtual void    ConstructorInit();
virtual void    Free();
};

typedef SliderJoint *pSliderJoint;





BASEOBJECT_CLASS(ForceSensorJoint6DOF,Joint)

public:
ForceSensorJoint6DOF();
ForceSensorJoint6DOF(const pXmlTree tree);
virtual ~ForceSensorJoint6DOF();


virtual void          Update();

virtual void          Load(const pXmlTree tree);

virtual void          Print();

protected:
virtual void    ConstructorInit();
virtual void    Free();
};

typedef ForceSensorJoint6DOF *pForceSensorJoint6DOF;


BASEOBJECT_CLASS(ContactJoint,Joint)

public:
ContactJoint();
ContactJoint(const pXmlTree tree);
virtual ~ContactJoint();


virtual void          Update();

virtual void          Load(const pXmlTree tree);

virtual void          Print();

protected:
virtual void    ConstructorInit();
virtual void    Free();
};

typedef ContactJoint *pContactJoint;



class JointConstructor
{
 public:
  static pJoint Create(const pXmlTree tree);
};

#endif

