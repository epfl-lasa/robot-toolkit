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

#ifndef ForwardDynAMICSCONTROLLER_H_
#define ForwardDynAMICSCONTROLLER_H_

//#include "StdTools/SlotsAndBoxes.h"
#include "MathLib/MathLib.h"
#include "StdTools/Timer.h"

#include "Robot.h"
#include "Sensor.h"
#include "Actuator.h"
#include "InverseDynamics.h"

class ForwardDynamics
{  
protected:
  Robot             *mRobot;
  LinksList         *mLinks;
  JointsList        *mJoints;  
  unsigned int      mLinksCount;
  
  Vector3           mGravity;
  
  Vector            mOutputAccel;

  int               mFrictionMethod;
  
    
  typedef struct{
    TMatrix<6>      m_X;
    TMatrix<6>      m_Xt;
    TVector<6>      m_v;
    TVector<6>      m_s;
    TMatrix<6>      m_I;
    TVector<6>      m_p;
    TVector<6>      m_U;
    REALTYPE        m_D;
    REALTYPE        m_u;
    TVector<6>      m_a;
    TVector<6>      m_vdq;
    TVector<6>      m_vddq;
    TVector<6>      m_f;    
    REALTYPE        m_ddq;
    REALTYPE        m_dq;
    REALTYPE        m_q;
    REALTYPE        m_t;
    int             m_bJoint;
  } LinkProperty;
  
  LinkProperty *mLinkP;
    
public:
          ForwardDynamics();
  virtual ~ForwardDynamics();

          void    Init(XmlTree *tree = NULL);          
          void    Free();

          void    Update();
  
          void    SetGravity(const Vector3& gravity);
          void    SetRobot(pRobot robot);
          
          void    GetAccelerations(Vector & output);
          
};


#endif
