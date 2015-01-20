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

#ifndef PIDCONTROLLER_H_
#define PIDCONTROLLER_H_

#include "MathLib/MathLib.h"
using namespace MathLib;
#include "StdTools/XmlTree.h"

class PIDController
{  
protected:
  enum State {PIDCTRL_NONE=0,PIDCTRL_STOP,PIDCTRL_START};
protected:
  State           mState;
  
  Vector          mTarget;
  Vector          mInput;
  
public:
  Vector          mKp;
  Vector          mKd;
  Vector          mKi;
protected:
  Vector          mOutputLowBound;
  Vector          mOutputHighBound;
  
  Vector          mErrors;
  Vector          mErrorsDiff;
  Vector          mErrorsInt;
  Vector          mLastErrors;

  Vector          mBuffer;    

  Vector          mOutput;    
public:
          PIDController();
  virtual ~PIDController();

  virtual void    Init(pXmlTree tree = NULL);          
  virtual void    Free();

  virtual void    Update(REALTYPE dt);
  virtual void    Process();
  
          void    Start();
          void    Stop();
          void    Reset();
  
          void    SetTarget(const Vector & target);
          void    SetInput(const Vector & input);
          Vector& GetOutput(Vector & result);  
          Vector& GetOutput();
          void    AddOutput(Vector & result);
          
          void    SetKP(const Vector& kp);
          void    SetKD(const Vector& kd);
          void    SetKI(const Vector& ki);
          void    SetKs(const Vector& kp, const Vector& kd, const Vector& ki);
          void    SetBounds(const Vector& low,const Vector& high);
          
          void    Resize(int size,bool Ks=false);
  
};


#endif
