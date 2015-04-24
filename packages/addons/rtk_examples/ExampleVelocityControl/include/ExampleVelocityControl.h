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

#ifndef ExampleVelocityControl_H_
#define ExampleVelocityControl_H_

#include "RobotLib/RobotInterface.h"

class ExampleVelocityControl : public RobotInterface
{
protected:
    RevoluteJointSensorGroup    mSensorsGroup;
    RevoluteJointActuatorGroup  mActuatorsGroup;

    Vector                      mRestPosition;
    Vector                      mCurrentPosition;

    int                         mState;    

    int                         mLeftIndex;
    int                         mRightIndex;

public:
            ExampleVelocityControl();
    virtual ~ExampleVelocityControl();
  
    virtual Status              RobotInit();
    virtual Status              RobotFree();
  
    virtual Status              RobotStart();    
    virtual Status              RobotStop();
  
    virtual Status              RobotUpdate();
    virtual Status              RobotUpdateCore();

    virtual int                 RespondToConsoleCommand(const string cmd, const vector<string> &args);
};



#endif 
