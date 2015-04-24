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

#ifndef ExampleWorldControlModule_H_
#define ExampleWorldControlModule_H_

#include "RobotLib/WorldInterface.h"

class ExampleWorldControlModule : public WorldInterface
{
protected:
    Vector3     mExtForce;
    double      mImpulseTime;
public:
            ExampleWorldControlModule();
    virtual ~ExampleWorldControlModule();
  
    virtual Status              WorldInit();
    virtual Status              WorldFree();
  
    virtual Status              WorldStart();    
    virtual Status              WorldStop();
  
    virtual Status              WorldUpdate();
    virtual Status              WorldUpdateCore();

    virtual int                 RespondToConsoleCommand(const string cmd, const vector<string> &args);
};



#endif 
