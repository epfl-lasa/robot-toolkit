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

#ifndef DynamicSimulatorINTERFACE_H_
#define DynamicSimulatorINTERFACE_H_

#include <iostream>
#include <fstream>
using namespace std;

#include "RobotInterface.h"
#include "ForwardDynamics.h"
#include "InverseDynamics.h"
#include "MathLib/Differentiator.h"

using namespace MathLib;

class DynamicSimulatorInterface : public RobotInterface
{
public:  
    enum SimulationMode {
        SIMMODE_POSITION = 0,
        SIMMODE_VELOCITY,
        SIMMODE_ACCELERATION,
        SIMMODE_TORQUE
    };

protected:
    RevoluteJointSensorGroup    mSensorsGroup;
    RevoluteJointActuatorGroup  mActuatorsGroup;
    ForwardDynamics             mFwdDynamics;
    InverseDynamics             mInvDynamics;
    Integrator                  mIntegrator;
    EulerDifferentiator         mDifferentiator;

    SimulationMode              mSimMode;

    RevoluteJointSensorGroup    mSimSensorsGroup;
    RevoluteJointActuatorGroup  mSimActuatorsGroup;
    Robot                      *mSimRobot;
public:      
    
            DynamicSimulatorInterface();
    virtual ~DynamicSimulatorInterface();
  
    virtual Status              RobotInit();
    virtual Status              RobotFree();
  
    virtual Status              RobotStart();    
    virtual Status              RobotStop();
  
    virtual Status              RobotUpdate();
    virtual Status              RobotUpdateCore();

    virtual void                UpdateActuators();
    virtual void                UpdateSensors();
  
            void                SetSimulationMode(SimulationMode mode);
            void                SetSimulationRobot(Robot* simRobot);
};


#endif 
