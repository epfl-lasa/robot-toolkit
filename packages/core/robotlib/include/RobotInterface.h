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

#ifndef ROBOTINTERFACE_H_
#define ROBOTINTERFACE_H_

#include "StdTools/Console.h"
#include "StdTools/Various.h"
#include "StdTools/WrapMallocs.h"

#include "ModuleInterface.h"
#include "Sensor.h"
#include "Actuator.h"
#include "RobotTools.h"
#include "World.h"

class Robot;

/// Generic robot interface class which should be used for as a robot module
class RobotInterface : public ModuleInterface
{
protected:
    Robot          *mRobot;
    World          *mWorld;

public:
    /// Constructor. The parameter isControlInterface defines if this interface conbtrol the whole application
            RobotInterface(bool isControlInterface = false);
    /// Destructor
    virtual ~RobotInterface();

    /// Set the corresponding robot (has to be done prior initialization)
            void                SetRobot(Robot* robot);
    /// Set the corresponding world (has to be done prior initialization)
            void                SetWorld(World* world);
    /// Get the robot
            Robot*              GetRobot();
    /// Get the world
            World*              GetWorld();

    /// List of overloaded phase function called from the parent module interface
            Status              InterfaceInit();
            Status              InterfaceFree();
            Status              InterfaceStart();
            Status              InterfaceStop();
            Status              InterfaceUpdate();
            Status              InterfaceUpdateCore();

            void                InterfaceDraw();

    /// Function to be overloaded in user interfaces for acting given the operational phase
    virtual Status              RobotInit();
    virtual Status              RobotFree();

    virtual Status              RobotStart();
    virtual Status              RobotStop();

    virtual Status              RobotUpdate();
    virtual Status              RobotUpdateCore();

    /// If IsControlInterface()==true, this will be called to prepare the robot sensors
    virtual void                UpdateSensors();
    /// If IsControlInterface()==true, this will be called to send commands to the robot actuators
    virtual void                UpdateActuators();

    /// Overloadable function to respond to console commands
    virtual int                 RespondToConsoleCommand(const string cmd, const vector<string> &args);

    /// Overloadable function to access the opengl rendering pipeline (valid in simulation only)
    virtual void                RobotDraw();
};



#include "Robot.h"

#endif /*ROBOTINTERFACE_H_*/
