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

#ifndef WORLDINTERFACE_H_
#define WORLDINTERFACE_H_

#include "ModuleInterface.h"
#include "StdTools/WrapMallocs.h"
class World;

class WorldInterface : public ModuleInterface
{
protected:
    World          *mWorld;

public:
    /// Constructor. The parameter isControlInterface defines if this interface conbtrol the whole application
            WorldInterface(bool isControlInterface = false);
    /// Destructor
    virtual ~WorldInterface();

    /// Set the corresponding world (has to be done prior initialization)
            void                SetWorld(World* world);
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
    virtual Status              WorldInit();
    virtual Status              WorldFree();

    virtual Status              WorldStart();
    virtual Status              WorldStop();

    virtual Status              WorldUpdate();
    virtual Status              WorldUpdateCore();

    /// Overloadable function to respond to console commands
    virtual int                 RespondToCommand(const string cmd, const vector<string> &args);
    
    /// Overloadable function to access the opengl rendering pipeline (valid in simulation only)
    virtual void                WorldDraw();
    
};



#include "World.h"

#endif /*WorldINTERFACE_H_*/
