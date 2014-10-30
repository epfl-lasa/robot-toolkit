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

#ifndef SIMULATORDYNAMICS_H_
#define SIMULATORDYNAMICS_H_

#include "MathLib/MathLib.h"
#include "RobotLib/World.h"


using namespace MathLib;

class BaseSimulatorDynamics;
class BaseDynamicObject;
class SimulatorDynamics;
class DynamicObject;



class BaseDynamicObject
{
    friend class BaseSimulatorDynamics;
protected:
    WorldObject             *mObject;
    BaseSimulatorDynamics   *mDynWorld;

public:
            BaseDynamicObject(BaseSimulatorDynamics *world);
    virtual ~BaseDynamicObject();

    virtual void    Free();

    virtual void    LinkToObject(WorldObject *object);


    virtual void    Prepare();
    virtual void    Update(REALTYPE dt);

    virtual void    Reset();

    virtual BaseDynamicObject*  GetDynamicObject(WorldObject *object);

};


class BaseSimulatorDynamics
{
    friend class BaseDynamicObject;
    friend class DynamicObject;
protected:
    vector<BaseDynamicObject*>   mDynObjects;

    World                       *mWorld;

public:
            BaseSimulatorDynamics();
    virtual ~BaseSimulatorDynamics();

    virtual void    Free();

            void                LinkToWorld(World *world);
    virtual BaseDynamicObject*  CreateDynamicObject();
    virtual BaseDynamicObject*  CreateDynamicRobot();
    virtual void                CreateFixedLink(BaseDynamicObject* objA, BaseDynamicObject* objB);
            BaseDynamicObject*  GetDynamicObject(WorldObject *object);

            void                Update(REALTYPE dt, REALTYPE maxStepTime = 0.002);
    virtual void                UpdateCore(REALTYPE dt);

    virtual void                Reset();

    static  void                AllowRobotReset(bool allow);
    static  bool                 bAllowRobotReset;

};

#include "SimulatorDynamicsODEBULLET.h"

#endif
