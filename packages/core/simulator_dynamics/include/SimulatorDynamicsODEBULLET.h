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

#ifndef SIMULATORDYNAMICS_ODEBULLET_H_
#define SIMULATORDYNAMICS_ODEBULLET_H_


#include "SimulatorDynamics.h"
#include "RobotLib/Robot.h"
#include "ode/ode.h"
#include "btBulletDynamicsCommon.h"

#include <vector>
using namespace std;




class DynamicObject : public BaseDynamicObject
{
    friend class SimulatorDynamics;
    friend class DynamicRobot;
protected:
    dBodyID                 mODEBody;

    btCollisionObject*      mBTColObject;
    btCollisionShape*       mBTColShape;


    ReferenceFrame          mCOMFrame;
    SpatialFrame            mCOMSpatialFrame;

    int                     mCollisionGroupId;
    int                     mCollisionGroupPartId;

    bool                    bIsKinematic;
    bool                    bNeedContact;
    bool                    bNeedReset;
    bool                    bSetDynamics;


    Matrix                  mHeightField;
public:
            DynamicObject(SimulatorDynamics *world);
    virtual ~DynamicObject();


    virtual void    Free();

            void    SetKinematicOnly(bool only);
    virtual void    LinkToObject(WorldObject *object);


    virtual void    Prepare();
    virtual void    Update(REALTYPE dt);
    virtual void    Reset();

    virtual BaseDynamicObject*  GetDynamicObject(WorldObject *object);

};

class DynamicRobot : public DynamicObject
{
protected:
    Robot                       *mRobot;
    RevoluteJointSensorGroup    mSensorsGroup;
    RevoluteJointActuatorGroup  mActuatorsGroup;

    EulerDifferentiator         mDifferentiator;
    Integrator                  mIntegrator;

    Robot                      *mSimRobot;
    RevoluteJointSensorGroup    mSimSensorsGroup;
    RevoluteJointActuatorGroup  mSimActuatorsGroup;

    vector<DynamicObject*>      mDynLinks;
    vector<dJointID>            mDynJoints;
    vector<Joint*>              mRobotJoints;
    vector<dJointFeedback*>     mDynJointsFeedback;
    vector<REALTYPE>            mDynJointsOffset;


    double mGTime;
public:
            DynamicRobot(SimulatorDynamics *world);
    virtual ~DynamicRobot();

    virtual void    Free();

    virtual void    LinkToObject(WorldObject *object);
            void    LinkToRobot(Robot* robot);

    virtual void    Prepare();
    virtual void    Update(REALTYPE dt);
    virtual void    Reset();

    virtual BaseDynamicObject*  GetDynamicObject(WorldObject *object);
};

class SimulatorDynamics : public BaseSimulatorDynamics
{
    friend class DynamicObject;
    friend class DynamicRobot;
protected:


    dWorldID                mODEWorld;
    dJointGroupID           mODEContacts;

    btAlignedObjectArray<btCollisionShape*>     mBTCollisionShapes;
    btBroadphaseInterface                      *mBTBroadphase;
    btCollisionDispatcher                      *mBTDispatcher;
    btDefaultCollisionConfiguration            *mBTCollisionConfiguration;
    btCollisionWorld                           *mBTCollisionWorld;

    btCollisionObject*  mGroundObject;
    btCollisionShape*   mGroundShape;

    vector<dJointID>            mDynFixedJoints;

    Vector3                     mGravity;
    double						mDamping;

public:
    static  REALTYPE  sGlobalScaling;

public:
    SimulatorDynamics();
    ~SimulatorDynamics();

    void    SetGravity(const Vector3 & gravity);
    void    SetDamping(double damping);

            void                Init();
    virtual void                Free();

    virtual BaseDynamicObject*  CreateDynamicObject();
    virtual BaseDynamicObject*  CreateDynamicRobot();
    virtual void                CreateFixedLink(BaseDynamicObject* objA, BaseDynamicObject* objB);


    virtual void                UpdateCore(REALTYPE dt);

    virtual void                Reset();

    static  void StaticCollisionCallback(void *data, dGeomID o0, dGeomID o1);
            void CollisionCallback(dGeomID o0, dGeomID o1);


    class customOverlapFilterCallback :public btOverlapFilterCallback
    {
    public:
        virtual bool needBroadphaseCollision(btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1) const;
    };
    static customOverlapFilterCallback mCustomOverlapFilterCallback;
    static  void nearCallback(btBroadphasePair& collisionPair,
                              btCollisionDispatcher& dispatcher, const btDispatcherInfo& dispatchInfo);

};



#endif
