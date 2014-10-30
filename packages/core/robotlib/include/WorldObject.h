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

#ifndef WORLDOBJECT_H_
#define WORLDOBJECT_H_

#include "MathLib/MathLib.h"
using namespace MathLib;
#include "StdTools/XmlTree.h"
#include "StdTools/LogStream.h"
#include "StdTools/Streamable.h"

#include <string>
#include <vector>
using namespace std;


class Robot;

class WorldObject : public Streamable
{
public:
    typedef struct{
        REALTYPE    mBouncyness;
        REALTYPE    mStaticFrictionCoef;
    } MaterialProperty;
protected:
    string                  mName;

    pXmlTree                mConfigTree;
    pXmlTree                mBBoxShapeTree;
    pXmlTree                mGfxShapeTree;

    SpatialInertia          mInertia;
    MaterialProperty        mMaterial;

    ReferenceFrame          mWorldReferenceFrame;
    SpatialFrame            mWorldSpatialFrame;
    SpatialVelocity         mWorldVelocity;
    SpatialForce            mWorldExternalForces;
    SpatialVelocity         mRelativeVelocity;
    SpatialForce            mRelativeExternalForces;

    ReferenceFrame          mInitialWorldReferenceFrame;
    SpatialVelocity         mInitialWorldVelocity;

    Robot                  *mRobot;

    bool                    bSlaveMode;
public:
    /// Constructor
    WorldObject();
    /// Destructor
    ~WorldObject();
    /// Cleanup
    void    Free();

    /// Load the object from a xml tree
    virtual bool        Load(pXmlTree tree);
    /// Set the robot name
    void                SetName(string name);

    /// Get the object name
    string&             GetName();
    /// Get the collision shape tree of the object
    pXmlTree            GetBBoxShapeTree();
    /// Get the graphical shape tree of the object
    pXmlTree            GetGfxShapeTree();
    /// Get the original configuration tree of the object
    pXmlTree            GetConfigTree();

    /// Get the frame of reference of the object in world corrdinate frame (initial=true provide initial position) This could be used for setting it as well
    ReferenceFrame&     GetReferenceFrame(bool initial = false);
    /// Get the spatial frame of the object (similar to GetReferenceFrame)
    SpatialFrame&       GetSpatialFrame();

    /// Get the curent object velocity in world frame (initia=true provide initial velocity)
    SpatialVelocity&    GetSpatialVelocity(bool initial = false);
    /// Get the curent object velocity in object frame
    SpatialVelocity&    GetRelativeSpatialVelocity();

    /// Set the curent object velocity in world frame
    void                SetSpatialVelocity(const SpatialVelocity& vel);
    /// Set the curent object velocity in object frame
    void                SetRelativeSpatialVelocity(const SpatialVelocity& vel);

    /// Get object velocity mass information
    SpatialInertia&     GetSpatialInertia();
    /// Get object material property
    MaterialProperty&   GetMaterialProperty();


    /// Get the current sum of external forces exerted on the object in world frame
    SpatialForce&       GetExternalForces();
    /// Get the current sum of external forces exerted on the object in object frame
    SpatialForce&       GetExternalRelativeForces();

    /// Set all external forces to zero
    void                ClearExternalForces();
    /// Add center of mass centered force and torque in world frame
    void                AddExternalForces(const SpatialForce& force);
    /// Add a center of mass centered force in world frame
    void                AddExternalForce (const Vector3& force);
    /// Add a center of mass centered torque in world frame
    void                AddExternalTorque(const Vector3& torque);
    /// Add a force in world frame at a given postion in world frame
    void                AddExternalForce (const Vector3& force, const Vector3 & pos);
    /// Add a center of mass centered force in object frame
    void                AddExternalRelativeForce (const Vector3& force);
    /// Add a center of mass centered torque in object frame
    void                AddExternalRelativeTorque(const Vector3& torque);
    /// Add a force in object frame at a given position in object frame
    void                AddExternalRelativeForce (const Vector3& force, const Vector3 & pos);

    /// Set a pointer to a robot is the object should be one
    void                SetRobot(Robot *robot);
    /// Is the object a robot
    bool                IsRobot();
    /// Get the robot
    Robot*              GetRobot();

    /// Set the initial state from current
    void                SetInitialState();
    /// Restore the object initial state
    void                SetToInitialState();

    vector<Vector3>     mContacts;
    vector<Vector3>     mContactsNormal;

    bool                IsSlave();

    virtual int         StreamSize();
    virtual int         StreamSizeFromStream(const void* memory);
    virtual int         SetStream(void* memory);
    virtual int         SetFromStream(const void* memory);

public:
    static WorldObject  sDummyObject;
};




typedef WorldObject *pWorldObject;

class World;

class WorldObjectLink
{
protected:
    string  mObjAName;
    string  mObjBName;

    pWorldObject    mObjA;
    pWorldObject    mObjB;
public:
    WorldObjectLink(string objA, string objB);

    pWorldObject    GetObjectA();
    pWorldObject    GetObjectB();

    void Resolve(World* world);


};

#endif
