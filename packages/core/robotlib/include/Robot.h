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

#ifndef __ROBOT_H__
#define __ROBOT_H__

#include <string>
using namespace std;

#include "Link.h"
#include "Sensor.h"
#include "Actuator.h"
#include "RobotInterface.h"
#include "ForwardKinematics.h"
#include "World.h"

#include "ros/ros.h"

class World;

/// Core robot class
class Robot
{
public:
    enum ControlMode {
        CTRLMODE_NONE = 0,
        CTRLMODE_POSITION ,
        CTRLMODE_VELOCITY,
        CTRLMODE_ACCELERATION,
        CTRLMODE_TORQUE,
        CTRLMODE_DEFAULT,
	//added for LWR compatibility
	CTRLMODE_CARTIMPEDANCE,
	CTRLMODE_JOINTIMPEDANCE,
	CTRLMODE_GRAVITYCOMPENSATION,

    };

protected:

    string                  mType;
    string                  mSubType;
    vector<string>          mPatches;
    string                  mName;

    vector<string>          mArgs;
    ControlMode             mControlMode;

    vector<int>             mParents;
    LinksList               mLinks;
    JointsList              mJoints;
    unsigned int            mLinksCount;

    LinkTree                mLinksTree;

    vector<RobotInterface*> mInterfaces;

    //TimeSensor              mTimeSensor;

    // For low level transfers from-to robot interfaces...
    ActuatorsList           mActuators;
    SensorsList             mSensors;

    ForwardKinematics       mForwardKinematics;

    void*                   mUserData;

    World*                  mWorld;
    WorldObject*            mWorldInstance;

    bool                    bSimMode;

    ros::NodeHandle * mNodeHandle;
    bool bROSInitialized;



public:
            Robot(bool simMode = false);
    virtual ~Robot();

    /// Get the robot type
            string          GetType();
    /// Get the robot subtype (extension)
            string          GetSubType();
    /// Get the robot name
            string          GetName();
    /// Set the robot name
            void            SetName(string name);
    /// Get the robot patches 
            string          GetPatches();

    /// Load a robot given its type and optionally, a list of patches
            bool            Load(string robotType, string patchList);
    /// Load a robot given its type, sub type and optionally, a list of patches (wrapper fro above function)
            bool            Load(string robotType, string robotSubType, string patchList);

            void            AddInterface(RobotInterface *robotInterface);


    /// Interfaces management: Init phase
            void            Init();
    /// Interfaces management: Free phase
            void            Free();
    /// Interfaces management: Start phase
            void            Start();
    /// Interfaces management: Stop phase
            void            Stop();
    /// Interfaces management: Update phase
            void            Update();
    /// Interfaces management: Update core phase
            void            UpdateCore();
    /// Interfaces management: Additional drawing phase
            void            Draw();

    /// Update links information given a new joint configuration (initialState stores the robot initial state as well)
            void            UpdateLinks(bool initialState = false);

    /// Get the vector of links
            LinksList&      GetLinks();
    /// Get the vector of joints
            JointsList&     GetJoints();
    /// Get the parent of each link by its index
            vector<int>&    GetParents();
    /// Get the index of link with a given name
            int             GetLinkIndex(const string name);
    /// Get the index of dof with a given name
            int             GetDOFIndex(const string name);
    /// Get the number of links
            unsigned int    GetLinksCount();
    /// Get the number of degree of freedom
            int             GetDOFCount();

    /// Get the number of sensors
            int             GetSensorsCount();
    /// Get the number of actuators
            int             GetActuatorsCount();

    /// Get the sensors in an array
            SensorsList&    GetSensors();
    /// Get the actuators in an array
            ActuatorsList&  GetActuators();
    /// Get a sensor by its index
            pSensor         GetSensor   (int index);
    /// Get an actuator by its index
            pActuator       GetActuator (int index);
    /// Get a sensor by its name
            pSensor         FindSensor  (const string name);
    /// Get an actuator by its name
            pActuator       FindActuator(const string name);

    /// Get the reference frame of link fromLink in the frame of toLink
            ReferenceFrame&   GetReferenceFrame(unsigned int fromLink, unsigned int toLink);
    /// Get the reference frame of link link in the world frame
            ReferenceFrame&   GetReferenceFrame(unsigned int link);

    /// Get the generic time sensor
            //TimeSensor&     GetTimeSensor();

            void            SetControlMode(ControlMode mode);
            ControlMode     GetControlMode();

    /// Get the world
            World*          GetWorld();
    /// Set the world
            void            SetWorld(World* world);

    /// Set the world object instance
            void            SetWorldInstance(WorldObject* wObject);
    /// Get the world object instance
            WorldObject*    GetWorldInstance();

    /// Set a argument list (can be used for passing command line arguments)
            void            SetArgs(const vector<string>& args);
    /// Get the argument list (can be used for reading command line arguments)
            vector<string>& GetArgs();

    /// Set some user pointer
            void            SetUserInfo(void * userData);
    /// Set the user pointer
            void*           GetUserInfo();

    /// Gets the loaded interfaces
            const vector<RobotInterface*>&  GetRobotInterfaces();


    /// Check for simulation mode
            bool            IsSimulationMode();

    /// Sets up a ros node handle for this robot
            ros::NodeHandle*  InitializeROS();
            ros::NodeHandle*  InitializeROS(string NAME);



};
typedef Robot *pRobot;



#endif
