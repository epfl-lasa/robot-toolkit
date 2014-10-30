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

#ifndef SENSOR_H_
#define SENSOR_H_

#include <string>
#include <vector>
using namespace std;

#include "MathLib/MathLib.h"
using namespace MathLib;
#include "StdTools/BaseObject.h"
#include "Actuator.h"


class Sensor;
typedef Sensor *pSensor;
typedef vector<pSensor> SensorsList;

BASEOBJECT_TOPCLASS(Sensor)

protected:
    string  mName;
public:
            Sensor();
    virtual ~Sensor();

            void      SetName(string name);
            string    GetName();

    virtual void      Zero();
    virtual void      Set(const Sensor& sensor);
    virtual void      Set(const Actuator& actuator);

    virtual int         StreamSize();
    virtual int         SetStream(void* memory);
    virtual int         SetFromStream(const void* memory);

    static  int         SensorsListStreamSize(SensorsList & list);
    static  int         SensorsListSetStream(SensorsList & list,void* memory);
    static  int         SensorsListSetFromStream(SensorsList & list,const void* memory);
};


BASEOBJECT_CLASS(Generic1DOFJointSensor,Sensor)

public:
    REALTYPE    mPosition;
    REALTYPE    mVelocity;
    REALTYPE    mAcceleration;
    REALTYPE    mTorque;
    REALTYPE    mLimits[2];

public:
            Generic1DOFJointSensor();
    virtual ~Generic1DOFJointSensor();

    virtual void        Zero();
    virtual void        Set(const Sensor& sensor);
    virtual void        Set(const Actuator& actuator);

            void        Set(REALTYPE position, REALTYPE velocity, REALTYPE acceleration, REALTYPE torque);
            void        SetLimits(REALTYPE limitLow, REALTYPE limitHigh);

    virtual int         StreamSize();
    virtual int         SetStream(void* memory);
    virtual int         SetFromStream(const void* memory);
};
typedef Generic1DOFJointSensor *pGeneric1DOFJointSensor;

typedef vector<pGeneric1DOFJointSensor> Generic1DOFJointSensorsList;

class Generic1DOFJointSensorGroup
{
protected:
    Generic1DOFJointSensorsList     mSensorList;
    int                             mSensorCount;

    Vector mPosition;
    Vector mVelocity;
    Vector mAcceleration;
    Vector mTorque;
    Vector mLimits[2];
public:
    Generic1DOFJointSensorGroup();
    ~Generic1DOFJointSensorGroup();

    void        SetSensorsList(const SensorsList& list);
    void        ReadSensors();
    void        WriteSensors();

    int         GetCount();

    Vector&     GetJointAngles();
    Vector&     GetJointPositions();
    Vector&     GetJointVelocities();
    Vector&     GetJointAccelerations();
    Vector&     GetJointTorques();
    Vector&     GetJointForces();

    Vector&     GetJointLimitsHigh();
    Vector&     GetJointLimitsLow();

    void        SetJointAngles(const Vector& vector);
    void        SetJointPositions(const Vector& vector);
    void        SetJointVelocities(const Vector& vector);
    void        SetJointTorques(const Vector& vector);
    void        SetJointForces(const Vector& vector);
    void        SetJointAccelerations(const Vector& vector);

    void        MapFromActuators(const Generic1DOFJointActuatorGroup& actuators);
};


#define RevoluteJointSensor           Generic1DOFJointSensor
#define pRevoluteJointSensor          pGeneric1DOFJointSensor
#define RevoluteJointSensorsList      Generic1DOFJointSensorsList
#define RevoluteJointSensorGroup      Generic1DOFJointSensorGroup

#define SliderJointSensor             Generic1DOFJointSensor
#define pSliderJointSensor            pGeneric1DOFJointSensor



BASEOBJECT_CLASS(ForceSensorJoint6DOFSensor,Sensor)

public:
    SpatialForce    mForce;

public:
            ForceSensorJoint6DOFSensor();
    virtual ~ForceSensorJoint6DOFSensor();

    virtual void        Zero();
    virtual void        Set(const Sensor& sensor);
    virtual void        Set(const Actuator& actuator);

            void        Set(const SpatialForce &force);
            void        SetLimits(const SpatialForce &limitLow,const SpatialForce &limitHigh);

    virtual int         StreamSize();
    virtual int         SetStream(void* memory);
    virtual int         SetFromStream(const void* memory);
};
typedef ForceSensorJoint6DOFSensor *pForceSensorJoint6DOFSensor;



BASEOBJECT_CLASS(ContactJointSensor,Sensor)

public:
    REALTYPE    mValue;

public:
            ContactJointSensor();
    virtual ~ContactJointSensor();

    virtual void        Zero();
    virtual void        Set(const Sensor& sensor);
    virtual void        Set(const Actuator& actuator);

            void        Set(const REALTYPE value);

    virtual int         StreamSize();
    virtual int         SetStream(void* memory);
    virtual int         SetFromStream(const void* memory);
};
typedef ContactJointSensor *pContactJointSensor;


BASEOBJECT_CLASS(KeyPressSensor,Sensor)

public:
  char      mKeyPressed;

public:
          KeyPressSensor();
  virtual ~KeyPressSensor();

  virtual void      Zero();
  virtual void      Set(const Sensor& sensor);

          void      Set(char key);
          char      GetKey();

    virtual int         StreamSize();
    virtual int         SetStream(void* memory);
    virtual int         SetFromStream(const void* memory);
};
typedef KeyPressSensor *pKeyPressSensor;

BASEOBJECT_CLASS(TimeSensor,Sensor)

public:
  REALTYPE  mTime;

public:
          TimeSensor();
  virtual ~TimeSensor();

  virtual void      Zero();
  virtual void      Set(const Sensor& sensor);

          void      Set(REALTYPE time);
          REALTYPE  GetTime();

    virtual int         StreamSize();
    virtual int         SetStream(void* memory);
    virtual int         SetFromStream(const void* memory);
};
typedef TimeSensor *pTimeSensor;


#endif /*SENSOR_H_*/
