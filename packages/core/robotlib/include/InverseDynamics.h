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

#ifndef INVERSEDYNAMICSCONTROLLER_H_
#define INVERSEDYNAMICSCONTROLLER_H_

#include "MathLib/MathLib.h"
#include "StdTools/Timer.h"

#include "Sensor.h"
#include "Actuator.h"
#include "Robot.h"

class InverseDynamics
{
protected:
    Robot          *mRobot;
    LinksList      *mLinks;
    JointsList     *mJoints;
    unsigned int    mLinksCount;

    Vector3         mGravity;
    bool            bGravityCompensationOnly;

    int             mFrictionCompensationMethod;
    REALTYPE        mFrictionCompensationRatio;

    Vector          mOutputTorques;

    Matrix          mJSIM;
    TMatrix<6>     *m_Ic;


    SpatialForce   *mExternalForces;

public:
            InverseDynamics();
    virtual ~InverseDynamics();

            void    Init(XmlTree *tree = NULL);
            void    Free();

            void    Update();
            void    UpdateForward();
            void    UpdateBackward();

            void    SetForce(unsigned int linkId, const Vector3& force);
            void    SetWrench(unsigned int linkId, const Vector& force);

            void    SetGravity(const Vector3& gravity);
            void    SetRobot(pRobot robot);
            void    SetGravityCompensationOnly(bool only);

            void    SetFrictionCompensation(int method, REALTYPE ratio = 1.0);

            void    GetTorques(Vector & output);
            void    AddTorques(Vector & result);
            

            Matrix& ComputeJSIM();
            void ClearWrenches();
};


#endif
