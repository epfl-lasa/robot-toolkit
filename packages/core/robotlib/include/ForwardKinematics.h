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

#ifndef __FORWARDKINEMATICS_H__
#define __FORWARDKINEMATICS_H__

#include "Link.h"

class Robot;

class ForwardKinematics
{
protected:
    Robot              *mRobot;
    LinksList          *mLinks;
    JointsList         *mJoints;  
    vector<int>        *mParents;
  
    unsigned int        mLinksCount;

    ReferenceFrame    **mFullKinematicRefFrames;
    bool               *bFullKinematicRefFramesValid;
    bool               *bFullKinematicRefFramesCreated;

    ReferenceFrame    **mWorldKinematicRefFrames;
    bool               *bWorldKinematicRefFramesValid;
    bool               *bWorldKinematicRefFramesCreated;
    

public:
            ForwardKinematics();
    virtual ~ForwardKinematics();

            void              Free();
            
            void              Update();

            ReferenceFrame&   GetReferenceFrame(unsigned int link); 
            ReferenceFrame&   GetReferenceFrame(unsigned int fromLink, unsigned int toLink); 
  
            void              SetRobot(Robot *robot);
          

protected:
            void              BuildFullKinematicRefFrames();
            void              ResetFullKinematicRefFrames();
};

typedef ForwardKinematics *pForwardKinematics;

#include "Robot.h"

#endif
