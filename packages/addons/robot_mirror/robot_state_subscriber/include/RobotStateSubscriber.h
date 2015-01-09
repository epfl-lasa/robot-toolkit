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

#ifndef RobotStateSubscriber_H_
#define RobotStateSubscriber_H_

#include "RobotLib/RobotInterface.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseStamped.h"

class RobotStateSubscriber : public RobotInterface
{

    ros::NodeHandle * nh;
    ros::Subscriber jointStateSubscriber;

    RevoluteJointActuatorGroup actuators;
    Vector jointPositions;

public:
    RobotStateSubscriber();
    virtual ~RobotStateSubscriber();

    virtual Status              RobotInit();
    virtual Status              RobotFree();

    virtual Status              RobotStart();
    virtual Status              RobotStop();

    virtual Status              RobotUpdate();
    virtual Status              RobotUpdateCore();

    virtual int                 RespondToConsoleCommand(const string cmd, const vector<string> &args);

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr & msg);
};



#endif 
