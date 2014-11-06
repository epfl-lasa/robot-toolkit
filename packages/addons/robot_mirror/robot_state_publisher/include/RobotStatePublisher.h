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

#ifndef RobotStatePublisher_H_
#define RobotStatePublisher_H_

#include "RobotLib/RobotInterface.h"
#include "ros/ros.h"
//#include "rosrt/rosrt.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Empty.h"
#include "eigen3/Eigen/Dense"

class RobotStatePublisher : public RobotInterface
{

    ros::Publisher jointStatePublisher;
    ros::Publisher posePublisher;
    ros::NodeHandle * nh;
    //a realtime publisher that tuns in realtime loop so that the rate of updatecore can be measured by rostopic hz
    //    rosrt::Publisher<std_msgs::Empty> * ratePublisher;
    std_msgs::EmptyPtr emptyMsg;
    sensor_msgs::JointState jointStateMsg;
    geometry_msgs::PoseStamped poseStampedMsg;

    RevoluteJointSensorGroup mSensorsGroup;
    Vector3 currEEPos;
    Eigen::Matrix3d rot_Eigen;
   // Eigen::Quaternion<double> rot_quat;
    Matrix3 rot_MathLib;

public:
            RobotStatePublisher();
    virtual ~RobotStatePublisher();
  
    virtual Status              RobotInit();
    virtual Status              RobotFree();
  
    virtual Status              RobotStart();    
    virtual Status              RobotStop();
  
    virtual Status              RobotUpdate();
    virtual Status              RobotUpdateCore();

    virtual int                 RespondToConsoleCommand(const string cmd, const vector<string> &args);
};



#endif 
