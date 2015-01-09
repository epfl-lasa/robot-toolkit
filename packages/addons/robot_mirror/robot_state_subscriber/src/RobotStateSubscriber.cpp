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

#include "RobotStateSubscriber.h"


RobotStateSubscriber::RobotStateSubscriber()
:RobotInterface(){
}
RobotStateSubscriber::~RobotStateSubscriber(){
}

RobotInterface::Status RobotStateSubscriber::RobotInit(){

    jointPositions.Resize(mRobot->GetDOFCount());
    actuators.SetActuatorsList(mRobot->GetActuators());


    string nodename = mRobot->GetName();
    nodename += "_MIRROR";
    nh = mRobot->InitializeROS(nodename);
    string topicName = mRobot->GetName();
    topicName += "/JointState";

    jointStateSubscriber = nh->subscribe(topicName,100,&RobotStateSubscriber::jointStateCallback,this);
    return STATUS_OK;
}
RobotInterface::Status RobotStateSubscriber::RobotFree(){
    return STATUS_OK;
}
RobotInterface::Status RobotStateSubscriber::RobotStart(){
    return STATUS_OK;
}    
RobotInterface::Status RobotStateSubscriber::RobotStop(){
    return STATUS_OK;
}
RobotInterface::Status RobotStateSubscriber::RobotUpdate(){
    ros::spinOnce();
    //jointPositions.Print();
    actuators.SetJointPositions(jointPositions);
    actuators.WriteActuators();
    return STATUS_OK;
}
RobotInterface::Status RobotStateSubscriber::RobotUpdateCore(){
    return STATUS_OK;
}
int RobotStateSubscriber::RespondToConsoleCommand(const string cmd, const vector<string> &args){
    return 0;
}

void RobotStateSubscriber::jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
    for(int i=0;i<mRobot->GetDOFCount();i++){
        jointPositions(i) = msg->position[i];

    }

}


extern "C"{
    // These two "C" functions manage the creation and destruction of the class
    RobotStateSubscriber* create(){return new RobotStateSubscriber();}
    void destroy(RobotStateSubscriber* module){delete module;}
}

