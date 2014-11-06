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

#include "RobotStatePublisher.h"


RobotStatePublisher::RobotStatePublisher()
    :RobotInterface(){
}
RobotStatePublisher::~RobotStatePublisher(){
}

RobotInterface::Status RobotStatePublisher::RobotInit(){
    nh = mRobot->InitializeROS();
    string topicName;
    topicName = mRobot->GetName();
    topicName += "/JointState";
    jointStatePublisher = nh->advertise<sensor_msgs::JointState>(topicName,100);
    topicName = mRobot->GetName();
    topicName += "/Pose";
    posePublisher = nh->advertise<geometry_msgs::PoseStamped>(topicName,100);

    mSensorsGroup.SetSensorsList(mRobot->GetSensors());

    jointStateMsg.position.resize(mRobot->GetDOFCount());
    jointStateMsg.velocity.resize(mRobot->GetDOFCount());
    jointStateMsg.effort.resize(mRobot->GetDOFCount());

    topicName = mRobot->GetName();
    topicName += "/CoreRate";
    //rosrt::init();
    //ratePublisher = new rosrt::Publisher<std_msgs::Empty>(*nh,topicName,1,10,std_msgs::Empty());
    //emptyMsg  = ratePublisher->allocate();


    return STATUS_OK;
}
RobotInterface::Status RobotStatePublisher::RobotFree(){
    return STATUS_OK;
}
RobotInterface::Status RobotStatePublisher::RobotStart(){
    return STATUS_OK;
}    
RobotInterface::Status RobotStatePublisher::RobotStop(){
    return STATUS_OK;
}
RobotInterface::Status RobotStatePublisher::RobotUpdate(){


    for(int ii =0;ii<3;ii++){
        for( int jj = 0;jj<3;jj++){
            rot_Eigen(ii,jj) = rot_MathLib(ii,jj);
        }
    }
    //cout<<rot_Eigen<<endl;
    //rot_MathLib.Print();
    Eigen::Quaternion<double> rot_quat(rot_Eigen);
    //cout<<rot_quat.w()<<endl;
    //cout<<rot_quat.toRotationMatrix()<<endl;
    for(int i=0;i<mRobot->GetDOFCount();i++){
        jointStateMsg.position[i] = mSensorsGroup.GetJointAngles()(i);
        jointStateMsg.velocity[i] = mSensorsGroup.GetJointVelocities()(i);
        jointStateMsg.effort[i] = mSensorsGroup.GetJointTorques()(i);
    }

    jointStateMsg.header.stamp = ros::Time::now();

    poseStampedMsg.pose.position.x = currEEPos(0);
    poseStampedMsg.pose.position.y = currEEPos(1);
    poseStampedMsg.pose.position.z = currEEPos(2);
    poseStampedMsg.pose.orientation.w = rot_quat.w();
    poseStampedMsg.pose.orientation.x = rot_quat.x();
    poseStampedMsg.pose.orientation.y = rot_quat.y();
    poseStampedMsg.pose.orientation.z = rot_quat.z();

    poseStampedMsg.header.stamp = ros::Time::now();


    jointStatePublisher.publish(jointStateMsg);
    posePublisher.publish(poseStampedMsg);

    return STATUS_OK;
}
RobotInterface::Status RobotStatePublisher::RobotUpdateCore(){
    //ratePublisher->publish(emptyMsg);
    // not safe for realtime
    mSensorsGroup.ReadSensors();
    currEEPos  = mRobot->GetReferenceFrame(mRobot->GetLinksCount()-1,0).GetOrigin();
    rot_MathLib = mRobot->GetReferenceFrame(mRobot->GetLinksCount()-1,0).GetOrient();
    return STATUS_OK;
}
int RobotStatePublisher::RespondToConsoleCommand(const string cmd, const vector<string> &args){
    return 0;
}


extern "C"{
// These two "C" functions manage the creation and destruction of the class
RobotStatePublisher* create(){return new RobotStatePublisher();}
void destroy(RobotStatePublisher* module){delete module;}
}

