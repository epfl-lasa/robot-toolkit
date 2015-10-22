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


    mLWRRobot = (LWRRobot*)mRobot; 
 
    nh = mRobot->InitializeROS();
    string topicName;
    topicName = mRobot->GetName();

    GetConsole()->Print("Robot Init Publisher");
    float new_sampling_time = 0.002;
    mLWRRobot->SetSamplingTime(new_sampling_time);
    dt = mLWRRobot->GetSamplingTime();
    std::ostringstream ss;
    ss << "DT: " << dt;
    std::string msg(ss.str());
    GetConsole()->Print(msg);

    // topicName += "/JointState";
    string JointStateName = "/joint_states";

    jointStatePublisher = nh->advertise<sensor_msgs::JointState>(JointStateName,100);
    topicName = mRobot->GetName();
    topicName += "/Pose";
    posePublisher = nh->advertise<geometry_msgs::PoseStamped>(topicName,100);

    topicName = mRobot->GetName();
    topicName += "/FT";
    ftPublisher = nh->advertise<geometry_msgs::WrenchStamped>(topicName,3);

    topicName = mRobot->GetName();
    topicName += "/Stiff";
    stiffPublisher = nh->advertise<geometry_msgs::TwistStamped>(topicName,3);


    mSensorsGroup.SetSensorsList(mRobot->GetSensors());

    jointStateMsg.position.resize(mRobot->GetDOFCount());
    jointStateMsg.velocity.resize(mRobot->GetDOFCount());
    jointStateMsg.effort.resize(mRobot->GetDOFCount());
    jointStateMsg.name.resize(mRobot->GetDOFCount());
    char buf[255];
    pXmlTree options = GetOptionTree();
    string which_arm;
    if(options) {
        which_arm = options->CGet("Options.Arm", string("right"));
    } else {
        which_arm = "right";
    }
    cout<<"Using as "<<which_arm<<" arm";

    for(int i=0; i<mRobot->GetDOFCount(); ++i) {
        sprintf(buf, "%s_arm_%d_joint",which_arm.c_str(), i);
        jointStateMsg.name[i] = buf;
    }



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


     // New message for Cart FT
     ftMsg.header.stamp = ros::Time::now();
     ftMsg.wrench.force.x = eeFT[0];
     ftMsg.wrench.force.y = eeFT[1];
     ftMsg.wrench.force.z = eeFT[2];
     ftMsg.wrench.torque.x = eeFT[3];
     ftMsg.wrench.torque.y = eeFT[4];
     ftMsg.wrench.torque.z = eeFT[5];
     
     // New message for Cart Stiffness
     stiffMsg.header.stamp = ros::Time::now();
     stiffMsg.twist.linear.x = eeStiff[0];
     stiffMsg.twist.linear.y = eeStiff[1];
     stiffMsg.twist.linear.z = eeStiff[2];
     stiffMsg.twist.angular.x = eeStiff[3];
     stiffMsg.twist.angular.y = eeStiff[4];
     stiffMsg.twist.angular.z = eeStiff[5];

     ftPublisher.publish(ftMsg);
     stiffPublisher.publish(stiffMsg);
    return STATUS_OK;
}

RobotInterface::Status RobotStatePublisher::RobotUpdateCore(){
    //ratePublisher->publish(emptyMsg);
    // not safe for realtime
    mSensorsGroup.ReadSensors();
    currEEPos  = mRobot->GetReferenceFrame(mRobot->GetLinksCount()-1,0).GetOrigin();
    rot_MathLib = mRobot->GetReferenceFrame(mRobot->GetLinksCount()-1,0).GetOrient();

    // New Cartesian Measurements
    eeFT = ((LWRRobot*)mRobot)->GetEstimatedExternalCartForces();
    eeStiff = ((LWRRobot*)mRobot)->GetCartStiffness();


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

