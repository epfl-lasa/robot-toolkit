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

#include "ExampleForceSensorModule.h"


ExampleForceSensorModule::ExampleForceSensorModule()
:RobotInterface(){
}
ExampleForceSensorModule::~ExampleForceSensorModule(){
}

RobotInterface::Status ExampleForceSensorModule::RobotInit(){
    mSensorsGroup.SetSensorsList(mRobot->GetSensors());
    mActuatorsGroup.SetActuatorsList(mRobot->GetActuators());

    mForceSensor = ForceSensorJoint6DOFSensor::Cast(mRobot->FindSensor("FORCE_SENSOR"));

    // Inverse dynamics
    mInvDynamics.SetRobot(mRobot);
    mInvDynamics.Init();
    mInvDynamics.SetGravityCompensationOnly(true);

    mJointTorques.Resize(mRobot->GetDOFCount());
    mJointTarget.Resize(mRobot->GetDOFCount());
    mJointCurrentTarget.Resize(mRobot->GetDOFCount());

    // PID
    XmlTree tree;
    tree.LoadFromFile("./data/Misc/WAMDefaultPID.xml");
    mPIDCtrl.Init(&tree);

    AddConsoleCommand("Rest");
    AddConsoleCommand("GetReady");
    AddConsoleCommand("Press");
    AddConsoleCommand("ForceValue");

    bPrintForce = false;

    return STATUS_OK;
}
RobotInterface::Status ExampleForceSensorModule::RobotFree(){
    return STATUS_OK;
}
RobotInterface::Status ExampleForceSensorModule::RobotStart(){
    mState = 0;
    bFirst = true;

    return STATUS_OK;
}    
RobotInterface::Status ExampleForceSensorModule::RobotStop(){
    return STATUS_OK;
}
RobotInterface::Status ExampleForceSensorModule::RobotUpdate(){
    return STATUS_OK;
}
RobotInterface::Status ExampleForceSensorModule::RobotUpdateCore(){

    mSensorsGroup.ReadSensors();

    if(bPrintForce){
        ostringstream ss;
        ss << Vector(mForceSensor->mForce.GetLinearComponent()) << "\t"<< Vector(mForceSensor->mForce.GetAngularComponent());
        GetConsole()->Print(ss.str());

        //mRobot->GetLinks()[mRobot->GetLinkIndex("HAPTIC_BALL")]->mForce.Print();
        
        //mForceSensor->mForce.Print();
        //mForceSensor = ForceSensorJoint6DOFSensor::Cast(mRobot->FindSensor("FORCE_SENSOR"));

        //GetConsole()->mForceSensor->mForce.GetAngularComponent().Print();
        //mForceSensor->mForce.GetLinearComponent().Print();
        bPrintForce = false;
    }


    if(bFirst){
        mJointCurrentTarget = mSensorsGroup.GetJointAngles();
        mPIDCtrl.SetInput(mSensorsGroup.GetJointAngles());
        mPIDCtrl.SetTarget(mSensorsGroup.GetJointAngles());
        mPIDCtrl.Start();
        bFirst = false;
    }


    switch(mState){
    case 0:
        {
            double target[] = {0.0,-110.0,0.0,175.0,0.0,0.0,0.0};
            mJointTarget.Set(target,7);
            mJointTarget *= PI/180.0;
        }
        break;
    case 1:
        {
            double target[] = {0.0,45.0,0.0,90.0,0.0,-45.0,0.0};
            mJointTarget.Set(target,7);
            mJointTarget *= PI/180.0;
            break;
        }
    case 2:
        {
            double target[] = {0.0,50.0,0.0,70.0,0.0,-30.0,0.0};
            mJointTarget.Set(target,7);
            mJointTarget *= PI/180.0;
            break;
        }
    }


    mJointCurrentTarget += (mJointTarget - mJointCurrentTarget) * GetClock().GetDt()/2.0;

    mInvDynamics.SetGravityCompensationOnly(true);
    mInvDynamics.Update();
    mInvDynamics.GetTorques(mJointTorques);

    mPIDCtrl.SetInput(mSensorsGroup.GetJointAngles());
    mPIDCtrl.SetTarget(mJointCurrentTarget);
    mPIDCtrl.Update(GetClock().GetDt());

    mJointTorques += mPIDCtrl.GetOutput();


    // Sends order to robot
    mActuatorsGroup.SetJointTorques(mJointTorques);
    mActuatorsGroup.WriteActuators();

    return STATUS_OK;
}
int ExampleForceSensorModule::RespondToConsoleCommand(const string cmd, const vector<string> &args){
    if(cmd=="Rest"){
        mState = 0;
        bFirst = true;
    }else if(cmd=="GetReady"){
        mState = 1;
        bFirst = true;
    }else if(cmd=="Press"){
        mState = 2;
        bFirst = true;
    }else if(cmd=="ForceValue"){
        bPrintForce = true;
    }

    return 0;
}


extern "C"{
    // These two "C" functions manage the creation and destruction of the class
    ExampleForceSensorModule* create(){return new ExampleForceSensorModule();}
    void destroy(ExampleForceSensorModule* module){delete module;}
}

