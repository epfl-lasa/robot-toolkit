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

#include "ExampleWorldControlModule.h"


ExampleWorldControlModule::ExampleWorldControlModule()
:WorldInterface(){
}
ExampleWorldControlModule::~ExampleWorldControlModule(){
}

WorldInterface::Status ExampleWorldControlModule::WorldInit(){
    AddConsoleCommand("impulse");
    AddConsoleCommand("impulse2");
    AddConsoleCommand("showPos");
    return STATUS_OK;
}
WorldInterface::Status ExampleWorldControlModule::WorldFree(){
    return STATUS_OK;
}
WorldInterface::Status ExampleWorldControlModule::WorldStart(){
    mImpulseTime = 0;
    return STATUS_OK;
}    
WorldInterface::Status ExampleWorldControlModule::WorldStop(){
    return STATUS_OK;
}
WorldInterface::Status ExampleWorldControlModule::WorldUpdate(){
    if(GetWorld()){
        // Move object called FixBallC
        pWorldObject obj = GetWorld()->Find("FixBallC");
        if(obj){
            Vector3 pos;
            double angle = DEG2RAD(45)*sin(GetClock().GetTime());
            pos(0) = 0.8*sin(angle);
            pos(1) = 0.5;
            pos(2) = -0.3+0.8*cos(angle);

            obj->GetReferenceFrame().SetOrigin() = pos;
            Matrix3 orient;
	        orient.RotationX(angle);
            obj->GetReferenceFrame().SetOrient() = orient;
        }
        obj = GetWorld()->Find("RA");
        if(obj){
            obj->AddExternalForce(mExtForce);
            if(GetClock().GetTime()-mImpulseTime > 0.2){
                mExtForce.Zero();
            }
        }
    }
    return STATUS_OK;
}
WorldInterface::Status ExampleWorldControlModule::WorldUpdateCore(){
    return STATUS_OK;
}
int ExampleWorldControlModule::RespondToConsoleCommand(const string cmd, const vector<string> &args){
    if(cmd=="impulse"){
        mExtForce.Set(0,0,20);
        mImpulseTime=GetClock().GetTime();
    }
    else if(cmd=="impulse2"){
        mExtForce.Set(0,-10,0);
        mImpulseTime=GetClock().GetTime();
    }
    else if(cmd=="showPos"){
        pWorldObject obj = GetWorld()->Find("RA");
        if(obj){
            Vector3 pos = obj->GetReferenceFrame().GetOrigin();
            char txt[256];
            sprintf(txt,"Object RA is at: %f %f %f",pos.x(),pos.y(),pos.z());
            GetConsole()->Print(txt);
        }
    }
    return 0;
}


extern "C"{
    // These two "C" functions manage the creation and destruction of the class
    ExampleWorldControlModule* create(){return new ExampleWorldControlModule();}
    void destroy(ExampleWorldControlModule* module){delete module;}
}

