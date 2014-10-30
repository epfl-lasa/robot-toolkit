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

#include "EmptyRobotModule.h"


EmptyRobotModule::EmptyRobotModule()
:RobotInterface(){
}
EmptyRobotModule::~EmptyRobotModule(){
}

RobotInterface::Status EmptyRobotModule::RobotInit(){
    return STATUS_OK;
}
RobotInterface::Status EmptyRobotModule::RobotFree(){
    return STATUS_OK;
}
RobotInterface::Status EmptyRobotModule::RobotStart(){
    return STATUS_OK;
}    
RobotInterface::Status EmptyRobotModule::RobotStop(){
    return STATUS_OK;
}
RobotInterface::Status EmptyRobotModule::RobotUpdate(){
    return STATUS_OK;
}
RobotInterface::Status EmptyRobotModule::RobotUpdateCore(){
    return STATUS_OK;
}
int EmptyRobotModule::RespondToConsoleCommand(const string cmd, const vector<string> &args){
    return 0;
}


extern "C"{
    // These two "C" functions manage the creation and destruction of the class
    EmptyRobotModule* create(){return new EmptyRobotModule();}
    void destroy(EmptyRobotModule* module){delete module;}
}

