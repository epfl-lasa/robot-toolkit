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

#include "kuka_polish_module.h"


kuka_polish_module::kuka_polish_module()
:RobotInterface(){
}
kuka_polish_module::~kuka_polish_module(){
}

RobotInterface::Status kuka_polish_module::RobotInit(){
    return STATUS_OK;
}
RobotInterface::Status kuka_polish_module::RobotFree(){
    return STATUS_OK;
}
RobotInterface::Status kuka_polish_module::RobotStart(){
    return STATUS_OK;
}    
RobotInterface::Status kuka_polish_module::RobotStop(){
    return STATUS_OK;
}
RobotInterface::Status kuka_polish_module::RobotUpdate(){
    return STATUS_OK;
}
RobotInterface::Status kuka_polish_module::RobotUpdateCore(){
    return STATUS_OK;
}
int kuka_polish_module::RespondToConsoleCommand(const string cmd, const vector<string> &args){
    return 0;
}


extern "C"{
    // These two "C" functions manage the creation and destruction of the class
    kuka_polish_module* create(){return new kuka_polish_module();}
    void destroy(kuka_polish_module* module){delete module;}
}

