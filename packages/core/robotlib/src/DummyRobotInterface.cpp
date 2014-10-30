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

#include "DummyRobotInterface.h"

DummyRobotInterface::DummyRobotInterface():RobotInterface(true){
}
DummyRobotInterface::~DummyRobotInterface(){
}
RobotInterface::Status DummyRobotInterface::RobotInit(){
    mSensorsGroup.SetSensorsList(mRobot->GetSensors());
    mActuatorsGroup.SetActuatorsList(mRobot->GetActuators());
    return STATUS_OK;
}
RobotInterface::Status DummyRobotInterface::RobotFree(){
    return STATUS_OK;
}
RobotInterface::Status DummyRobotInterface::RobotStart(){
    return STATUS_OK;
}
RobotInterface::Status DummyRobotInterface::RobotStop(){
    return STATUS_OK;
}
RobotInterface::Status DummyRobotInterface::RobotUpdate(){
    return STATUS_OK;
}
RobotInterface::Status DummyRobotInterface::RobotUpdateCore(){
    //mRobot->GetTimeSensor().Set(GetClock().GetTime());

    mSensorsGroup.MapFromActuators(mActuatorsGroup);
    return STATUS_OK;
}
void DummyRobotInterface::UpdateActuators(){
}
void DummyRobotInterface::UpdateSensors(){
}
