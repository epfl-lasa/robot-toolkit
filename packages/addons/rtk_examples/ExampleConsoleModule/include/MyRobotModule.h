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

#ifndef MyRobotModule_H_
#define MyRobotModule_H_

#include "RobotLib/RobotInterface.h"

class MyRobotModule : public RobotInterface
{
public:
            MyRobotModule();
    virtual ~MyRobotModule();
  
    virtual Status              RobotInit();
    virtual Status              RobotFree();
  
    virtual Status              RobotStart();    
    virtual Status              RobotStop();
  
    virtual Status              RobotUpdate();
    virtual Status              RobotUpdateCore();

    virtual int                 RespondToConsoleCommand(const string cmd, const vector<string> &args);

            void                RespondToCustomCommand();
public:

    class MyCommand : public ModuleInterfaceCommand
    {
    public:
                    MyCommand(string name, ModuleInterface* interface);
        virtual     ~MyCommand();
        virtual int Execute(string args);
    };

};



#endif 
