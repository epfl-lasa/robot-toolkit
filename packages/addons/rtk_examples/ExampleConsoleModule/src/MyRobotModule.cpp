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

#include "MyRobotModule.h"
#include "StdTools/Various.h"


MyRobotModule::MyRobotModule()
:RobotInterface(){
}
MyRobotModule::~MyRobotModule(){
}

RobotInterface::Status MyRobotModule::RobotInit(){

    // Did someone give us a console?
    if(GetConsole()){
        // The simplest way to add a command that will be sent in RespondToConsoleCommand
        AddConsoleCommand("echo");
        AddConsoleCommand("time");

        // Here's another way (AddConsoleCommand is just a wrapper to this line)
        //  set the second parameter to true if you want the console to take care of deleting
        //  the newly created command upon exit
        GetConsole()->AddCommand(new ModuleInterfaceCommand("argsCount"  , this),true);
        GetConsole()->AddCommand(new ModuleInterfaceCommand("argsList"   , this),true);

        AddConsoleCommand("testScript");

        // Yet another way using a custom command class
        GetConsole()->AddCommand(new MyCommand("test"       , this),true);

        // Let's print out some stuff on screem
        GetConsole()->Print("Well, you may type in some commands");
        GetConsole()->Print("<Tab> for auto completion ;)");
    }


    return STATUS_OK;
}
RobotInterface::Status MyRobotModule::RobotFree(){
    return STATUS_OK;
}
RobotInterface::Status MyRobotModule::RobotStart(){
    return STATUS_OK;
}
RobotInterface::Status MyRobotModule::RobotStop(){
    return STATUS_OK;
}
RobotInterface::Status MyRobotModule::RobotUpdate(){
    return STATUS_OK;
}
RobotInterface::Status MyRobotModule::RobotUpdateCore(){
    return STATUS_OK;
}
int MyRobotModule::RespondToConsoleCommand(const string cmd, const vector<string> &args){
    // Answering to console command
    if(GetConsole()){
        if(cmd=="echo"){
            GetConsole()->Print(Serialize(args));
        }else if(cmd=="time"){
            char txt[256];
            sprintf(txt,"Current time is %f",GetClock().GetTime());
            GetConsole()->Print(txt);
        }else if(cmd=="test"){
            GetConsole()->Print("This is a test");
        }else if(cmd=="argsCount"){
            char txt[256];
            sprintf(txt,"Found %lu arguments",args.size());
            GetConsole()->Print(txt);
        }else if(cmd=="argsList"){
            for(size_t i=0;i<args.size();i++){
                GetConsole()->Print(args[i]);
            }
        }else if(cmd=="testScript"){
            GetConsole()->Execute("echo running test script",false);
            GetConsole()->Execute("test",false);
            GetConsole()->Execute("argsCount a b c d",false);
            GetConsole()->Execute("argsList a b c d",false);
        }
    }
    return 0;
}
void MyRobotModule::RespondToCustomCommand(){
    if(GetConsole()){
        GetConsole()->Print("Responding to a custom command");
        GetConsole()->Print("  using a custom function called");
        GetConsole()->Print("  from a custom class...");
    }
}




MyRobotModule::MyCommand::MyCommand(string name, ModuleInterface* interface)
:ModuleInterfaceCommand(name,interface){}

MyRobotModule::MyCommand::~MyCommand(){}

int MyRobotModule::MyCommand::Execute(string args){
    MyRobotModule *interface = dynamic_cast<MyRobotModule*>(GetInterface());
    if(interface!=NULL){
        interface->RespondToCustomCommand();
    }
    return 0;
}



extern "C"{
   // These two "C" functions manage the creation and destruction of the class
   MyRobotModule* create()                    {return new MyRobotModule();}
   void           destroy(MyRobotModule* mod) {delete mod;}
}

