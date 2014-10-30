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

#include "Robot.h"


#include "SimulatorDynamicsInterface.h"
#include <limits.h>
#include "StdTools/LogStream.h"
#include "RobotLib/WorldInterface.h"

SimulatorDynamicsInterface::SimulatorDynamicsInterface()
:WorldInterface(true){
    mPeriod = 0.001;
    SetName("Simulation");
    AddConsoleCommand("ResetAll");
    AddConsoleCommand("ResetWorld");
    AddConsoleCommand("Pause");
    AddConsoleCommand("Run");
    AddConsoleCommand("Step");
    AddConsoleCommand("SetRunSpeed");

    mPPSChrono.Start();
    mPPS        = 0;
    mPPSCounter = 0;

    mProcessingPeriod = 0.001;
    mRunTime          = 0.0;

    mRunChrono.Start();
    mRunChrono.Pause();

    bIsPausing      = false;
    mTimeToPause    = 0.0;
    bIsPaused       = true;

    mRunSpeed = 1.0;
    mRunSpeedOffset = 0.0;

    mGravity.Set(0.0,0.0,-9.81);
}
SimulatorDynamicsInterface::~SimulatorDynamicsInterface(){
    FreeWorld();
}

WorldInterface::Status SimulatorDynamicsInterface::WorldInit(){
    mSimulationTime = 0;
    mPeriodCounter  = 0;
    mPeriodOffset   = 0;
    mRunSpeed = 1.0;
    mRunSpeedOffset = 0.0;


    mSimulatorDynamics.SetGravity(mGravity);
    mSimulatorDynamics.SetDamping(mDamping);
    mSimulatorDynamics.Init();
    mSimulatorDynamics.LinkToWorld(GetWorld());
    return STATUS_OK;
}
WorldInterface::Status SimulatorDynamicsInterface::WorldFree(){
    mSimulatorDynamics.Free();
    FreeWorld();
    return STATUS_OK;
}
WorldInterface::Status SimulatorDynamicsInterface::WorldStart(){

    mSimulatorDynamics.Reset();
    return STATUS_OK;
}
WorldInterface::Status SimulatorDynamicsInterface::WorldStop(){
    return STATUS_OK;
}
WorldInterface::Status SimulatorDynamicsInterface::WorldUpdate(){
    return STATUS_OK;
}
WorldInterface::Status SimulatorDynamicsInterface::WorldUpdateCore(){
    mSimulatorDynamics.Update(GetClock().GetDt());

    return STATUS_OK;
}
int SimulatorDynamicsInterface::RespondToCommand(const string cmd, const vector<string> &args){
    return 0;
}

bool SimulatorDynamicsInterface::LoadConfig(pXmlTree config){
    if(mWorld!=NULL){
        FreeWorld();
    }
    if(config==NULL){
        return false;
    }


    XmlTree args;
    pXmlTree tree;
    pXmlTreeList tlist,tlist2;
    char modName[256];
    Robot *cRobot;
    Robot *csRobot;
    vector<string> nameAndPatches;

    bool bError         = false;

    FileFinder::ClearAdditionalPaths();
    if((tree = config->Find("UserPath"))!=NULL){
        string path = config->Find("UserPath")->GetData();
        if(path.length()>0){
            FileFinder::AddAdditionalPath(path);
            gLOG.AppendToEntry("Messages","Using user path: <%s>",path.c_str());
        }else{
            gLOG.AppendToEntry("Messages","Warning: Bad user path specified %s",path.c_str());
        }
    }

    if((tree = config->Find("Packages"))!=NULL){
        vector<string> paths = Tokenize(config->Find("Packages")->GetData());
        int cnt=0;
        for(unsigned int i=0;i<paths.size();i++){
            string path = string("./data/packages/")+paths[i];
            paths[i] = path;
            if(path.length()>0){
                FileFinder::AddAdditionalPath(path);
                cnt++;
            }
        }
        if(cnt>0){
             gLOG.AppendToEntry("Messages","Using package path(s): <%s>",Serialize(paths).c_str());
        }
    }


    mWorld = new World();
    //Loading world
    if((tree = config->Find("World"))!=NULL){
        if(tree->GetData().length()>0){
            if(!mWorld->Load(tree->GetData())){
                gLOG.AppendToEntry("Messages","Error while opening and reading world file: %s",tree->GetData().c_str());
                bError = true;
                FreeWorld();
                return false;
            }
        }
        // World modules
        tlist = tree->GetSubTrees();
        for(int i=0;i<int(tlist->size());i++){
            if(tlist->at(i)->GetName() =="WorldModule"){
                sprintf(modName,"module/%s.so",tlist->at(i)->GetData().c_str());
                if(FileFinder::Find(modName)){
                    ModuleInterface *interface = ModuleInterface::LoadInterface(FileFinder::GetCStr());
                    if(dynamic_cast<WorldInterface*>(interface)!=NULL){
                        interface->SetOptionTree(tlist->at(i));
                        interface->SetAutoPerformanceTiming(true);
                        mWorldInterfaces.push_back((WorldInterface*)interface);
                        mWorld->AddInterface((WorldInterface*)interface);
                        if(tlist->at(i)->Get("Name",string("")).length()>0){
                            interface->SetName(string("World_")+tlist->at(i)->Get("Name",string("")));
                        }else{
                            interface->SetName(string("World"));
                        }
                        GetConsole()->AddConsole(interface->GetConsole());
                        gLOG.AppendToEntry("Messages","World module <%s> sucessfully loaded",modName);
                    }else if(interface!=NULL){
                        ModuleInterface::CloseInterface(interface);
                        gLOG.AppendToEntry("Messages","Error: Module <%s> is not a world module",modName);
                        bError = true;
                    }else{
                        gLOG.AppendToEntry("Messages","Error: Failed to load world module <%s>",modName);
                        bError = true;
                    }
                }else{
                    gLOG.AppendToEntry("Messages","Error: Failed to find world module <%s>",modName);
                    bError = true;
                }
            }
        }
    }

    if(bError){
        FreeWorld();
        return false;
    }


    //Loading robots
    tlist = config->GetSubTrees();
    for(int i=0;i<int(tlist->size());i++){
        if(tlist->at(i)->GetName() =="Robot"){
            tree = tlist->at(i);
            nameAndPatches = Tokenize(RemoveSpaces(tree->GetData()));
            if(nameAndPatches.size()>0){
                cRobot = new Robot(true);
                if(!cRobot->Load(nameAndPatches[0], Serialize(nameAndPatches,1))){
                    gLOG.AppendToEntry("Messages","Error: While operning and reading robot file <%s>",nameAndPatches[0].c_str());
                    delete cRobot; cRobot = NULL;
                    bError = true;
                }else{
                    cRobot->SetArgs(Tokenize(tree->Get("Args",string(""))));
                    cRobot->SetName(tree->CGet("Name",string("Unknown")));
                    cRobot->SetWorld(mWorld);
                    cRobot->SetUserInfo(this);
                    mRobots.push_back(cRobot);

                    csRobot = NULL;
                    if(tree->Find("SimulationRobot")){
                        nameAndPatches = Tokenize(RemoveSpaces(tree->Find("SimulationRobot")->GetData()));
                        if(nameAndPatches.size()>0){
                            csRobot = new Robot(true);
                            csRobot->Load(nameAndPatches[0],Serialize(nameAndPatches,1));
                            mSimRobots.push_back(csRobot);
                        }else{
                            gLOG.AppendToEntry("Messages","Error: No simulation robot type defined");
                            bError = true;
                        }
                    }

                    if(tree->Find("ControlMode")){
                        string cmode = tree->Get("ControlMode",string(""));
                        if(cmode == "position"){
                            cRobot->SetControlMode( Robot::CTRLMODE_POSITION);
                            if(csRobot) csRobot->SetControlMode(Robot::CTRLMODE_POSITION);
                        }else if(cmode == "velocity"){
                            cRobot->SetControlMode( Robot::CTRLMODE_VELOCITY);
                            if(csRobot) csRobot->SetControlMode(Robot::CTRLMODE_VELOCITY);
                        }else if(cmode == "acceleration"){
                            cRobot->SetControlMode( Robot::CTRLMODE_ACCELERATION);
                            if(csRobot) csRobot->SetControlMode(Robot::CTRLMODE_ACCELERATION);
                        }else if(cmode == "torque"){
                            cRobot->SetControlMode( Robot::CTRLMODE_TORQUE);
                            if(csRobot) csRobot->SetControlMode(Robot::CTRLMODE_TORQUE);
                        }else{
                            gLOG.AppendToEntry("Messages","Error: Bad control mode defined.");
                            bError = true;
                            FreeWorld();
                            return false;
                        }
                    }

                    tlist2 = tree->GetSubTrees();
                    for(int j=0;j<int(tlist2->size());j++){
                        if(tlist2->at(j)->GetName() =="RobotModule"){
                            sprintf(modName,"module/%s.so",tlist2->at(j)->GetData().c_str());
                            if(FileFinder::Find(modName)){
                                ModuleInterface *interface = ModuleInterface::LoadInterface(FileFinder::GetCStr());
                                if(dynamic_cast<RobotInterface*>(interface)!=NULL){
                                    interface->SetOptionTree(tlist2->at(j));
                                    interface->SetAutoPerformanceTiming(true);
                                    mRobotInterfaces.push_back((RobotInterface*)interface);
                                    cRobot->AddInterface((RobotInterface*)interface);
                                    if(tlist2->at(j)->Get("Name",string("")).length()>0){
                                        interface->SetName(cRobot->GetName()+string("_")+tlist2->at(j)->Get("Name",string("")));
                                    }else{
                                        interface->SetName(cRobot->GetName());
                                    }
                                    GetConsole()->AddConsole(interface->GetConsole());
                                    gLOG.AppendToEntry("Messages","Robot module <%s> sucessfully loaded",modName);
                                }else if(interface!=NULL){
                                    ModuleInterface::CloseInterface(interface);
                                    gLOG.AppendToEntry("Messages","Error: Module <%s> is not a robot module",modName);
                                    bError = true;
                                }else{
                                    gLOG.AppendToEntry("Messages","Error: Failed to load robot module <%s>",modName);
                                    bError = true;
                                }
                            }else{
                                gLOG.AppendToEntry("Messages","Error: Failed to find robot module <%s>",modName);
                                bError = true;
                            }
                        }
                        if(bError){
                            FreeWorld();
                            return false;
                        }
                    }

                    // Setting up robot object and put it into the world
                    WorldObject* wObject = new WorldObject();
                    //WorldObject* wObject = cRobot->GetLinks()[0]; //new WorldObject();
                    wObject->SetName(cRobot->GetName());
                    wObject->SetRobot(cRobot);
                    if(tree->Find("Origin")){
                        int size;
                        double *array;
                        size=tree->GetArray("Origin",&array);
                        if(size==3){
                            wObject->GetReferenceFrame(true).SetOrigin().Set(array);
                        }else{
                            cerr <<  "Error: Bad robot <Origin> array size (should be 3)"<<endl;
                        }
                    }
                    if(tree->Find("Orient")){
                        int size;
                        double *array;
                        size=tree->GetArray("Orient",&array);
                        if(size==9){
                            wObject->GetReferenceFrame(true).SetOrient().Set(array);
                            wObject->GetReferenceFrame(true).SetOrient().Normalize();
                        }else if(size==3){
                            wObject->GetReferenceFrame(true).SetOrient().SRotationV(Vector3(array));
                        }else{
                            cerr << "Error: Bad robot <Orient> array size (should be 3(axis*angle) or 9(full rotation matrix))"<<endl;
                        }
                    }
                    wObject->SetToInitialState();
                    wObject->SetInitialState();

                    mWorld->AddObject(wObject,true);
                }
            }else{
                gLOG.AppendToEntry("Messages","Error: No robot type defined");
                bError = true;
            }
        }
    }

    mPeriod = config->Get("Simulator.Period",0.001);
    double scal = config->Get("Simulator.MassScaling",10.0);
    if(scal<=0.0) scal = 1.0;
    mSimulatorDynamics.sGlobalScaling = scal;




    mGravity.Set(0.0,0.0,-9.81);
    if(config->Find("Simulator.Gravity")){
        int size;
        double *array;
        size=config->GetArray("Simulator.Gravity",&array);
        if(size==3){
            mGravity.Set(array);
        }else{
            cerr <<  "Error: Bad <Gravity> vector size (should be 3)"<<endl;
        }
    }

    mDamping = 0;
    if(config->Find("Simulator.Damping")){
    	mDamping = config->Get("Simulator.Damping", 0.0);
    }

    if(bError){
        FreeWorld();
        return false;
    }

    mRunTime          = 0.0;

    mRunChrono.Start();
    mRunChrono.Pause();
    bIsPausing  = false;
    bIsPaused   = true;

    return true;
}

void SimulatorDynamicsInterface::FreeWorld(){

    GetConsole()->ClearConsoles();

    if(mWorld!=NULL) delete mWorld;
    mWorld = NULL;

    for(int i=0;i<int(mSimRobots.size());i++)
        delete mSimRobots[i];
    mSimRobots.clear();

    for(int i=0;i<int(mRobots.size());i++)
        delete mRobots[i];
    mRobots.clear();

   for(int i=0;i<int(mRobotInterfaces.size());i++)
        ModuleInterface::CloseInterface(mRobotInterfaces[i]);
    mRobotInterfaces.clear();

    for(int i=0;i<int(mWorldInterfaces.size());i++)
        ModuleInterface::CloseInterface(mWorldInterfaces[i]);
    mWorldInterfaces.clear();
}

void SimulatorDynamicsInterface::SetPeriod(double period){
    if(mPeriod!=period){
        mPeriodOffset += double(mPeriodCounter)*mPeriod;
        mPeriodCounter = 0;
        mPeriod = period;
    }
}


double SimulatorDynamicsInterface::GetSimulationTime(){
    return mSimulationTime;
}
double SimulatorDynamicsInterface::GetSimulationRunTime(){
    if(mRunSpeed>=0.0){
        return mRunSpeedOffset + double(mRunChrono.ElapsedTimeMs()*1e-3)*mRunSpeed;
    }else{
        return -1;
    }
}
int SimulatorDynamicsInterface::GetPPS(){
    return mPPS;
}
double SimulatorDynamicsInterface::GetPTime(){
    return mProcessingTime.GetTime();
}


int SimulatorDynamicsInterface::SimulationStep(double targetTime, double maxProcTime){
    if(targetTime>mSimulationTime){
        double steps = targetTime/mPeriod;
        //cout << targetTime <<" "<<mSimulationTime<<" "<<steps<<" "<<int(steps)<<endl;
        return SimulationStep((steps>=1.0?int(steps):int(1)),maxProcTime);
    }
    return 0;
}

int SimulatorDynamicsInterface::SimulationStep(int nbSteps, double maxProcTime){
    Chrono mChrono;
    int steps = 0;
    double cProcTime = 0;

    while(nbSteps!=0){
        mChrono.Start();

        mInternalClock.SetTimeDt(mSimulationTime,mPeriod);
        mInternalClock.Update();

        UpdateCore();
        Update();

        mPeriodCounter++;
        if(mPeriodCounter > (UINT_MAX<<2)){
            mPeriodOffset += double(mPeriodCounter)*mPeriod;
            mPeriodCounter = 0;
        }
        mSimulationTime = mPeriodOffset + double(mPeriodCounter)*mPeriod;

        steps++;

        double mctime = mChrono.ElapsedTimeUs();
        cProcTime += mctime;
        if(cProcTime+0.8*mctime > maxProcTime){
            break;
        }
        nbSteps--;
    }

    return steps;
}


int SimulatorDynamicsInterface::RespondToConsoleCommand(const string cmd, const vector<string> &args){
    // Answering to console command

    if(cmd=="ResetAll"){
        mSimulatorDynamics.AllowRobotReset(true);
        Stop();
        Start();
    }else if(cmd=="ResetWorld"){
        mSimulatorDynamics.AllowRobotReset(false);
        Stop();
        Start();
    }else if(cmd=="Run"){
        Run();
    }else if(cmd=="Pause"){
        Pause();
    }else if(cmd=="Step"){
        Step(0.1);
    }else if(cmd=="SetRunSpeed"){
        if(args.size()>0){
            double rs = atof(args[0].c_str());
            SetRunSpeed(rs);
            char txt[256];
            sprintf(txt,"Setting run speed to %f",rs);
            GetConsole()->Print(txt);
        }else{
            GetConsole()->Print("Error: double argument required");
        }
        
    }
    return 0;
}


void                SimulatorDynamicsInterface::Process(double maxDesiredProcTime){
   REALTYPE realTime = mRunSpeedOffset + double(mRunChrono.ElapsedTimeUs()*1e-6)*mRunSpeed;

    if(bIsPausing){
        if(mRunTime>=mTimeToPause){
            bIsPausing  = false;
            bIsPaused   = true;
            mRunChrono.Pause();
        }
    }

    
    if(!bIsPaused){
        double maxTotalProcessingTime = maxDesiredProcTime;

        int steps = 0;
        mChrono.Start();
        if(mRunSpeed>=0.0){
            //cout << realTime<<" "<<mRunTime<<" "<<mRunChrono.ElapsedTimeUs()<<" "<<mRunChrono.ElapsedTimeUs()*1e-6<<endl;
            if(realTime-mRunTime>0.0){
                steps = SimulationStep(realTime,maxTotalProcessingTime);
            }
        }else{
            steps = SimulationStep(-1,maxTotalProcessingTime);
        }
        double mctime = mChrono.ElapsedTimeUs();

        if(steps>0){
            mProcessingTime.AddMeasurement(REALTYPE(mctime/double(steps)));
            mRunTime = GetSimulationTime();
            mPPSCounter+=steps;
        }
    }
    if(mPPSChrono.ElapsedTimeMs()>1000){
        mPPS        = MAX(0,mPPSCounter-1);
        mPPSCounter = 0;
        mPPSChrono.Start();
    }
}
void                SimulatorDynamicsInterface::Run(){
    if(bIsPaused)
        mRunChrono.Resume();
    bIsPaused = false;
    bIsPausing = false;

}
void                SimulatorDynamicsInterface::Pause(){
    if(!bIsPaused)
        mRunChrono.Pause();
    bIsPaused = true;
    bIsPausing = false;
}
void                SimulatorDynamicsInterface::Step(double dt){
    //mTimeToPause = mRunTime +(double(mRunChrono.ElapsedTimeMs()))*1e-3+dt;
    mTimeToPause = mRunTime + dt;
    mRunChrono.Resume();
    bIsPaused   = false;
    bIsPausing  = true;
}
bool                SimulatorDynamicsInterface::IsRunning(){
    return !bIsPaused;
}
void                SimulatorDynamicsInterface::SetRunSpeed(double factor){
    if(factor>=0.0)
        mRunSpeed= factor;
    else
        mRunSpeed= -1.0;
    bool paused = mRunChrono.IsPaused();
    mRunChrono.Start();
    if(paused)
        mRunChrono.Pause();

    mRunSpeedOffset = mRunTime;

}
