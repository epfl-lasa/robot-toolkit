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

#include "WorldInterface.h"

WorldInterface::WorldInterface(bool isControlInterface)
:ModuleInterface(isControlInterface){
    mWorld = NULL;
}

WorldInterface::~WorldInterface(){
}
void  WorldInterface::SetWorld(World* world){
    if(GetSystemState() == SYSSTATE_NONE)
        mWorld = world;
}

WorldInterface::Status WorldInterface::InterfaceFree(){
    Status res = WorldFree();
    if(res==STATUS_OK){
        if(IsControlInterface()){
            if(mWorld)
                mWorld->Free();
        }
    }
    return res;
}

WorldInterface::Status WorldInterface::InterfaceInit(){
    Status res = STATUS_ERROR;
    if(mWorld!=NULL){
        res = WorldInit();
    }
    if(res==STATUS_OK){
        if(IsControlInterface()){
            mWorld->Init();
        }
    }
    return res;
}

WorldInterface::Status WorldInterface::InterfaceStart(){
    Status res = STATUS_ERROR;
    res = WorldStart();
    if(res==STATUS_OK){
        if(IsControlInterface()){
            mWorld->Start();
        }
    }
    return res;
}

WorldInterface::Status WorldInterface::InterfaceStop(){
    Status res = STATUS_ERROR;
    res = WorldStop();
    if(res==STATUS_OK){
        if(IsControlInterface()){
            mWorld->Stop();
        }
    }
    return res;
}


WorldInterface::Status WorldInterface::InterfaceUpdate(){
    Status res = STATUS_ERROR;
    res = WorldUpdate();
    if(res==STATUS_OK){
        if(IsControlInterface()){
            mWorld->Update();
        }
    }
    return res;
}
WorldInterface::Status WorldInterface::InterfaceUpdateCore(){
    Status res = STATUS_ERROR;
    res = WorldUpdateCore();
    if(res==STATUS_OK){
        if(IsControlInterface()){
            mWorld->UpdateCore();
        }
    }
    return res;
}
void WorldInterface::InterfaceDraw(){
    WorldDraw();
    if(IsControlInterface()){
        mWorld->Draw();
    }
}

World* WorldInterface::GetWorld(){
    return mWorld;
}


WorldInterface::Status WorldInterface::WorldInit()             {return STATUS_OK;}
WorldInterface::Status WorldInterface::WorldFree()             {return STATUS_OK;}
WorldInterface::Status WorldInterface::WorldStart()            {return STATUS_OK;}
WorldInterface::Status WorldInterface::WorldStop()             {return STATUS_OK;}
WorldInterface::Status WorldInterface::WorldUpdate()           {return STATUS_OK;}
WorldInterface::Status WorldInterface::WorldUpdateCore()       {return STATUS_OK;}
void                   WorldInterface::WorldDraw()             {}

int WorldInterface::RespondToCommand(const string cmd, const vector<string> &args){return 0;}
