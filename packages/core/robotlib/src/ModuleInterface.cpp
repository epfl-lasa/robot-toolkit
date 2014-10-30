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

#include "ModuleInterface.h"
#include <stdio.h>
#include <dlfcn.h>

ModuleInterface* ModuleInterface::sMainControlInterface = NULL;

ModuleInterface::ModuleInterface(bool isControlInterface){
    if(isControlInterface){
        if(sMainControlInterface!=NULL){
            fprintf(stderr,"Error: One single control interface can be set within an application\n");
            fprintf(stderr,"Exiting...\n");
            exit(-1);
        }
        sMainControlInterface = this;
    }

    mInterfaceModuleDestructor  = NULL;
    mInterfaceModuleConstructor = NULL;
    mInterfaceModuleHandle      = NULL;

    mClock                      = NULL;
    mInternalConsole            = new Console;
    mConsole                    = mInternalConsole;
    mOptionTree                 = NULL;
    bIsControlInterface         = isControlInterface;

    mSystemState                = SYSSTATE_NONE;

    mUpdateCoreMutex            = CreateUpdateMutex(true);
    mUpdateMutex                = CreateUpdateMutex(false);

    bEnableAutoTiming           = false;
}
ModuleInterface::~ModuleInterface(){
    if(mOptionTree) delete mOptionTree; mOptionTree = NULL;
    if(mInternalConsole) delete mInternalConsole; mInternalConsole = NULL;
    DestroyUpdateMutex(&mUpdateCoreMutex,true);
    DestroyUpdateMutex(&mUpdateMutex,false);
}


bool ModuleInterface::IsControlInterface(){
    return bIsControlInterface;
}
Console* ModuleInterface::GetConsole(){
    return mConsole;
}
void ModuleInterface::SetConsole(Console* console){
    if(console!=NULL)
        mConsole = console;
    else
        mConsole = mInternalConsole;
}
void ModuleInterface::AddConsoleCommand(const string cmd){
    if(mConsole!=NULL)
        mConsole->AddCommand(new ModuleInterfaceCommand(cmd, this),true);
}
int ModuleInterface::RespondToConsoleCommand(const string cmd, const vector<string> &args){
    return 0;
}
void ModuleInterface::SetExternalClock(Clock *clock){
    mClock = clock;
}
const Clock& ModuleInterface::GetClock(){
    if(mClock!=NULL)
        return *mClock;
    else
        return mInternalClock;
}
void ModuleInterface::SetOptionTree(pXmlTree tree){
    if(mOptionTree) delete mOptionTree; mOptionTree = NULL;
    if(tree!=NULL)
        mOptionTree = tree->Clone();
}
pXmlTree ModuleInterface::GetOptionTree(){
    return mOptionTree;
}


ModuleInterface::SystemState ModuleInterface::GetSystemState(){
    return mSystemState;
}
void                    ModuleInterface::SetAutoPerformanceTiming(bool enable){
    bEnableAutoTiming = enable;
}
const PerformanceEstimator&   ModuleInterface::GetPerformanceEstimatorUpdate(){
    return mPerformanceEstimatorUpdate;
}
const PerformanceEstimator&   ModuleInterface::GetPerformanceEstimatorUpdateCore(){
    return mPerformanceEstimatorUpdateCore;
}
string              ModuleInterface::GetInterfaceName(){
    return mInterfaceName;
}
void                ModuleInterface::SetInterfaceName(string name){
    mInterfaceName = name;
}
string              ModuleInterface::GetName(){
    return mName;
}
void                ModuleInterface::SetName(string name){
    mName = name;    
    mConsole->SetName(name);
}


ModuleInterface::Status ModuleInterface::Init(){
    Status res = STATUS_ERROR;
    switch(mSystemState){
    case SYSSTATE_NONE:
        if(!bIsControlInterface){
            if(sMainControlInterface!=NULL){
                if(sMainControlInterface->mClock!=NULL) SetExternalClock(sMainControlInterface->mClock);
                else                                    SetExternalClock(&sMainControlInterface->mInternalClock);
            }
        }
        res = InterfaceInit();
        if(res==STATUS_OK){
            mSystemState = SYSSTATE_STOPPED;
        }
        break;
    case SYSSTATE_STOPPED:
        res = STATUS_OK;
        break;
    case SYSSTATE_STARTED:
        res = STATUS_OK;
        break;
    }
    return res;
}

ModuleInterface::Status ModuleInterface::Free(){
    Status res = STATUS_ERROR;
    switch(mSystemState){
    case SYSSTATE_NONE:
        res = STATUS_OK;
        break;
    case SYSSTATE_STOPPED:
        res = InterfaceFree();
        break;
    case SYSSTATE_STARTED:
        res = Stop();
        if(res==STATUS_OK){
            res = InterfaceFree();
        }
        break;
    }
    if(res==STATUS_OK){
        mSystemState = SYSSTATE_NONE;
    }
    return res;
}


ModuleInterface::Status ModuleInterface::Start(){
    Status res = STATUS_ERROR;
    switch(mSystemState){
    case SYSSTATE_NONE:
        res = STATUS_ERROR;
        break;
    case SYSSTATE_STOPPED:
        res = InterfaceStart();
        if(res==STATUS_OK){
            mSystemState = SYSSTATE_STARTED;
        }
        break;
    case SYSSTATE_STARTED:
        res = STATUS_OK;
        break;
    }
    return res;
}

ModuleInterface::Status ModuleInterface::Stop(){
    Status res = STATUS_ERROR;
    switch(mSystemState){
    case SYSSTATE_NONE:
        res = STATUS_ERROR;
        break;
    case SYSSTATE_STOPPED:
        res = STATUS_OK;
        break;
    case SYSSTATE_STARTED:
        res = InterfaceStop();
        if(res==STATUS_OK){
            mSystemState = SYSSTATE_STOPPED;
        }
        break;
    }
    return res;
}


ModuleInterface::Status ModuleInterface::Update(){
    Status res = STATUS_ERROR;
    if(mSystemState == SYSSTATE_STARTED){
        if(mUpdateMutex) mUpdateMutex->Lock();
        if(bEnableAutoTiming) mPerformanceEstimatorUpdate.Tic();
        res = InterfaceUpdate();
        if(bEnableAutoTiming) mPerformanceEstimatorUpdate.Toc();
        if(mUpdateMutex) mUpdateMutex->Unlock();
    }
    return res;
}
ModuleInterface::Status ModuleInterface::UpdateCore(){
    Status res = STATUS_ERROR;
    if(mSystemState == SYSSTATE_STARTED){
        if(bIsControlInterface){
            if(mClock==NULL) mInternalClock.Update();
        }
        if(mUpdateCoreMutex) mUpdateCoreMutex->Lock();
        if(bEnableAutoTiming) mPerformanceEstimatorUpdateCore.Tic();
        res = InterfaceUpdateCore();
        if(bEnableAutoTiming) mPerformanceEstimatorUpdateCore.Toc();
        if(mUpdateCoreMutex) mUpdateCoreMutex->Unlock();
    }
    return res;
}

void ModuleInterface::Draw(){
    if(mSystemState == SYSSTATE_STARTED){
        if(mUpdateMutex) mUpdateMutex->Lock();
        InterfaceDraw();
        if(mUpdateMutex) mUpdateMutex->Unlock();
    }
}







ModuleInterface*     ModuleInterface::LoadInterface(const char* filename){
    void *handle;
    ModuleInterface*           (*constructor)();
    void                      (*destructor)(ModuleInterface*);

    handle = dlopen(filename, RTLD_LAZY);
    //cout << "Loading interface "<<filename<<endl;
    if (!handle){
        fprintf(stderr, "Error while opening interface module: %s\n", filename);
        fprintf(stderr, "  %s\n",dlerror());
        return NULL;
    }
    constructor = (ModuleInterface* (*)())     dlsym(handle , "create");
    if (!constructor) {
        fprintf(stderr, "Error while getting create function in module %s\n", filename);
        fprintf(stderr, "  %s\n",dlerror());
        if(dlclose(handle)!=0){
            fprintf(stderr, "Error while closing module\n");
            fprintf(stderr, "  %s\n",dlerror());
        }
        return NULL;
    }
    destructor  = (void (*)(ModuleInterface*)) dlsym(handle , "destroy");
    if (!destructor) {
        fprintf(stderr, "Error while getting destroy function in module %s\n", filename);
        fprintf(stderr, "  %s\n",dlerror());
        if(dlclose(handle)!=0){
            fprintf(stderr, "Error while closing module\n");
            fprintf(stderr, "  %s\n",dlerror());
        }
        return NULL;
    }

    ModuleInterface *interface = constructor();
    if(dynamic_cast<ModuleInterface*>(interface)!=NULL){
        if(interface){
            interface->mInterfaceModuleHandle       = handle;
            interface->mInterfaceModuleConstructor  = constructor;
            interface->mInterfaceModuleDestructor   = destructor;
            string fname = GetFileFromFilename(filename);
            if(fname.length()>3)
                interface->mInterfaceName           = fname.substr(0,fname.length()-3);
            else
                interface->mInterfaceName           = fname;
        }else{
            fprintf(stderr, "Error while creating interface from module %s\n", filename);
            if(dlclose(handle)!=0){
                fprintf(stderr, "Error while closing module\n");
                fprintf(stderr, "  %s\n",dlerror());
            }
            return NULL;
        }
    }else{
        fprintf(stderr, "Error while creating interface from module %s\n", filename);
        fprintf(stderr, "  Constructor does not provide a ModuleInterface\n");
        destructor(interface);
        if(dlclose(handle)!=0){
            fprintf(stderr, "Error while closing module\n");
            fprintf(stderr, "  %s\n",dlerror());
        }
        return NULL;
    }
    return interface;
}

void                ModuleInterface::CloseInterface(ModuleInterface* interface){
    if(interface!=NULL){
        void * handle = interface->mInterfaceModuleHandle;
        if(interface->mInterfaceModuleDestructor){
            interface->mInterfaceModuleDestructor(interface);
        }
        if(handle){
            if(dlclose(handle)!=0){
                fprintf(stderr, "Error while closing module\n");
                fprintf(stderr, "  %s\n",dlerror());
            }
        }
    }
}


ModuleInterface::ModuleInterfaceMutex*   ModuleInterface::CreateUpdateMutex(bool coreMutex){
    if(sMainControlInterface) return sMainControlInterface->LocalCreateUpdateMutex(coreMutex);
    return NULL;
}
void ModuleInterface::DestroyUpdateMutex(ModuleInterface::ModuleInterfaceMutex** mutex, bool coreMutex){
    if(sMainControlInterface) sMainControlInterface->LocalDestroyUpdateMutex(mutex,coreMutex);
}
ModuleInterface::ModuleInterfaceMutex*   ModuleInterface::LocalCreateUpdateMutex(bool coreMutex){
    return new ModuleInterfaceMutex();
}
void ModuleInterface::LocalDestroyUpdateMutex(ModuleInterfaceMutex** mutex, bool coreMutex){
    if(mutex!=NULL){
        if(*mutex!=NULL){
            delete *mutex;
            mutex = NULL;
        }
    }
}


ModuleInterface::ModuleInterfaceCommand::ModuleInterfaceCommand(string name, ModuleInterface* interface)
:Command(name){
    mInterface = interface;
}
ModuleInterface::ModuleInterfaceCommand::~ModuleInterfaceCommand(){
}
ModuleInterface*    ModuleInterface::ModuleInterfaceCommand::GetInterface(){
    return mInterface;
}

int ModuleInterface::ModuleInterfaceCommand::Execute(string args){
    if(mInterface!=NULL){
        if(mInterface->mUpdateMutex) mInterface->mUpdateMutex->Lock();
        if(mInterface->mUpdateCoreMutex) mInterface->mUpdateCoreMutex->Lock();
        int res = mInterface->RespondToConsoleCommand(m_Name, Tokenize(args));
        if(mInterface->mUpdateCoreMutex) mInterface->mUpdateCoreMutex->Unlock();
        if(mInterface->mUpdateMutex) mInterface->mUpdateMutex->Unlock();
        return res;
    }
    return 0;
}

ModuleInterface::ModuleInterfaceMutex::ModuleInterfaceMutex(){}
ModuleInterface::ModuleInterfaceMutex::~ModuleInterfaceMutex(){}

bool        ModuleInterface::ModuleInterfaceMutex::Lock(){return true;}
bool        ModuleInterface::ModuleInterfaceMutex::Unlock(){return true;}
