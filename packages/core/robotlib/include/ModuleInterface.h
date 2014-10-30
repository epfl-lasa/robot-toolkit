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

#ifndef MODULEINTERFACE_H_
#define MODULEINTERFACE_H_

#include "RobotTools.h"
#include "StdTools/Console.h"
#include "StdTools/Various.h"
#include "StdTools/XmlTree.h"
#include "StdTools/Timer.h"


/// Generic class inferface to be loaded as module.
class ModuleInterface
{
public:
    enum Status      {STATUS_ERROR=0,
                      STATUS_OK};
    enum SystemState {SYSSTATE_NONE=0,
                      SYSSTATE_STOPPED,
                      SYSSTATE_STARTED};

private:
    SystemState             mSystemState;

private:

    /// Module name
    string              mInterfaceName;
    /// Module special name
    string              mName;
    /// Pointer to an external clock
    Clock               *mClock;
    /// Pointer to a console
    Console             *mConsole;
    /// Pointer to a internal default console
    Console             *mInternalConsole;
    /// Pointer to an option tree
    XmlTree             *mOptionTree;

    /// Define if this interface is the main application controller
    bool                bIsControlInterface;
    /// Define if automatic performance timing is enabled
    bool                bEnableAutoTiming;

    /// Performance estimators
    PerformanceEstimator    mPerformanceEstimatorUpdate;
    PerformanceEstimator    mPerformanceEstimatorUpdateCore;

protected:
    /// Internal clock
    Clock               mInternalClock;

public:
    /// Constructor
            ModuleInterface(bool isControlInterface = false);
    /// Destructor
    virtual ~ModuleInterface();

    /// Return current system state
            SystemState         GetSystemState();

    /// Function to be called on init
            Status              Init();
    /// Function to be called on free
            Status              Free();
    /// Function to be called on before running
            Status              Start();
    /// Function to be called on stopping
            Status              Stop();
    /// Function to be called on update
            Status              Update();
    /// Function to be called on core update
            Status              UpdateCore();

    /// Function to be called on opengl drawing requests
            void                Draw();

    /// Subfunction to be called on init
    virtual Status              InterfaceInit()          = 0;
    /// Subfunction to be called on free
    virtual Status              InterfaceFree()          = 0;
    /// Subfunction to be called on before running
    virtual Status              InterfaceStart()         = 0;
    /// Subfunction to be called on stopping
    virtual Status              InterfaceStop()          = 0;
    /// Subfunction to be called on update
    virtual Status              InterfaceUpdate()        = 0;
    /// Subfunction to be called on core update
    virtual Status              InterfaceUpdateCore()    = 0;

    /// Subfunction to be called on drawing requests
    virtual void                InterfaceDraw()          = 0;

    /// Tell if this interface is a main application control interface
            bool                IsControlInterface();

    /// Get the pointer to the internal console (if given, NULL by default)
            Console*            GetConsole();
    /// Set the pointer to the internal console
            void                SetConsole(Console* console);
    /// Add a valid console command (other methods are possible using the Console pointer directly)
            void                AddConsoleCommand(const string cmd);
    /// Virtual function to respond to conole commands
    virtual int                 RespondToConsoleCommand(const string cmd, const vector<string> &args);
    /// Enable auto performance timing
            void                SetAutoPerformanceTiming(bool enable);
    /// Get performance estimator for the Update loop
            const PerformanceEstimator& GetPerformanceEstimatorUpdate();
    /// Get performance estimator for the UpdateCore loop
            const PerformanceEstimator& GetPerformanceEstimatorUpdateCore();

    /// Get the module interface name (if loaded from a module, the filename is the default)
            string              GetInterfaceName();
    /// Set the module interface name
            void                SetInterfaceName(string name);
    /// Get the module name 
            string              GetName();
    /// Set the module name
            void                SetName(string name);
    /// Set an external clock
            void                SetExternalClock(Clock *clock);
    /// Get current clock
            const Clock&        GetClock();

    /// Set an option tree (the tree will be cloned)
            void                SetOptionTree(pXmlTree tree);
    /// Get the option tree
            pXmlTree            GetOptionTree();
    /// Get

private:
    void                        *mInterfaceModuleHandle;
    ModuleInterface*            (*mInterfaceModuleConstructor)();
    void                        (*mInterfaceModuleDestructor)(ModuleInterface*);

public:
    /// Static function that load the interface from a module file
    static  ModuleInterface*    LoadInterface(const char* filename);
    /// Static function that close the interface module
    static  void                CloseInterface(ModuleInterface* interface);


private:
    static  ModuleInterface*    sMainControlInterface;

public:
    /// Generic console command class that will send Console::Command directly to the ModuleInterface
    class ModuleInterfaceCommand : public Console::Command
    {
    protected:
        ModuleInterface  *mInterface;
    public:
        /// Constructor
                                    ModuleInterfaceCommand(string name, ModuleInterface* interface);
        /// Destructor
        virtual                     ~ModuleInterfaceCommand();
        /// Get the module interface linked to the command
                ModuleInterface*    GetInterface();
        /// Send the command to the RespondToConsoleCommand function of the ModuleInterface
        virtual int                 Execute(string args);
    };

    class ModuleInterfaceMutex
    {
    public:
                    ModuleInterfaceMutex();
        virtual     ~ModuleInterfaceMutex();

        virtual bool        Lock();
        virtual bool        Unlock();
    };

private:
    ModuleInterfaceMutex   *mUpdateCoreMutex;
    ModuleInterfaceMutex   *mUpdateMutex;
public:
    static  ModuleInterfaceMutex*   CreateUpdateMutex(bool coreMutex = false);
    static  void                    DestroyUpdateMutex(ModuleInterfaceMutex** mutex,bool coreMutex = false);
protected:
    virtual ModuleInterfaceMutex*   LocalCreateUpdateMutex(bool coreMutex = false);
    virtual void                    LocalDestroyUpdateMutex(ModuleInterfaceMutex** mutex,bool coreMutex = false);

};


#endif
