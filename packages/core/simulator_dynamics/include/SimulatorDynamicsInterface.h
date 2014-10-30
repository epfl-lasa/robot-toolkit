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

#ifndef SIMULATORDYNAMICSINTERFACE_H_
#define SIMULATORDYNAMICSINTERFACE_H_

#include "RobotLib/WorldInterface.h"
#include "SimulatorDynamics.h"

class SimulatorDynamicsInterface : public WorldInterface
{
protected:
    vector<Robot*>                              mRobots;
    vector<Robot*>                              mSimRobots;
    vector<RobotInterface*>                     mRobotInterfaces;
    vector<WorldInterface*>                     mWorldInterfaces;

    double                                      mPeriod;
    double                                      mSimulationTime;
    unsigned int                                mPeriodCounter;
    double                                      mPeriodOffset;


    Chrono                      mChrono;
    PerformanceEstimator        mProcessingTime;
    Chrono                      mPPSChrono;
    int                         mPPS;
    int                         mPPSCounter;


    REALTYPE                    mProcessingPeriod;
    REALTYPE                    mRunTime;
    Chrono                      mRunChrono;

    double                      mRunSpeed;
    double                      mRunSpeedOffset;

    bool                        bIsPaused;
    bool                        bIsPausing;
    double                      mTimeToPause;

    Vector3                     mGravity;
    double						mDamping;

public:
    SimulatorDynamics      mSimulatorDynamics;

public:
            SimulatorDynamicsInterface();
    virtual ~SimulatorDynamicsInterface();

    virtual Status              WorldInit();
    virtual Status              WorldFree();

    virtual Status              WorldStart();
    virtual Status              WorldStop();

    virtual Status              WorldUpdate();
    virtual Status              WorldUpdateCore();

    virtual int                 RespondToCommand(const string cmd, const vector<string> &args);


            void                SetPeriod(double period);

            bool                LoadConfig(pXmlTree config);
            double              GetSimulationTime();
            double              GetSimulationRunTime();
            double              GetPTime();
            int                 GetPPS();

            int                 SimulationStep(double targetTime, double maxProcTime);
            int                 SimulationStep(int nbSteps, double maxProcTime);

            void                Process(double maxDesiredProcTime);
            void                Run();
            void                Pause();
            void                Step(double dt);
            bool                IsRunning();


            void                SetRunSpeed(double factor);

            int                 RespondToConsoleCommand(const string cmd, const vector<string> &args);
protected:
            void                FreeWorld();            
};



#endif
