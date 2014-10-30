/*
 * Copyright (C) 2010 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
 * Author: Klas Kronander
 * email:   klas.kronander@epfl.ch
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
//testing svn

#ifndef GolfModule_H_
#define GolfModule_H_

#include "MathLib/MathLib.h"
#include "MathLib/IKGroupSolver.h"
using namespace MathLib;

#include "RobotLib/Robot.h"
#include "RobotLib/RobotTools.h"
#include "RobotLib/RobotInterface.h"
#include "RobotLib/ForwardDynamics.h"
#include "RobotLib/InverseDynamics.h"
#include "RobotLib/KinematicChain.h"
#include "RobotLib/PIDController.h"
#include "GMR/GMR.h"
#include "RobotLib/WorldLite.h"
#include "UDPNetwork/UDPNetwork.h"
#include "UDPNetwork/UDPConnection.h"
#include "StdTools/Timer.h"
#include "StdTools/Streamable.h"
#include "StdTools/WrapMallocs.h"
//#include "SimulatorDynamics/SimulatorDynamicsInterface.h"

#include <string>



//ROS-stuff
//#include <ros/ros.h>
//#include <wam_msgs/JointAngles.h>


#define MAX_LOG_TIME_SEC        120
#define CYCLES_PER_SEC          500
#define MAX_LOG_CYCLES          (MAX_LOG_TIME_SEC * CYCLES_PER_SEC)
//#define PI						3.141592653589793


class GolfModule : public RobotInterface
{
 public:

    Robot                       mInternalRobot;
    RevoluteJointSensorGroup    mInternalSensorsGroup;
    RevoluteJointActuatorGroup  mInternalActuatorsGroup;
    Clock                       mClock;
    RevoluteJointSensorGroup    mSensorsGroup;
    RevoluteJointActuatorGroup  mActuatorsGroup;
    KinematicChain              mKinematicChain;
    IKGroupSolver               mIKSolver;
    InverseDynamics             mInvDynamics;
    GaussianMixture		DS_model;
    GaussianMixture             DS_model_v;

    PIDController               mPIDCtrl;
	
    Vector                      mJointTorques;
    Vector                      mJointTarget;
    Vector                      mJointTarget_vel;
    Vector                      mJointTarget_acc;
    Vector                      mJointSetPos;
    Vector                      TorqueLimits;
    Vector                      JointHomePos;

    Vector			xd_switching; 
    Vector			Dynamic_dir;
    Vector			hitting_dir;
    Vector			Racket_dir;
    Vector                      Racket_dir_proj;
    //Vector                      swing_dir;
    
    Vector TmpRowFiltering;

    
    int 			mState;

    bool			hit;

    bool                        autoadjust;
	
	
    double			beta; 
    double			hitting_gain;
    double                      angular_offset;

    int                         eventID;


    double                      command_time; //time of last command (hit or init)
    double                      hit_time; //time of impact with ball

    Vector                      lim1; 
    Vector                      lim2; 
    Vector                      weights; 
    Vector                      rest; 
    Vector                      targetCart,targetCart_last;
    Vector                      xd;

    Vector                      target_vel; 
    Vector                      target_offset;

    Matrix                      R; 
    Vector                      Ball_pos,Target_pos,Initial_point; 
    Vector                      hitting_point;
    bool                        hpball;

    Vector                      w_rotation;
    Matrix                      T_L_G;

    Vector                      currPos;





    GaussianMixture             adjustModel;
    Matrix                      adjustData;
    int                         demoCount;


    bool                        simulatorMode;

    Matrix                      R_DS;
    Vector                      Default_dir;
   



    Vector                      Target_pos_original;
    

    double                      exploreclock;
    bool                        exploreMode;
    bool                        rw;
    int                         exploreTargetIter;
    int                         exploreParamIter;
    Matrix                      exploreTargetPos;
    Matrix                      exploreParams;
    string                      exploreFileName;
    double                      min_dist;
    double                      final_dist;

 public:
    GolfModule();
    virtual ~GolfModule();
  
    virtual Status              RobotInit();
    virtual Status              RobotFree();
  
    virtual Status              RobotStart();    
    virtual Status              RobotStop();
  
    virtual Status              RobotUpdate();
    virtual Status              RobotUpdateCore();
    virtual int                 RespondToConsoleCommand(const string cmd, const vector<string> &args);
	
    Vector                      load_FilterCoefficients(const char filename[]);

    void                        Filter(Vector& input, Matrix& input_old, int& inputp, Vector& filter);


    Matrix                      Compute_Rotation_Matrix(Vector Racket_dir, Vector next_dir);
    void                        InitializeCommands();

    void                        changeHittingDirection();

    void                        learn(int num_k);
    void                        addCurrent();
    void                        adjust();
    void                        loadTrainingData(const char filename[],int democnt);
    void                        saveTrainingData(const char filename[]);
    void                        clearTrainingData();
    void                        saveModel(const char filename[]);

    void                        updateTargetPosition();

    //    void                        resetWorld();
    void                        exploreModeDo();






    
};

#endif 
