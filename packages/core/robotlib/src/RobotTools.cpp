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

#include "RobotTools.h"

Clock::Clock(){
    bUseInternalTime    = true;
    bUseDt              = true;
    bSetStartTime       = false;
    mTime               = R_ZERO;
    mDt                 = R_ZERO;
    mStartTime          = R_ZERO;
    Reset();
}

Clock::~Clock(){}

void    Clock::Reset(){
    if(bUseInternalTime)
        mChrono.Start();
    mTime       = R_ZERO;
    mDt         = R_ZERO;
}

void    Clock::SetTime(REALTYPE time){
    bSetStartTime       = true;
    bUseDt              = true;
    mStartTime          = time;
}
void    Clock::SetDt(REALTYPE dt){
    mDt                 = dt;
    bUseInternalTime    = false;
    bUseDt              = true;
}
void    Clock::SetTimeDt(REALTYPE time, REALTYPE dt){
    mDt                 = dt;
    mTime               = time;
    mStartTime          = time;
    bUseInternalTime    = false;
    bUseDt              = false;
}
void    Clock::SetInternal(bool bInternal){
    bUseInternalTime    = bInternal;
}

void    Clock::Update(){
    if(bSetStartTime){
        if(bUseInternalTime){
            mChrono.Start();
            mDt     = mStartTime - mTime;
        }
        mDt     = mStartTime - mTime;
        mTime   = mStartTime;
        bSetStartTime = false;
    }else{
        if(bUseInternalTime){
            REALTYPE newTime;
            newTime = mStartTime + REALTYPE(mChrono.ElapsedTimeUs())*(1e-6);
            mDt     = newTime - mTime;
            mTime   = newTime;
        }else{
            if(bUseDt)
                mTime += mDt;
        }
    }
}

REALTYPE  Clock::GetDt() const{
    return mDt;
}
REALTYPE  Clock::GetTime() const{
    return mTime;
}
