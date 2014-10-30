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

#ifndef ROBOTTOOLS_H_
#define ROBOTTOOLS_H_

#include "StdTools/Timer.h"
#include "MathLib/MathLib.h"
using namespace MathLib;

class Clock
{
protected:
    Chrono      mChrono;
    bool        bUseInternalTime;
    bool        bUseDt;
    bool        bSetStartTime;
    REALTYPE    mTime;
    REALTYPE    mStartTime;
    REALTYPE    mDt;

public:
            Clock();
    virtual ~Clock();

public:
            void      Reset();
            void      SetTime(REALTYPE time);
            void      SetDt(REALTYPE dt);
            void      SetTimeDt(REALTYPE time, REALTYPE dt);
            void      SetInternal(bool bInternal = true);
            REALTYPE  GetDt() const;
            REALTYPE  GetTime() const;

            void      Update();

};

#endif

