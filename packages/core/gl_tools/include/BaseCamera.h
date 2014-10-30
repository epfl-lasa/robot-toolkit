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

#ifndef __BASECAMERA_H__
#define __BASECAMERA_H__

#include "MathLib/MathLib.h"
using namespace MathLib;

class BaseCamera
{
public:
    Matrix4     mRef;
  
    int         mWidth;
    int         mHeight;
    
    REALTYPE    mPrincipalPointX;
    REALTYPE    mPrincipalPointY;
    REALTYPE    mFocalX;
    REALTYPE    mFocalY;

    REALTYPE    mNearPlane;
    REALTYPE    mFarPlane;

public:
            BaseCamera();
    virtual ~BaseCamera();
    
            void Clear();

            void SetOrientation (const Matrix3 & orient);
            void SetPosition    (const Vector3 & pos);
            void SetPose        (const Matrix4 & pose);

            void SetViewport    (int w, int h);
            void SetProjection  (int im_width,
                                 int im_height,
                                 REALTYPE principal_x,
                                 REALTYPE principal_y,
                                 REALTYPE focal_x,
                                 REALTYPE focal_y,
                                 REALTYPE mnear, 
                                 REALTYPE mfar);

};

#endif
