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

#ifndef GL3DOBJECT_H_
#define GL3DOBJECT_H_

#include "GLTools.h"
#include "Base3DObject.h"
#include "MathLib/MathLib.h"


class GL3DObject : public Base3DObject
{
  
public:
    
  int           mCallListId;
  int           mWireframeCallListId;

  GL3DObject   *mShadowObject;
  
public:  
            GL3DObject();
    virtual ~GL3DObject();

    virtual void    Free();

    virtual void    Render();
            void    RenderWireframe();
            void    RenderShadow(const Vector3& light);

            int     BuildDisplayList();
            int     BuildWireframeDisplayList();

    virtual GL3DObject*    Clone(GL3DObject* res=NULL);
            void    SetShadow(GL3DObject* shadow);

            void    RenderOutline(const Vector3& observer);

    static  void    RenderShadowInit();
    static  void    RenderShadowInitPass(int pass);
            void    RenderShadowPass(int pass, const Vector3& light);
    static  void    RenderShadowEnd();

private:
    bool            bShadowReady;
    Matrix          mShadowVertices;
};


#endif /*GL3DOBJECT_H_*/
