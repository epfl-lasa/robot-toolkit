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

#ifndef OBJECTRENDERER_H_
#define OBJECTRENDERER_H_

#include "MathLib/MathLib.h"
using namespace MathLib;

#include "AbstractRenderer.h"
#include "GLTools/GL3DObject.h"
#include "RobotLib/World.h"
 

class ObjectRenderer : public AbstractRenderer
{

protected:
    typedef struct{
        GL3DObject      *shape;
        ReferenceFrame   refShape;  
        REALTYPE         color[4];
        REALTYPE         scale[3];
        bool			 culling;
        string      strShapeName;
    } ShapeStruct;
    typedef ShapeStruct *pShapeStruct;

    string              mBasePath;

    WorldObject         *mObject;
     
    vector<pShapeStruct> mShapes; 
    vector<pShapeStruct> mBBoxShapes;


    bool                bDrawObject;
    bool                bUseDefaultColor;
    bool                bUseTransparency;
    float               mDefaultColor[4];


    float               mComColor[4];


    float               mRefSize;
  
public:  
    bool                bDrawRef;
    bool                bDrawCom;
                    ObjectRenderer();
    virtual         ~ObjectRenderer();
    virtual void    Free();
  
  
    virtual void            Render();
    virtual void            RenderShadow(int pass, const Vector3& light, const Vector3& observer);
    virtual void            RenderOutline(const Vector3& observer);
    virtual void            RenderBoundingBox();

    virtual bool            Load(string name);
    virtual bool            Load(const pXmlTree tree);
    virtual bool            Configure(const pXmlTree tree);


            pShapeStruct    LoadShape(const pXmlTree tree);

    virtual bool            LinkToObject(WorldObject *object);
  
            void            SetBasePath(string path);
	// Courtesy of M Duvanel
    virtual bool            ComputeRayIntersection(Vector3 vecRay1, Vector3 vecRay2, double & dFactor, bool bIntersections, unsigned* pShapeIndex);
    virtual void            AddShapeFromRay(unsigned uShapeIndex, Vector3 vecRay1, Vector3 vecRay2, double dFactor, double dSphereScale);
    virtual void            RemoveShape(unsigned uShapeIndex);
protected:
    static const string  mStrIntersectsShapeName;

};


#endif

