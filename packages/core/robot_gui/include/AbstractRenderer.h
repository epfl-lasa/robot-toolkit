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

#ifndef ABSTRACTRENDERER_H_
#define ABSTRACTRENDERER_H_

#include "StdTools/XmlTree.h"
#include "GLTools/GLTools.h"

#include <vector>
using namespace std;

class AbstractRenderer
{
public:
    vector<AbstractRenderer*>   mSubRenderers;
public:
          AbstractRenderer();
  virtual ~AbstractRenderer();

public:
    virtual void    Free();
    virtual void    Render();
    virtual void    RenderShadow(int pass, const Vector3& light, const Vector3& observer);
    virtual void    RenderOutline(const Vector3& observer);
    virtual void    RenderBoundingBox();

    virtual bool    Load(const pXmlTree tree);
    virtual bool    Configure(const pXmlTree tree);

            void    AddRenderer(AbstractRenderer* renderer);

  // Courtesy of M Duvanel
    virtual bool    ComputeRayIntersection(Vector3 vecRay1, Vector3 vecRay2, double & dFactor, bool bIntersections, unsigned* pShapeIndex) {return false;};
    virtual void    AddShapeFromRay(unsigned uShapeIndex, Vector3 vecRay1, Vector3 vecRay2, double dFactor, double dSphereScale) {}
    virtual void    RemoveShape(unsigned uShapeIndex) {}

};

typedef AbstractRenderer *pAbstractRenderer;

#endif /*ROBOTRENDERER_H_*/
