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

#ifndef __LINK_H__
#define __LINK_H__

#include <string>
#include <vector>
using namespace std;

#include "MathLib/MathLib.h"
using namespace MathLib;
#include "StdTools/XmlTree.h"
#include "StdTools/TTree.h"

#include "Joint.h"
#include "World.h"

class Link;
typedef Link *pLink;
typedef vector<pLink> LinksList;

class Link : public WorldObject
{
public:
  int               mId;
//  string            mName;

  Joint            *mJoint;
  vector<pJoint>    mChildrenJoints;

//  SpatialInertia    mInertia;
//  Vector3           mCenterOfMass;

  ReferenceFrame    mRefFrame;

  SpatialForce      mForce;
  SpatialVelocity   mVelocity;
  SpatialVector     mAccel;

  SpatialFrame      mSpFrame;

  ReferenceFrame    mPos;



public:
          Link();
  virtual ~Link();

  virtual bool      Load(const pXmlTree tree);
          void      SetIndex(int index);
          void      AddChildJoint(pJoint child);


          Matrix4&  GetRef();

          pJoint    GetJoint();

          void      Update();
          bool      operator == (const Link & link);

          REALTYPE      GetMass();
          Vector3&      GetCenterOfMass(Vector3 &result, bool bRecursive = true, REALTYPE *cumMass = NULL);
          //Vector3&      GetFirstMoment(Vector3 &result, bool bRecursive = true, REALTYPE *mass = NULL);


          void      Print();

protected:
  virtual void  Free();
};





class LinkTree : public TTree<Link>
{
public:
          LinkTree();
          LinkTree(const LinkTree & tree);
  virtual ~LinkTree();

          LinkTree*     Load(const pXmlTree tree, int *currIndexPtr = NULL);
          void          Update();
          void          SetInitialState();

          void          Print();

};

typedef LinkTree *pLinkTree;



#endif
