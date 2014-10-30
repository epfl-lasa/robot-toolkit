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

#ifndef WORLD_H_
#define WORLD_H_

#include "MathLib/MathLib.h"
using namespace MathLib;
#include "StdTools/XmlTree.h"
#include "StdTools/LogStream.h"
#include "StdTools/Streamable.h"

#include <string>
#include <vector>
using namespace std;

#include "WorldInterface.h"
#include "WorldObject.h"


class World : public Streamable
{
protected:
    vector<WorldInterface*> mInterfaces;
    vector<Robot*>          mRobots;

public:
    vector<pWorldObject>    mObjects;
    vector<bool>            mObjectsOwnership;

    vector<WorldObjectLink*> mObjectLinks;

public:
    /// Constructor
    World();
    /// Destructor
    ~World();

    /// Load the world from a xml tree
    bool                Load(pXmlTree tree);
    /// Load the world from filename
    bool                Load(string name);

    /// Add an object to the world
    void                AddObject(pWorldObject object, bool bAttach = true);

    /// Get the number of objects in the world
    int                 GetObjectCount();
    /// Get the object by id
    pWorldObject        GetObject(int id);
    /// Get the object by it's name
    pWorldObject        Find(string name);


    /// Interfaces management: Init phase
            void            Init();
    /// Interfaces management: Free phase
            void            Free();
    /// Interfaces management: Start phase
            void            Start();
    /// Interfaces management: Stop phase
            void            Stop();
    /// Interfaces management: Update phase
            void            Update();
    /// Interfaces management: Update core phase
            void            UpdateCore();
    /// Interfaces management: Additional drawing phase
            void            Draw();

            void            AddInterface(WorldInterface *worldInterface);

            const vector<WorldInterface*>&  GetWorldInterfaces();
            const vector<Robot*>&           GetRobots();

            bool            IsEmpty();

    virtual int         StreamSize();
    virtual int         StreamSizeFromStream(const void* memory);
    virtual int         SetStream(void* memory);
    virtual int         SetFromStream(const void* memory);

};


#endif /*WORLD_H_*/
