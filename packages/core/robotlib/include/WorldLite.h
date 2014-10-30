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

#ifndef WORLDLITE_H_
#define WORLDLITE_H_

#if defined(WIN32) || defined (USE_FLAT_SOURCE_CODE)
#include "Streamable.h"
#include "UDPNetwork.h"
#else
#include "StdTools/Streamable.h"
#include "UDPNetwork/UDPNetwork.h"
#endif

#include <string>
#include <vector>

using namespace std;

class WorldLiteObject : public Streamable
{
protected:
    string              mName;
    double              mRef[4][4];
    double              mTimeStamp;

public:
    WorldLiteObject();
    ~WorldLiteObject();

    void        SetName(string name);
    void        SetRef(double *ref);
    void        SetTime(double time);

    string      GetName();
    double*     GetRef();
    double      GetTime();

public:
    virtual int         StreamSize();
    virtual int         StreamSizeFromStream(const void* memory);
    virtual int         SetStream(void* memory);
    virtual int         SetFromStream(const void* memory);
};



class WorldLite : public Streamable
{
protected:
    vector<WorldLiteObject*>    mObjects;

    UDPNetwork                 *mNetwork;

public:
                        WorldLite();
                        ~WorldLite();

    int                 AddObject(string name);
    int                 AddObject(WorldLiteObject* object);
    WorldLiteObject*    GetObject(int id);
    WorldLiteObject*    GetObject(string name);

            void        SetNetwork(UDPNetwork* network);

            void        SendWorld();
            bool        UpdateWorld();

protected:
    virtual int         StreamSize();
    virtual int         StreamSizeFromStream(const void* memory);
    virtual int         SetStream(void* memory);
    virtual int         SetFromStream(const void* memory);
protected:
    static WorldLiteObject sDummyObject;
};

#endif
