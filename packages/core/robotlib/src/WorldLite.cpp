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

#include "WorldLite.h"
#include <string.h>

WorldLiteObject WorldLite::sDummyObject;

WorldLite::WorldLite(){
    mNetwork = NULL;
}
WorldLite::~WorldLite(){
    for(int i=0;i<int(mObjects.size());i++)
        delete mObjects[i];
    mObjects.clear();
    mNetwork = NULL;
}

int                 WorldLite::AddObject(string name){
    WorldLiteObject* obj = new WorldLiteObject();
    obj->SetName(name);
    mObjects.push_back(obj);
    return int(mObjects.size())-1;
}
int                 WorldLite::AddObject(WorldLiteObject* object){
    if(object!=NULL){
        mObjects.push_back(object);
        return int(mObjects.size())-1;
    }
    return -1;
}

WorldLiteObject*    WorldLite::GetObject(int id){
    if((id>=0)&&(id<int(mObjects.size())))
        return mObjects[id];
    return NULL;
}
WorldLiteObject*    WorldLite::GetObject(string name){
    for(int i=0;i<int(mObjects.size());i++){
        if(mObjects[i]->GetName()==name)
            return mObjects[i];
    }
    return NULL;
}

int         WorldLite::StreamSize(){
    int res = GetStreamStringSize("World")+sizeof(int);
    for(unsigned int i=0;i<mObjects.size();i++)
        res += mObjects[i]->StreamSize();

    return res;
}
int         WorldLite::SetStream(void* memory){
    char* mem = (char*) memory;
    mem += SetStreamString("World",mem);
    *((int*)mem) = int(mObjects.size());
    mem += sizeof(int);
    //cout << "Sending "<<int(mObjects.size())<<" Objects"<<endl;
    for(unsigned int i=0;i<mObjects.size();i++)
        mem += mObjects[i]->SetStream(mem);
    return ((char*)mem)-((char*)memory);
}

int         WorldLite::StreamSizeFromStream(const void* memory){
    int res = CheckStreamTag("World",memory);
    if(res>0){
        char* mem = ((char*) memory) + res;
        int size = *((int*)mem);
        mem += sizeof(int);
        for(int i=0;i<size;i++)
            mem += sDummyObject.StreamSizeFromStream(mem);
        return ((char*)mem)-((char*)memory);
    }else{
        return 0;
    }
}

int         WorldLite::SetFromStream(const void* memory){
    int res = CheckStreamTag("World",memory);
    if(res>0){
        char* mem = ((char*) memory) + res;
        int size = *((int*)mem);
        mem += sizeof(int);
        for(int i=0;i<size;i++){
            int len = sDummyObject.SetFromStream(mem);
            WorldLiteObject* obj;
            if((obj=GetObject(sDummyObject.GetName()))!=NULL){
                mem += obj->SetFromStream(mem);

            }else{
                mem += len;
            }
        }
        return ((char*)mem)-((char*)memory);
    }else{
        return 0;
    }
}

void        WorldLite::SetNetwork(UDPNetwork* network){
    mNetwork = network;
}

void        WorldLite::SendWorld(){
    if(mNetwork){
        int size = SetStream(mNetwork->GetOutgoingMessageBuffer());
        mNetwork->SendMessage(size);
    }
}
bool        WorldLite::UpdateWorld(){
    if(mNetwork){
        if(mNetwork->GetMessage()>0){
            char* dataPtr = mNetwork->GetIncomingMessageBuffer();
            dataPtr  += SetFromStream(dataPtr);
            return true;
        }
    }
    return false;
}


WorldLiteObject::WorldLiteObject(){
    mName = "";
    memset(mRef,0,16*sizeof(double));
    mRef[0][0]  = 1.0;
    mRef[1][1]  = 1.0;
    mRef[2][2]  = 1.0;
    mRef[3][3]  = 1.0;
    mTimeStamp  = 0.0;
}
WorldLiteObject::~WorldLiteObject(){
}

void        WorldLiteObject::SetName(string name){
    mName = name;
}
void        WorldLiteObject::SetRef(double* ref){
    memcpy(mRef,ref,16*sizeof(double));
}
void        WorldLiteObject::SetTime(double time){
    mTimeStamp =  time;
}
string  WorldLiteObject::GetName(){
    return mName;
}
double* WorldLiteObject::GetRef(){
    return (double*)mRef;
}
double WorldLiteObject::GetTime(){
    return mTimeStamp;
}

int         WorldLiteObject::StreamSize(){
    int len = GetStreamStringSize("WorldObject");
    len += GetStreamStringSize(mName.c_str());
    len += sizeof(double)+16*sizeof(double);
    return len;
}
int         WorldLiteObject::StreamSizeFromStream(const void* memory){
    int res = CheckStreamTag("WorldObject",memory);
    if(res>0){
        char* mem = ((char*) memory) + res;
        mem += GetStreamStringSize(mem);
        return ((char*)mem)-((char*)memory) + sizeof(double)+16*sizeof(double);
    }
    return 0;
}

int         WorldLiteObject::SetStream(void* memory){
    char *mem = (char*)memory;
    mem += SetStreamString("WorldObject",mem);
    mem += SetStreamString(mName.c_str(),mem);
    *((double*)mem) = mTimeStamp;
    mem += sizeof(double);
    memcpy(mem,mRef,16*sizeof(double));
    //mWorldReferenceFrame.GetHMatrix().Print();
    return ((char*)mem)-((char*)memory) + 16*sizeof(double);
}
int         WorldLiteObject::SetFromStream(const void* memory){
    int res = CheckStreamTag("WorldObject",memory);
    if(res>0){
        char* mem = ((char*) memory) + res;
        mName = mem;
        mem += GetStreamStringSize(mName.c_str());
        mTimeStamp = *((double*)mem);
        mem += sizeof(double);
        memcpy(mRef,mem,16*sizeof(double));
        return ((char*)mem)-((char*)memory) + 16*sizeof(double);
    }
    return 0;
}
