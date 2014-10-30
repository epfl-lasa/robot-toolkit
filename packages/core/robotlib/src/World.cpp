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

#include "World.h"
#include "Robot.h"





World::World(){
}
World::~World(){
    Free();
}



void World::Free(){
    for(int i=0;i<int(mInterfaces.size());i++)
        mInterfaces[i]->Free();
    mInterfaces.clear();

    for(int i=0;i<int(mRobots.size());i++)
        mRobots[i]->Free();
    mRobots.clear();

    for(unsigned int i=0;i<mObjects.size();i++){
        if(mObjectsOwnership[i])
            delete mObjects[i];
    }
    mObjects.clear();
    mObjectsOwnership.clear();
}

void World::Init(){
    for(int i=0;i<int(mInterfaces.size());i++){
        mInterfaces[i]->SetWorld(this);
        mInterfaces[i]->Init();
    }
    for(int i=0;i<int(mRobots.size());i++){
        mRobots[i]->SetWorld(this);
        mRobots[i]->Init();
    }
}
void World::Start(){
    for(int i=0;i<int(mInterfaces.size());i++)
        mInterfaces[i]->Start();
    for(int i=0;i<int(mRobots.size());i++)
        mRobots[i]->Start();
}
void World::Stop(){
    for(int i=0;i<int(mInterfaces.size());i++)
        mInterfaces[i]->Stop();
    for(int i=0;i<int(mRobots.size());i++)
        mRobots[i]->Stop();
}
void  World::Update(){
    for(int i=0;i<int(mInterfaces.size());i++)
        mInterfaces[i]->Update();
    for(int i=0;i<int(mRobots.size());i++)
        mRobots[i]->Update();
}

void  World::UpdateCore(){
    for(int i=0;i<int(mInterfaces.size());i++)
        mInterfaces[i]->UpdateCore();
    for(int i=0;i<int(mRobots.size());i++)
        mRobots[i]->UpdateCore();
}
void  World::Draw(){
    for(int i=0;i<int(mInterfaces.size());i++)
        mInterfaces[i]->Draw();
    for(int i=0;i<int(mRobots.size());i++)
        mRobots[i]->Draw();
}

void  World::AddInterface(WorldInterface *worldInterface){
    if(worldInterface!=NULL)
        mInterfaces.push_back(worldInterface);
}


bool World::Load(string name){
    char filename[256];
    XmlTree tree;
    sprintf(filename,"Worlds/%s.xml",name.c_str());
    if(FileFinder::Find(filename)){
        if(tree.LoadFromFile(FileFinder::GetString())){
            return Load(&tree);
        }
    }
    return false;
}
bool World::IsEmpty(){
    return !((mInterfaces.size()>0)||(mRobots.size()>0)||(mObjects.size()>0));
}

bool World::Load(pXmlTree tree){
    if(tree==NULL) return false;

    Free();
    pXmlTreeList  tmpTreeList;
    string        str;

    gLOG.SetCurrentEntry("World");
    gLOG.Append("Loading World: %s",tree->GetData().c_str());
    gLOG.SetDeltaIndent(2);
    tmpTreeList = tree->GetSubTrees();

    for(unsigned int i=0;i<tmpTreeList->size();i++){
        if(tmpTreeList->at(i)->GetName()=="Object"){
            pWorldObject obj = new WorldObject();
            if(obj->Load(tmpTreeList->at(i))){
                AddObject(obj,true);
            }
        }else if(tmpTreeList->at(i)->GetName()=="FixJoint"){
            WorldObjectLink* link = new WorldObjectLink(tmpTreeList->at(i)->Get("ObjectA",string("")),
                                                        tmpTreeList->at(i)->Get("ObjectB",string("")));
            mObjectLinks.push_back(link);
        }

    }
    gLOG.SetDeltaIndent(-2);
    return true;
}

pWorldObject World::Find(string name){

    vector<string> nameList = Tokenize(name,".",".");

    if(nameList.size()==1){
        for(unsigned int i=0;i<mObjects.size();i++){
            if(name == mObjects[i]->GetName())
                return mObjects[i];
        }
    }else if(nameList.size()==2){
        for(unsigned int i=0;i<mObjects.size();i++){
            if(nameList[0] == mObjects[i]->GetName()){
                if(mObjects[i]->IsRobot()){
                    int id = mObjects[i]->GetRobot()->GetLinkIndex(nameList[1]);
                    if(id<0) return NULL;
                    else return mObjects[i]->GetRobot()->GetLinks()[id];
                }else{
                    return NULL;
                }
            }
        }
    }
    return NULL;

}

void    World::AddObject(pWorldObject object, bool bAttach){
    if(object!=NULL){
        if(object->IsRobot()){
            mRobots.push_back(object->GetRobot());
        }
        mObjects.push_back(object);
        mObjectsOwnership.push_back(bAttach);
    }
}

int World::GetObjectCount(){
    return int(mObjects.size());
}
pWorldObject World::GetObject(int id){
    if((id>=0)&&(id<int(mObjects.size())))
        return mObjects[id];
    return NULL;
}


const vector<WorldInterface*>&  World::GetWorldInterfaces(){
    return mInterfaces;
}
const vector<Robot*>&           World::GetRobots(){
    return mRobots;
}




int         World::StreamSize(){
    int res = GetStreamStringSize("World");
    res += sizeof(int);
    for(unsigned int i=0;i<mObjects.size();i++)
        res += mObjects[i]->StreamSize();

    return res;
}
int         World::SetStream(void* memory){
    char* mem = (char*) memory;
    mem += SetStreamString("World",mem);
    *((int*)mem) = int(mObjects.size());
    mem += sizeof(int);
    // cout << "Sending "<<int(mObjects.size())<<" Objects"<<endl;
    for(unsigned int i=0;i<mObjects.size();i++)
        mem += mObjects[i]->SetStream(mem);
    return ((char*)mem)-((char*)memory);
}

int         World::StreamSizeFromStream(const void* memory){
    int res = CheckStreamTag("World",memory);
    if(res>0){
        char* mem = ((char*) memory) + res;
        //mem += sizeof(double);
        int size = *((int*)mem);
        mem += sizeof(int);
        for(int i=0;i<size;i++)
            mem += WorldObject::sDummyObject.StreamSizeFromStream(mem);
        return ((char*)mem)-((char*)memory);
    }else{
        return 0;
    }
}

int         World::SetFromStream(const void* memory){
    int res = CheckStreamTag("World",memory);
    if(res>0){
        char* mem = ((char*) memory) + res;
        //mem += sizeof(double);
        int size = *((int*)mem);
        mem += sizeof(int);
        //cout << "FOund "<<size<<"Objects"<<endl;
        //size = MIN(size,int(mObjects.size()));
        for(int i=0;i<size;i++){
            WorldObject::sDummyObject.SetFromStream(mem);
            pWorldObject obj;
            if((obj=Find(WorldObject::sDummyObject.GetName()))!=NULL){
                mem += obj->SetFromStream(mem);
            }else{
                mem += WorldObject::sDummyObject.StreamSizeFromStream(mem);
            }
        }
        return ((char*)mem)-((char*)memory);
    }else{
        return 0;
    }
}


