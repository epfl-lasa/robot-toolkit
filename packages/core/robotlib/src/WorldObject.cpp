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

#include "WorldObject.h"
#include "World.h"
#include "Robot.h"


WorldObject WorldObject::sDummyObject;

WorldObject::WorldObject(){
    mName               = "";
    mConfigTree         = NULL;
    mBBoxShapeTree      = NULL;
    mGfxShapeTree       = NULL;
    mRobot              = NULL;
    bSlaveMode          = false;
    mWorldSpatialFrame.Set(mWorldReferenceFrame);
}

WorldObject::~WorldObject(){
    Free();
}

void  WorldObject::Free(){
    mName            = "";
    if(mConfigTree)     delete mConfigTree;     mConfigTree     = NULL;
    mBBoxShapeTree  = NULL;
    mGfxShapeTree   = NULL;
    bSlaveMode          = false;
}

bool WorldObject::Load(pXmlTree tree){
    if(tree==NULL) return false;
    REALTYPE *array;

    Free();


    mName        = tree->GetData();

    gLOG.SetCurrentEntry("World");
    gLOG.Append("Loading Object: %s",mName.c_str());
    gLOG.SetDeltaIndent(2);

    int size;

    bool bHasJoint = (tree->Find("Joint")!=NULL);

    if(tree->Find("Origin")){
        size=tree->GetArray("Origin",&array);
        if(size==3){            mWorldReferenceFrame.SetOrigin().Set(array);
        }else{                  mWorldReferenceFrame.SetOrigin().Zero();
                                gLOG.Append("Error: Bad <Origin> array size (should be 3)");
        }
    }else{
                                mWorldReferenceFrame.SetOrigin().Zero();
                                if(!bHasJoint) gLOG.Append("Warning: No <Origin> defined");
    }

    if(tree->Find("Orient")){
        size=tree->GetArray("Orient",&array);
        if(size==9){            mWorldReferenceFrame.SetOrient().Set(array);
                                mWorldReferenceFrame.SetOrient().Normalize();
        }else if(size==3){      mWorldReferenceFrame.SetOrient().SRotationV(Vector3(array));
        }else{                  mWorldReferenceFrame.SetOrient().Identity();
                                gLOG.Append("Error: Bad <Orient> array size (should be 3(axis*angle) or 9(full rotation matrix))");
        }
    }else{
                                mWorldReferenceFrame.SetOrient().Identity();
                                if(!bHasJoint) gLOG.Append("Warning: No <Orient> defined");
    }

    if(tree->Find("Velocity")){
        size=tree->GetArray("Velocity",&array);
                                gLOG.Append("Setting <Velocity> array size %d",size);
        if(size==3){            mWorldVelocity.SetLinearComponent().Set(array);
        }else{                  mWorldVelocity.SetLinearComponent().Zero();
                                gLOG.Append("Error: Bad <Velocity> array size (should be 3)");
        }
    }else{
                                mWorldVelocity.SetLinearComponent().Zero();
    }
    if(tree->Find("AngularVelocity")){
        size=tree->GetArray("AngularVelocity",&array);
        if(size==3){            mWorldVelocity.SetAngularComponent().Set(array);
        }else{                  mWorldVelocity.SetAngularComponent().Zero();
                                gLOG.Append("Error: Bad <AngularVelocity> array size (should be 3)");
        }
    }else{
                                mWorldVelocity.SetAngularComponent().Zero();
    }

    SetInitialState();

    if(tree->Find("Mass")){
                                mInertia.mMass = tree->Get("Mass",0.0);
    }else{
                                mInertia.mMass = 0.0;
                                gLOG.Append("Warning: No <Mass> defined");
    }

    if(tree->Find("CenterOfMass")){
        size=tree->GetArray("CenterOfMass",&array);
        if(size==3){            mInertia.mCenterOfMass.Set(array);
        }else{                  mInertia.mCenterOfMass.Zero();
                                gLOG.Append("Error: Bad <CenterOfMass> array size (should be 3)");
        }
    }else{
                                mInertia.mCenterOfMass.Zero();
                                gLOG.Append("Warning: No <CenterOfMass> defined");
    }

    if(tree->Find("InertiaMatrix")){
        size=tree->GetArray("InertiaMatrix",&array);
        if(size==9){            mInertia.mInertiaMoment.Set(array);
        }else if(size==3){      mInertia.mInertiaMoment.Diag(Vector3(array));
        }else{                  mInertia.mInertiaMoment.Zero();
                                gLOG.Append("Error: Bad <InertiaMatrix> array size (should be 3 or 9)");
        }
    }else{
                                mInertia.mInertiaMoment.Zero();
                                gLOG.Append("Warning: No <InertiaMatrix> defined");
    }

    mInertia.mCenterOfMass.Mult(mInertia.mMass,mInertia.mLinearMoment);

    if(tree->Find("Material")){
        if(tree->Find("Material.Bouncyness")){
            mMaterial.mBouncyness = tree->Get("Material.Bouncyness",0.0);
        }else{
            mMaterial.mBouncyness = 0.0;
            gLOG.Append("Warning: No <Bouncyness> defined, setting 0 by default");
        }
        if(tree->Find("Material.StaticFriction")){
            mMaterial.mStaticFrictionCoef   = tree->Get("Material.StaticFriction",0.0);
            if(mMaterial.mStaticFrictionCoef<0.0) mMaterial.mStaticFrictionCoef = R_INFINITY;
        }else{
            mMaterial.mStaticFrictionCoef   = 0;
            gLOG.Append("Warning: No <StaticFriction> defined, setting 0 by default");
        }
    }else{
        mMaterial.mBouncyness           = 0.0;
        mMaterial.mStaticFrictionCoef   = 0;
        gLOG.Append("Warning: No <Material> defined, setting default values (bouncyness 0, friction 0)");
    }


    pXmlTree bbox = tree->Find("BBoxShape");
    if(bbox!=NULL){
        /*
        pXmlTreeList tmpList = bbox->GetSubTrees();
        for(int i=0;i<int(tmpList->size());i++){
            if(tmpList->at(i)->GetName() == "Shape"){
                if(mShapeTree==NULL){
                    mShapeTree = new XmlTree("ShapeList");
                    mShapeTree->SetData(mName);
                }
                mShapeTree->AddSubTree(tmpList->at(i)->Clone());
            }
        }
        */
    }else{
        gLOG.Append("Warning: No <BBoxShape> defined");
    }

    //if(mGfxShapeTree)   delete mGfxShapeTree;   mGfxShapeTree   = NULL;
    pXmlTree gfx = tree->Find("GfxShape");
    if(gfx!=NULL){
        if(gfx->GetData() == "Clone"){
            if(bbox==NULL){
                gLOG.Append("Error: Cannot clone non-existent <Shape> to <GfxShape>");
            }else{
                pXmlTree gfxShape = bbox->Clone();
                gfxShape->SetName("GfxShape");
                gfxShape->SetData(mName);
                tree->AddSubTree(gfxShape);
            }
        }else{
            /*
            tmpList = gfx->GetSubTrees();
            for(int i=0;i<int(tmpList->size());i++){
                if(tmpList->at(i)->GetName() == "Shape"){
                    if(mGfxShapeTree==NULL){
                        mGfxShapeTree = new XmlTree("GfxShapeList");
                        mGfxShapeTree->SetData(mName);
                    }
                    mGfxShapeTree->AddSubTree(tmpList->at(i)->Clone());
                }
            }
            if(mGfxShapeTree==NULL){
                gLOG.Append("Warning: No <Shape> defined within <GfxShape>");
            }
            */
        }
    }else{
        gLOG.Append("Warning: No <GfxShape> defined");
        gLOG.SetDeltaIndent(2);
        if(bbox==NULL){
            gLOG.Append("Warning: Cannot clone non-existent <BBoxShape> to <GfxShape>");
        }else{
            pXmlTree gfxShape = bbox->Clone();
            gfxShape->SetName("GfxShape");
            gfxShape->SetData(mName);
            tree->AddSubTree(gfxShape);
            gLOG.Append("Cloning the list of <BBoxShape>");
        }
        gLOG.SetDeltaIndent(-2);
    }

    if(mConfigTree) delete mConfigTree;
    mConfigTree     = tree->Clone();
    mGfxShapeTree   = mConfigTree->Find("GfxShape");



    if(mGfxShapeTree != NULL){
    	pXmlTreeList treelist = mGfxShapeTree->GetSubTrees();

    }
    mBBoxShapeTree  = mConfigTree->Find("BBoxShape");

    gLOG.SetDeltaIndent(-2);

    return true;
}

ReferenceFrame&  WorldObject::GetReferenceFrame(bool initial){
    if(initial)     return mInitialWorldReferenceFrame;
    else            return mWorldReferenceFrame;
}
string&      WorldObject::GetName(){
    return mName;
}
void          WorldObject::SetName(string name){
    mName = name;
}

pXmlTree      WorldObject::GetBBoxShapeTree(){
    return mBBoxShapeTree;
}
pXmlTree      WorldObject::GetGfxShapeTree(){
    return mGfxShapeTree;
}
pXmlTree      WorldObject::GetConfigTree(){
    return mConfigTree;
}
void        WorldObject::SetInitialState(){
    mInitialWorldReferenceFrame = mWorldReferenceFrame;
    mInitialWorldVelocity       = mWorldVelocity;
    if(mRobot){
        mRobot->GetLinks()[0]->GetReferenceFrame(true)  = mInitialWorldReferenceFrame;
        mRobot->GetLinks()[0]->GetSpatialVelocity(true) = mInitialWorldVelocity;
        mRobot->UpdateLinks(true);
        //mInitialWorldReferenceFrame.GetOrigin().Print();
        //mRobot->GetLinks()[0]->GetReferenceFrame(true).GetOrigin().Print();
    }
}
void        WorldObject::SetToInitialState(){
    mWorldReferenceFrame    = mInitialWorldReferenceFrame;
    mWorldVelocity          = mInitialWorldVelocity;
    if(mRobot){
        mRobot->GetLinks()[0]->GetReferenceFrame(false)  = mInitialWorldReferenceFrame;
        mRobot->GetLinks()[0]->GetSpatialVelocity(false) = mInitialWorldVelocity;
        mRobot->UpdateLinks(false);
    }
}
SpatialFrame&    WorldObject::GetSpatialFrame(){
    return mWorldSpatialFrame;
}

SpatialInertia&  WorldObject::GetSpatialInertia(){
    return mInertia;
}
WorldObject::MaterialProperty&   WorldObject::GetMaterialProperty(){
    return mMaterial;
}

SpatialVelocity&  WorldObject::GetSpatialVelocity(bool initial){
    if(initial)     return mInitialWorldVelocity;
    else            return mWorldVelocity;
}
SpatialVelocity&  WorldObject::GetRelativeSpatialVelocity(){
    return mRelativeVelocity;
}
void              WorldObject::SetSpatialVelocity(const SpatialVelocity& vel){
    mWorldVelocity  = vel;
    mWorldSpatialFrame.InverseMult(vel,mRelativeVelocity);
}
void              WorldObject::SetRelativeSpatialVelocity(const SpatialVelocity& vel){
    mRelativeVelocity = vel;
    mWorldSpatialFrame.Mult(vel,mWorldVelocity);
}


SpatialForce&       WorldObject::GetExternalForces(){
    return mWorldExternalForces;
}
SpatialForce&       WorldObject::GetExternalRelativeForces(){
    mWorldSpatialFrame.InverseMult(mWorldExternalForces,mRelativeExternalForces);
    return mRelativeExternalForces;
}

void                WorldObject::ClearExternalForces(){
    mWorldExternalForces.Zero();
    mRelativeExternalForces.Zero();
}
void                WorldObject::AddExternalForces(const SpatialForce& force){
    mWorldExternalForces += force;
}
void                WorldObject::AddExternalForce (const Vector3& force){
    mWorldExternalForces.GetLinearComponent() += force;
}
void                WorldObject::AddExternalTorque(const Vector3& torque){
    mWorldExternalForces.GetAngularComponent() += torque;
}
void                WorldObject::AddExternalForce (const Vector3& force, const Vector3 & pos){
    AddExternalForce(force);
    Vector3 relPos;
    mWorldReferenceFrame.GetHMatrix().Transform(mInertia.mCenterOfMass,relPos);
    relPos -= pos;
    AddExternalTorque(force.Cross(relPos));
    //applyTorque(rel_pos.cross(force*m_linearFactor));
}
void                WorldObject::AddExternalRelativeForce (const Vector3& force){
    Vector3 res;
    mWorldReferenceFrame.GetOrient().Mult(force,res);
    AddExternalForce(res);
}
void                WorldObject::AddExternalRelativeTorque(const Vector3& torque){
    Vector3 res;
    mWorldReferenceFrame.GetOrient().Mult(torque,res);
    AddExternalTorque(res);
}
void                WorldObject::AddExternalRelativeForce (const Vector3& force, const Vector3 & pos){
    Vector3 res;
    mWorldReferenceFrame.GetOrient().Mult(force,res);
    AddExternalForce(res);
    Vector3 relPos(mInertia.mCenterOfMass);
    relPos -= pos;
    mWorldReferenceFrame.GetOrient().Mult(force.Cross(relPos),res);
    AddExternalTorque(res);
}



bool        WorldObject::IsRobot(){
    return mRobot!=NULL;
}
Robot*      WorldObject::GetRobot(){
    return mRobot;
}
void        WorldObject::SetRobot(Robot *robot){
    mRobot = robot;
    if(mRobot){
        mRobot->SetWorldInstance(this);
    }
    //GetReferenceFrame(true).SetHMatrix() = mRobot->GetLinks()[0]->GetReferenceFrame(true).GetHMatrix();
}
bool WorldObject::IsSlave(){
    return bSlaveMode;
}


int         WorldObject::StreamSize(){
    int len = GetStreamStringSize("WorldObject");
    len += GetStreamStringSize(mName.c_str());
    len += sizeof(double);
    len += 16*sizeof(REALTYPE);
    if(mRobot!=NULL){
        len += GetStreamStringSize("Robot");
        LinksList& links = mRobot->GetLinks();
        len += sizeof(int);
        for(size_t i=0;i<links.size();i++){
            len += links[i]->StreamSize();
        }
    }
    return len;
}
int         WorldObject::StreamSizeFromStream(const void* memory){
    int res = CheckStreamTag("WorldObject",memory);
    if(res>0){
        char* mem = ((char*) memory) + res;
        mem += GetStreamStringSize(mem);
        mem += sizeof(double);
        mem += 16*sizeof(REALTYPE);
        if((res = CheckStreamTag("Robot",mem))>0){
            mem += res;
            int size = *((int*)mem);
            mem += sizeof(int);
            for(int i=0;i<size;i++){
                mem += sDummyObject.StreamSizeFromStream(mem);
            }
        }
        return ((char*)mem)-((char*)memory);
    }
    return 0;
}

int         WorldObject::SetStream(void* memory){
    char *mem = (char*)memory;
    mem += SetStreamString("WorldObject",mem);
    mem += SetStreamString(mName.c_str(),mem);
    // cout << mName<<endl;
    *((double*)mem) = 0.0;
    mem += sizeof(double);
    memcpy(mem,mWorldReferenceFrame.GetHMatrix().Array(),16*sizeof(REALTYPE));
    mem += 16*sizeof(REALTYPE);
    if(mRobot!=NULL){
        mem += SetStreamString("Robot",mem);
        LinksList& links = mRobot->GetLinks();
        *((int*)mem) = int(links.size());
        mem += sizeof(int);
        for(size_t i=0;i<links.size();i++){
            mem += links[i]->SetStream(mem);
        }
    }
    return ((char*)mem)-((char*)memory);
}
int         WorldObject::SetFromStream(const void* memory){
    int res = CheckStreamTag("WorldObject",memory);
    if(res>0){
        char* mem = ((char*) memory) + res;
        mName = mem;
        //cout << mName<<" "<<GetStreamStringSize(mName.c_str())<<endl;
        mem += GetStreamStringSize(mName.c_str());
        mem += sizeof(double);
        memcpy(mWorldReferenceFrame.SetHMatrix().Array(),mem,16*sizeof(REALTYPE));
        mem += 16*sizeof(REALTYPE);
        bSlaveMode = true;

        if(mRobot!=NULL){
            if((res = CheckStreamTag("Robot",mem))>0){
                mem += res;
                int size = *((int*)mem);
                mem += sizeof(int);
                LinksList& links = mRobot->GetLinks();
                for(int i=0;i<size;i++){
                    sDummyObject.SetFromStream(mem);
                    int index = mRobot->GetLinkIndex(sDummyObject.GetName());
                    if(index>=0){
                        mem += links[index]->SetFromStream(mem);
                    }else{
                        mem += sDummyObject.StreamSizeFromStream(mem);
                    }
                }
            }
        }
        return ((char*)mem)-((char*)memory) ;
    }
    return 0;
}

WorldObjectLink::WorldObjectLink(string objA, string objB){
    mObjAName = objA;
    mObjBName = objB;
}

void WorldObjectLink::Resolve(World* world){
    vector<string> oA = Tokenize(mObjAName);
    //cout << mObjAName<<endl;
    if(oA.size()>0){
        mObjA = world->Find(oA[0]);
        if(mObjA && (oA.size()>1) ){
            if(mObjA->IsRobot()){
                //cout <<oA[0]<<" "<<oA[1]<<" "<<mObjA->GetRobot()->GetLinkIndex(oA[1])<<endl;
                mObjA = mObjA->GetRobot()->GetLinks()[mObjA->GetRobot()->GetLinkIndex(oA[1])];
                //if(mObjA) cout << mObjA->GetName()<<endl;
            }
        }
    }

    vector<string> oB = Tokenize(mObjBName);
    //cout << mObjBName<<endl;
    if(oB.size()>0){
        mObjB = world->Find(oB[0]);
        if(mObjB && (oB.size()>1) ){
            if(mObjB->IsRobot()){
                mObjB = mObjB->GetRobot()->GetLinks()[mObjB->GetRobot()->GetLinkIndex(oB[1])];
                //if(mObjB) cout << mObjB->GetName()<<endl;
            }
        }
    }



}

pWorldObject    WorldObjectLink::GetObjectA(){
    return mObjA;
}
pWorldObject    WorldObjectLink::GetObjectB(){
    return mObjB;
}




