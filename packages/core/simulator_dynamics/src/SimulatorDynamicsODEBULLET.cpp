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


#include "SimulatorDynamicsODEBULLET.h"

#include "BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h"

REALTYPE  SimulatorDynamics::sGlobalScaling  = 10.0;
#define SCALING SimulatorDynamics::sGlobalScaling
#define INVSCALING (1.0/SimulatorDynamics::sGlobalScaling)

DynamicObject::DynamicObject(SimulatorDynamics *world)
:BaseDynamicObject(world){

    mODEBody         = NULL;
    mBTColShape      = NULL;
    mBTColObject     = NULL;

    mCOMFrame.Identity();
    mCOMSpatialFrame.Set(mCOMFrame);

    mCollisionGroupId       = 0;
    mCollisionGroupPartId   = 0;
    bIsKinematic            = false;
    bNeedContact            = false;
    bNeedReset              = false;
    bSetDynamics            = false;
}

DynamicObject::~DynamicObject(){
    Free();
}
void    DynamicObject::Free(){

    if(mODEBody)    dBodyDestroy(mODEBody);      mODEBody = NULL;
    if(mBTColObject!=NULL) mBTColObject->setCollisionShape(NULL);
    if(mBTColShape      !=NULL){
        btCompoundShape *cs = dynamic_cast<btCompoundShape*>(mBTColShape);
        if(cs!=NULL){
            int size = cs->getNumChildShapes();
            for(int i=0;i<size;i++){
                btCollisionShape *cols = cs->getChildShape(0);
                cs->removeChildShapeByIndex(0);
                delete cols;
            }
        }
        delete mBTColShape;     mBTColShape     = NULL;
    }

    if(mBTColObject!=NULL) delete mBTColObject; mBTColObject = NULL;

}

void    DynamicObject::SetKinematicOnly(bool only){
    bIsKinematic = only;
}

void DynamicObject::LinkToObject(WorldObject *object){
    if(object==mObject)
        return;

    BaseDynamicObject::LinkToObject(object);

    gLOG.SetCurrentEntry("BULLET Dynamics");
    gLOG.Append("Setting up object: %s",mObject->GetName().c_str());
    gLOG.SetDeltaIndent(2);


    gLOG.Append("Setting up physical shape");
    // Get the rigid body center of mass reference frame
    mCOMFrame.Identity();
    // its origin
    mCOMFrame.SetOrigin() = mObject->GetSpatialInertia().mCenterOfMass * SCALING;
    //mCOMFrame.SetOrigin().Print();

    Vector3 localInertia(0,0,0);

    // its orientation, useful only if there is some mass
    if(mObject->GetSpatialInertia().mMass > 0.0){
        Matrix inertia(3,3);
        inertia = mObject->GetSpatialInertia().mInertiaMoment * (SCALING*SCALING);
        
        // Checking if inertia tensor is fine
        if( (inertia.AtNoCheck(0,0)>0) || (inertia.AtNoCheck(1,1)>0) || (inertia.AtNoCheck(2,2)>0) ){
            // Checking if inertia tensor is non diagonal
            if( (inertia.AtNoCheck(1,0)!=0) || (inertia.AtNoCheck(2,0)!=0) || (inertia.AtNoCheck(2,1)!=0) ){
                Vector diag(3);
                Matrix base(3,3);
                inertia.EigenValuesDecomposition(diag, base);
                localInertia.Set(diag.AtNoCheck(0),diag.AtNoCheck(1),diag.AtNoCheck(2));
                base.STranspose();
                Vector3 res;
                Vector3(base.GetColumn(0).Array()).Cross(Vector3(base.GetColumn(1).Array()),res);
                base.SetColumn(res,2);
                mCOMFrame.SetOrient().Set(base.Array());
            }else{
                localInertia = Vector3(inertia.AtNoCheck(0,0),inertia.AtNoCheck(1,1),inertia.AtNoCheck(2,2));
            }
        }
    }
    Matrix4 invTrans = mCOMFrame.GetInverse().GetHMatrix();
    btTransform invTransform;
    invTransform.setOrigin(btVector3(invTrans.AtNoCheck(0,3),invTrans.AtNoCheck(1,3),invTrans.AtNoCheck(2,3)));
    invTransform.setBasis(btMatrix3x3(invTrans.AtNoCheck(0,0),invTrans.AtNoCheck(0,1),invTrans.AtNoCheck(0,2),
                                      invTrans.AtNoCheck(1,0),invTrans.AtNoCheck(1,1),invTrans.AtNoCheck(1,2),
                                      invTrans.AtNoCheck(2,0),invTrans.AtNoCheck(2,1),invTrans.AtNoCheck(2,2)));


    dMass odeMass;
    REALTYPE mass    = mObject->GetSpatialInertia().mMass;

    if(mass>0.0){
        if(localInertia.Norm2()==0){
            dMassSetSphereTotal(&odeMass,mass,0.1);
        }else{
            dMassSetParameters (&odeMass, mass,
                                0,0,0,
                                localInertia.cx(),localInertia.cy(),localInertia.cz(),
                                0,0,0);
        }
        bIsKinematic = false;
    }else{
        gLOG.Append("Warning: body is kinematic");
        bIsKinematic = true;
    }

    if(!bIsKinematic){
        mODEBody = dBodyCreate(((SimulatorDynamics*)mDynWorld)->mODEWorld);
        dBodySetMass(mODEBody,&odeMass);
        dBodySetData(mODEBody,this);
        dBodyEnable(mODEBody);
    }else{
        mODEBody = NULL;
    }

    // Get the shape tree
    gLOG.Append("Setting up collision shape");
    pXmlTree tree = mObject->GetBBoxShapeTree();
    if(tree==NULL){
        gLOG.Append("Warning: no shape present.");// Not using it a physical shape");
        //gLOG.SetDeltaIndent(-2);
        //return;
    }

    // Create the main shape
    if(tree!=NULL){
        REALTYPE *array;
        pXmlTreeList tmpList = tree->GetSubTrees();
        pXmlTree     tmpTree;
        int shapeCnt = 0;
        for(int i=0;i<int(tmpList->size());i++){
            tmpTree = tmpList->at(i);
            if((tmpTree->GetName()=="Shape")){
                gLOG.Append("Setting up Shape : %s",tmpTree->GetData().c_str());
                gLOG.SetDeltaIndent(2);

                Vector3     origin;
                Matrix3     orient;
                Matrix4     ref;
                btTransform btRef;

                origin.Zero();
                orient.Identity();

                int size;
                if(tmpTree->Find("Origin")){
                    size=tmpTree->GetArray("Origin",&array);
                    if(size==3){            origin.Set(array); origin *= SCALING;
                    }else{                  gLOG.Append("Error: Bad <Origin> array size (should be 3)");
                    }
                }else{
                                            gLOG.Append("Warning: No <Origin> defined");
                }

                if(tmpTree->Find("Orient")){
                    size=tmpTree->GetArray("Orient",&array);
                    if(size==9){            orient.Set(array);
                                            orient.Normalize();
                    }else if(size==3){      orient.SRotationV(Vector3(array));
                    }else{                  gLOG.Append("Error: Bad <Orient> array size (should be 3(axis*angle) or 9(full rotation matrix))");
                    }
                }else{
                                            gLOG.Append("Warning: No <Orient> defined");
                }
                ref.SetTranslation(origin);
                ref.SetOrientation(orient);
                btRef.setOrigin(btVector3(  ref.AtNoCheck(0,3),ref.AtNoCheck(1,3),ref.AtNoCheck(2,3)));
                btRef.setBasis (btMatrix3x3(ref.AtNoCheck(0,0),ref.AtNoCheck(0,1),ref.AtNoCheck(0,2),
                                            ref.AtNoCheck(1,0),ref.AtNoCheck(1,1),ref.AtNoCheck(1,2),
                                            ref.AtNoCheck(2,0),ref.AtNoCheck(2,1),ref.AtNoCheck(2,2)));

                Vector3 scale;
                scale.x() = scale.y() = scale.z() = R_ONE;
                if(tmpTree->Find("Scale")){
                    size=tmpTree->GetArray("Scale",&array);
                    if(size==3){        scale.Set(array);
                    }else{              gLOG.Append("Error: Bad <Scale> array size (should be 3)");
                    }
                }else{
                                        gLOG.Append("Warning: No <Scale> set");
                }
                scale *=  SCALING;


                btCollisionShape* shape = NULL;


                      if(tmpTree->GetData() == "Floor"){
                        shape = new btBoxShape(btVector3(btScalar(10.) * SCALING,btScalar(10.) * SCALING,btScalar(0.5) * SCALING));
                        btVector3 orig = btRef.getOrigin();
                        orig.setZ(orig.z() + -0.5 * SCALING);
                        btRef.setOrigin(orig);
                }else if(tmpTree->GetData() == "Cube"){
                        shape = new btBoxShape(btVector3(scale.cx()*0.5,scale.cy()*0.5,scale.cz()*0.5));
                }else if(tmpTree->GetData() == "Cylinder"){
                        shape = new btCylinderShapeZ(btVector3(scale.cx()*0.5,scale.cy()*0.5,scale.cz()*0.5));
                }else if(tmpTree->GetData() == "Sphere"){
                        shape = new btSphereShape(btScalar((scale.cx()+scale.cy()+scale.cz())/3.0*0.5));
                }else if(tmpTree->GetData() == "Capsule"){
                        shape = new btCapsuleShapeZ(btScalar((scale.cx()+scale.cy())*0.25),btScalar(scale.cz()));
                }else if(tmpTree->GetData() == "HeightField"){
                        //gLOG.Append("Test: %s",tmpTree->GetBasePath().c_str());
                        if(tmpTree->Find("DataFile")){
                            string filename = tmpTree->GetBasePath()+string("/")+tmpTree->Find("DataFile")->GetData();
                            if(mHeightField.Load(filename.c_str())){
                                //mHeightField*=SCALING;
                                mHeightField*=scale.cz()*INVSCALING;
                                //mHeightField.Print();
                                REALTYPE mmin = mHeightField.Min();
                                REALTYPE mmax = mHeightField.Max();
                                btHeightfieldTerrainShape * heightfieldShape =
                                    new btHeightfieldTerrainShape(mHeightField.ColumnSize(), mHeightField.RowSize(),
                                                  mHeightField.Array(),
                                                  1.0,
                                                  mmin,mmax,
                                                  2,
                                                  PHY_FLOAT, true);
                                    //heightfieldShape->setLocalScaling(btVector3(scale.cx()*SCALING,scale.cy()*SCALING,scale.cz()*SCALING));
                                    heightfieldShape->setLocalScaling(btVector3(scale.cx(),scale.cy(),SCALING));//scale.cz()));
                                    btRef.setOrigin(btVector3(  ref.AtNoCheck(0,3),ref.AtNoCheck(1,3),ref.AtNoCheck(2,3)+mmin*SCALING+(mmax-mmin)*SCALING/2.0));
                                    shape = heightfieldShape;
                            }else{
                                gLOG.Append("Error: Height field file %s failed to open",filename.c_str());
                            }
                            //cout << tmpTree->Find("DataFile")->GetData()<<endl;
                        }
                        //shape = new btCapsuleShapeZ(btScalar((scale.cx()+scale.cy())*0.25),btScalar(scale.cz()));
                }else{
                        gLOG.Append("Error: No other type of shape possible for now: %s",tmpTree->GetData().c_str());
                }


                if(shape!=NULL){
                    if(shapeCnt==0){
                        // Create the main shape
                        mBTColShape = new btCompoundShape();
                        mBTColShape->setMargin(0);
                    }
                    btRef = invTransform*btRef;
                    shape->setMargin(0);
                    ((btCompoundShape*)mBTColShape)->addChildShape(btRef, shape);
                }

                gLOG.SetDeltaIndent(-2);
                shapeCnt++;
            }
        }
    }

    if(mBTColShape!=NULL){
        ((SimulatorDynamics*)mDynWorld)->mBTCollisionShapes.push_back(mBTColShape);
        mBTColObject = new btCollisionObject();
        mBTColObject->setCollisionShape(mBTColShape);
        mBTColObject->setUserPointer(this);

        ((SimulatorDynamics*)mDynWorld)->mBTCollisionWorld->addCollisionObject(mBTColObject);
    }else{
        gLOG.Append("Warning: No shape set. Considering as a pure physical entity (no collision)");
    }

    bSetDynamics = true;
    DynamicObject::Prepare();

    gLOG.SetDeltaIndent(-2);
}


void DynamicObject::Prepare(){
    //if(bNeedReset) bSetDynamics = true;
    //bNeedReset = false;
    if(mObject!=NULL){
        mObject->mContacts.clear();
        mObject->mContactsNormal.clear();

        if(mObject->IsSlave()){
            bIsKinematic = true;
            if(mODEBody!=NULL){
                if(dBodyIsEnabled(mODEBody)){
                    dBodyDisable(mODEBody);
                }
            }
        }

        if((mODEBody!=NULL)||(mBTColObject!=NULL)){

            Matrix4 ref,res;
            ref = mObject->GetReferenceFrame(bNeedReset).GetHMatrix();
            ref.RefNoCheck(0,3) *= SCALING;
            ref.RefNoCheck(1,3) *= SCALING;
            ref.RefNoCheck(2,3) *= SCALING;
            ref.Mult(mCOMFrame.GetHMatrix(),res);

            if(mBTColObject!=NULL){
                btTransform transform;
                transform.setOrigin(btVector3 (res.AtNoCheck(0,3),res.AtNoCheck(1,3),res.AtNoCheck(2,3)));
                transform.setBasis(btMatrix3x3(res.AtNoCheck(0,0),res.AtNoCheck(0,1),res.AtNoCheck(0,2),
                                               res.AtNoCheck(1,0),res.AtNoCheck(1,1),res.AtNoCheck(1,2),
                                               res.AtNoCheck(2,0),res.AtNoCheck(2,1),res.AtNoCheck(2,2)));
                mBTColObject->getWorldTransform() = transform;
            }

            if(mODEBody!=NULL){
                if(bNeedReset || bSetDynamics || bIsKinematic ){
                    dBodySetPosition(mODEBody, res.AtNoCheck(0,3),res.AtNoCheck(1,3),res.AtNoCheck(2,3));
                    dBodySetRotation(mODEBody, res.Array());
                }
                if(bNeedReset || bSetDynamics){
                    //SpatialVelocity
                    Vector3 avel = mObject->GetSpatialVelocity(bNeedReset).GetAngularComponent();
                    Vector3 vel  = mObject->GetSpatialVelocity(bNeedReset).GetLinearComponent();
                    vel *= SCALING;
                    Vector3 tmpVel;
                    Vector3 rdist = res.GetTranslation()-ref.GetTranslation();
                    rdist.Cross(avel,tmpVel);
                    vel += tmpVel;
                    dBodySetAngularVel(mODEBody, avel.cx(),
                                                 avel.cy(),
                                                 avel.cz());
                    dBodySetLinearVel (mODEBody, vel.cx(),
                                                 vel.cy(),
                                                 vel.cz());
                }
            }
            if(mODEBody!=NULL){
                Vector3 &force  = mObject->GetExternalForces().GetLinearComponent();
                Vector3 &torque = mObject->GetExternalForces().GetAngularComponent();
                dBodyAddForce (mODEBody,force.cx()*SCALING,force.cy()*SCALING,force.cz()*SCALING);
                dBodyAddTorque(mODEBody,torque.cx()*SCALING*SCALING,torque.cy()*SCALING*SCALING,torque.cz()*SCALING*SCALING);
            }
        }
        bNeedReset   = false;
        bSetDynamics = false;
    }
}

void DynamicObject::Update(REALTYPE dt){
    if(mObject!=NULL){
        if(mODEBody!=NULL){
            Matrix4 ref;
            const REALTYPE * p = dBodyGetPosition(mODEBody);
            const REALTYPE * r = dBodyGetRotation(mODEBody);
            ref.Set(r[ 0],r[ 1],r[ 2],p[0],
                        r[ 4],r[ 5],r[ 6],p[1],
                        r[ 8],r[ 9],r[10],p[2],
                        0.0,0.0,0.0,1.0);
            Matrix4 res;
            ref.Mult(mCOMFrame.GetInverse().GetHMatrix(),res);
            res.RefNoCheck(0,3) *= INVSCALING;
            res.RefNoCheck(1,3) *= INVSCALING;
            res.RefNoCheck(2,3) *= INVSCALING;
            mObject->GetReferenceFrame().SetHMatrix() = res;
            
            const REALTYPE * v = dBodyGetLinearVel(mODEBody);
            const REALTYPE * w = dBodyGetAngularVel(mODEBody);
            mObject->SetSpatialVelocity(SpatialVelocity(w[0],w[1],w[2],v[0],v[1],v[2]));

        }
        mObject->ClearExternalForces();
    }
}

void    DynamicObject::Reset(){
    bNeedReset = true;
    Prepare();
    Update(0);
}

BaseDynamicObject*  DynamicObject::GetDynamicObject(WorldObject *object){
    //cout << object<<endl;
    if(mObject == object) return this;
    return NULL;
}


DynamicRobot::DynamicRobot(SimulatorDynamics *world)
:DynamicObject(world){
    mRobot          = NULL;
    mSimRobot       = NULL;
}

DynamicRobot::~DynamicRobot(){
    Free();
}

void    DynamicRobot::Free(){
    mRobot      = NULL;
    mSimRobot   = NULL;
    for(int i=0;i<int(mDynJoints.size());i++){
        dJointDestroy(mDynJoints[i]);
        if(mDynJointsFeedback[i]!=NULL)
            delete mDynJointsFeedback[i];
    }
    mDynJoints.clear();
    mDynJointsFeedback.clear();
    mDynJointsOffset.clear();
    for(int i=0;i<int(mDynLinks.size());i++){
        delete mDynLinks[i];
    }
    mDynLinks.clear();
    mRobotJoints.clear();

    DynamicObject::Free();
}

void    DynamicRobot::LinkToObject(WorldObject *object){
    if(object->IsRobot()){
        LinkToRobot(object->GetRobot());
    }
}

void    DynamicRobot::LinkToRobot(Robot* robot){
    if(mRobot==robot) return;

    if(robot==NULL){
        Free();
        return;
    }

    mRobot = robot;
    if(mSimRobot ==NULL)
        mSimRobot = mRobot;

    gLOG.SetCurrentEntry("BULLET Dynamics");
    gLOG.Append("Setting up Robot: %s",mRobot->GetType().c_str());
    gLOG.SetDeltaIndent(2);


    mSensorsGroup.SetSensorsList(mSimRobot->GetSensors());
    mActuatorsGroup.SetActuatorsList(mSimRobot->GetActuators());
    mDifferentiator.Init(mSimRobot->GetDOFCount(),2);
    mIntegrator.Init(mSimRobot->GetDOFCount(),2);


    int linkCount = mRobot->GetLinksCount();
    if(linkCount>0){

        // Setting up mean position
        gLOG.Append("Putting robot in median posture");

        Vector meanOff(mSimRobot->GetDOFCount());
        Vector zero(mSimRobot->GetDOFCount());
        Vector zeroOff(mSimRobot->GetDOFCount());

        int jCnt = 0;
        for(int i=1;i<linkCount;i++){
            int jointType = mRobot->GetLinks()[i]->mJoint->GetObjectType();
            if(jointType == RevoluteJoint::GetClassType()){
                pRevoluteJoint joint     = RevoluteJoint::Cast(mRobot->GetLinks()[i]->mJoint);
                meanOff(jCnt)  = 0.5*(joint->mRange[0]+joint->mRange[1]);
                zeroOff(jCnt)  = joint->mZero;
                jCnt++;
            }else if(jointType == SliderJoint::GetClassType()){
                pSliderJoint joint     = SliderJoint::Cast(mRobot->GetLinks()[i]->mJoint);
                meanOff(jCnt)  = 0.5*(joint->mRange[0]+joint->mRange[1]);
                zeroOff(jCnt)  = joint->mZero;
                jCnt++;
            }
        }


        mSensorsGroup.ReadSensors();
        mSensorsGroup.SetJointAngles(meanOff);
        mSensorsGroup.SetJointVelocities(zero);
        mSensorsGroup.SetJointAccelerations(zero);
        mSensorsGroup.WriteSensors();
        mSimRobot->UpdateLinks();



        // hinge lock
        // kin -> pos -> kin
        // kin -> trq -> nrm
        // nrm -> trq -> nrm
        // nrm -> pos ->

        if(mRobot->GetLinks()[0]->mJoint->mControlMode!=Joint::JCTRLMODE_TORQUE)
            SetKinematicOnly(true);
        DynamicObject::LinkToObject(mRobot->GetLinks()[0]);
        bNeedContact = true;

        mCollisionGroupId       = 1;
        mCollisionGroupPartId   = 0;

        for(int i=1;i<linkCount;i++){
            DynamicObject *obj = new DynamicObject((SimulatorDynamics*)mDynWorld);
            if(mRobot->GetLinks()[i]->mJoint->mControlMode!=Joint::JCTRLMODE_TORQUE)
                obj->SetKinematicOnly(true);
            obj->LinkToObject(mRobot->GetLinks()[i]);
            obj->bNeedContact = true;
            obj->mCollisionGroupId       = 1;
            obj->mCollisionGroupPartId   = 0;
            mDynLinks.push_back(obj);
        }

        // Setting up joints
        for(int i=1;i<linkCount;i++){

            //cout << "---"<<endl;
            int pId = mRobot->GetParents()[i];
            //cout << pId<< "<-"<<i<<endl;
            DynamicObject *dObjA = (pId==0?this:mDynLinks[pId-1]);
            DynamicObject *dObjB = mDynLinks[i-1];

            dObjB->mCollisionGroupPartId   = dObjA->mCollisionGroupPartId+1;

            //cout << dObjA<<" "<<dObjB<<endl;
            /*
               DynamicObject *dObjA = (i==1?this:mDynLinks[i-2]);
            DynamicObject *dObjB = mDynLinks[i-1];
            */


            gLOG.Append("Setting up joint <%d> between %s and %s",i-1,dObjA->mObject->GetName().c_str(),dObjB->mObject->GetName().c_str());

            dBodyID objA = dObjA->mODEBody;
            dBodyID objB = dObjB->mODEBody;

            bool okA = (dObjA->mODEBody!=NULL) || (dObjA->mBTColShape!=NULL);
            bool okB = (dObjB->mODEBody!=NULL) || (dObjB->mBTColShape!=NULL);
            //if((!okA)||(!okB)){
                //if((objA==NULL)&&(objB==NULL)){
                    if(pId>0){
                    if(!okA){
                        gLOG.SetDeltaIndent(2);
                        gLOG.Append("Error: shape of object %s is not properly set-up... skipping",dObjA->mObject->GetName().c_str());
                        gLOG.SetDeltaIndent(-2);
                        continue;
                    }
                    }
                    if(!okB){
                        gLOG.SetDeltaIndent(2);
                        gLOG.Append("Error: shape of object %s is not properly set-up... skipping",dObjB->mObject->GetName().c_str());
                        gLOG.SetDeltaIndent(-2);
                        continue;
                    }
                //}
            //}            
            REALTYPE offset     = 0.0;
            REALTYPE zeroOffset = 0.0;

            Matrix4 res,ref;
            ref = ((Link*)dObjB->mObject)->GetReferenceFrame().GetHMatrix();
            ref.RefNoCheck(0,3) *= SCALING;
            ref.RefNoCheck(1,3) *= SCALING;
            ref.RefNoCheck(2,3) *= SCALING;

            dJointID        odeJoint      = NULL;
            dJointFeedback* jointFeedback = NULL;

            int jointType = mRobot->GetLinks()[i]->mJoint->GetObjectType();
            if(jointType == RevoluteJoint::GetClassType()){
                pRevoluteJoint joint     = RevoluteJoint::Cast(((Link*)dObjB->mObject)->mJoint);

                odeJoint = dJointCreateHinge(((SimulatorDynamics*)mDynWorld)->mODEWorld,0);
                dJointAttach(odeJoint, objB, objA);

                dJointSetHingeAnchor(odeJoint, ref.AtNoCheck(0,3),ref.AtNoCheck(1,3),ref.AtNoCheck(2,3));
                dJointSetHingeAxis(odeJoint, ref.AtNoCheck(0,2),ref.AtNoCheck(1,2),ref.AtNoCheck(2,2));

                offset      = 0.5*(joint->mRange[0]+joint->mRange[1]);
                zeroOffset  = joint->mZero;

                dJointSetHingeParam(odeJoint, dParamLoStop, joint->mRange[0]-offset);
                dJointSetHingeParam(odeJoint, dParamHiStop, joint->mRange[1]-offset);
                dJointSetHingeParam(odeJoint, dParamVel, 0);
                dJointSetHingeParam(odeJoint, dParamFMax, (joint->mFrictionCoefs[0])*SCALING*SCALING);
                gLOG.SetDeltaIndent(2);
                gLOG.Append("Setting up hinge joint");
                gLOG.SetDeltaIndent(-2);

            }else if(jointType == SliderJoint::GetClassType()){
                pSliderJoint joint     = SliderJoint::Cast(((Link*)dObjB->mObject)->mJoint);

                odeJoint = dJointCreateSlider(((SimulatorDynamics*)mDynWorld)->mODEWorld,0);
                dJointAttach(odeJoint, objB, objA);

                dJointSetSliderAxis(odeJoint, ref.AtNoCheck(0,2),ref.AtNoCheck(1,2),ref.AtNoCheck(2,2));

                offset      = 0.5*(joint->mRange[0]+joint->mRange[1]);
                zeroOffset  = joint->mZero;

                dJointSetSliderParam(odeJoint, dParamLoStop, (joint->mRange[0]-offset)*SCALING);
                dJointSetSliderParam(odeJoint, dParamHiStop, (joint->mRange[1]-offset)*SCALING);
                dJointSetSliderParam(odeJoint, dParamVel, 0);
                dJointSetSliderParam(odeJoint, dParamFMax, (joint->mFrictionCoefs[0])*SCALING);

                gLOG.SetDeltaIndent(2);
                gLOG.Append("Setting up slider joint");
                gLOG.SetDeltaIndent(-2);

            }else if(jointType == ForceSensorJoint6DOF::GetClassType()){
                //pForceSensorJoint6DOF joint     = ForceSensorJoint6DOF::Cast(((Link*)dObjB->mObject)->mJoint);

                odeJoint = dJointCreateFixed(((SimulatorDynamics*)mDynWorld)->mODEWorld,0);
                dJointAttach(odeJoint, objB, objA);
                dJointSetFixed(odeJoint);
                jointFeedback = new dJointFeedback();
                dJointSetFeedback (odeJoint, jointFeedback);

                gLOG.SetDeltaIndent(2);
                gLOG.Append("Setting up force sensor joint");
                gLOG.SetDeltaIndent(-2);
            }else{
                odeJoint = dJointCreateFixed(((SimulatorDynamics*)mDynWorld)->mODEWorld,0);
                dJointAttach(odeJoint, objB, objA);
                dJointSetFixed(odeJoint);
                gLOG.SetDeltaIndent(2);
                gLOG.Append("Warning: Don't know how to deal with this type of joint. Setting a fixed constraint");
                gLOG.SetDeltaIndent(-2);
            }

            //((SimulatorDynamics*)mDynWorld)->mBTDynamicsWorld->addConstraint(hinge, true);

            mDynJoints.push_back(odeJoint);
            mDynJointsOffset.push_back(offset);
            mDynJointsFeedback.push_back(jointFeedback);
            mRobotJoints.push_back(mRobot->GetLinks()[i]->mJoint);
        }


        gLOG.Append("Restoring robot zero posture");

        
        mSensorsGroup.ReadSensors();
        //cout << mSensorsGroup.GetJointAngles().Size()<<endl;;

        mSensorsGroup.SetJointAngles(zeroOff);
        mSensorsGroup.SetJointVelocities(zero);
        mSensorsGroup.SetJointAccelerations(zero);
        mSensorsGroup.WriteSensors();
        mSimRobot->UpdateLinks();

        
        Reset();

        /*
        mSensorsGroup.SetJointAngles(zeroOff);
        mSensorsGroup.SetJointVelocities(zero);
        mSensorsGroup.SetJointAccelerations(zero);
        mSensorsGroup.WriteSensors();
        mSimRobot->UpdateLinks();
            */
    }else{
        Free();
    }

    mGTime = 0;
    gLOG.SetDeltaIndent(-2);
}

void    DynamicRobot::Prepare(){

    DynamicObject::Prepare();
    for(int i=0;i<int(mDynLinks.size());i++){
        mDynLinks[i]->Prepare();
    }


    // Copy actuators from outside robot
    if(mSimRobot!=mRobot){
        ActuatorsList& actListSrc = mRobot->GetActuators();
        ActuatorsList& actListDst = mSimRobot->GetActuators();
        unsigned int size = MIN(actListSrc.size(),actListDst.size());
        for(unsigned int i=0;i<size;i++){
            actListDst[i]->Set(*actListSrc[i]);
        }
        SensorsList& senListSrc = mSimRobot->GetSensors();
        SensorsList& senListDst = mRobot->GetSensors();
        unsigned int size2 = MIN(senListSrc.size(),senListDst.size());
        for(unsigned int i=0;i<size2;i++){
            senListDst[i]->Set(*senListSrc[i]);
        }
    }
    mSensorsGroup.ReadSensors();
    mActuatorsGroup.ReadActuators();

    int jCnt = 0;
    switch(mRobot->GetLinks()[0]->mJoint->mControlMode){
    case Joint::JCTRLMODE_TORQUE:
        {
        Vector out = mActuatorsGroup.GetJointTorques();

        for(int i=0;i<int(mDynJoints.size());i++){
            if(mRobotJoints[i]->mControlMode == Joint::JCTRLMODE_TORQUE){
                dBodyID dobjA = dJointGetBody(mDynJoints[i], 1);
                dBodyID dobjB = dJointGetBody(mDynJoints[i], 0);
                DynamicObject *objB = ((DynamicObject*)dBodyGetData(dobjB));
                Matrix4 ref = objB->mObject->GetReferenceFrame().GetHMatrix();
                switch(dJointGetType(mDynJoints[i])){
                case dJointTypeHinge:
                    dBodyAddTorque(dobjB,ref.AtNoCheck(0,2)*out(jCnt)*SCALING*SCALING,ref.AtNoCheck(1,2)*out(jCnt)*SCALING*SCALING,ref.AtNoCheck(2,2)*out(jCnt)*SCALING*SCALING);
                    if(dobjA){
                        dBodyAddTorque(dobjA,-ref.AtNoCheck(0,2)*out(jCnt)*SCALING*SCALING,-ref.AtNoCheck(1,2)*out(jCnt)*SCALING*SCALING,-ref.AtNoCheck(2,2)*out(jCnt)*SCALING*SCALING);
                    }
                    jCnt++;
                    break;
                case dJointTypeSlider:
                    dBodyAddForce(dobjB,ref.AtNoCheck(0,2)*out(jCnt)*SCALING,ref.AtNoCheck(1,2)*out(jCnt)*SCALING,ref.AtNoCheck(2,2)*out(jCnt)*SCALING);
                    if(dobjA){
                        dBodyAddForce(dobjA,-ref.AtNoCheck(0,2)*out(jCnt)*SCALING,-ref.AtNoCheck(1,2)*out(jCnt)*SCALING,-ref.AtNoCheck(2,2)*out(jCnt)*SCALING);
                    }
                    jCnt++;
                    break;
                default:
                    break;
                }
            }
        }
        }break;
    default:
        break;
    }
}
void    DynamicRobot::Update(REALTYPE dt){

    mGTime += dt;

    DynamicObject::Update(dt);
    if(mRobot->GetWorldInstance()){
        mRobot->GetWorldInstance()->GetReferenceFrame() = mObject->GetReferenceFrame();
        mRobot->GetWorldInstance()->SetSpatialVelocity(mObject->GetSpatialVelocity());
        
    }

    for(int i=0;i<int(mDynLinks.size());i++){
        mDynLinks[i]->Update(dt);
    }

    //cout << mDynLinks.size()<<" --- "<<mDynJoints.size()<<endl;

    //if(mRobot->GetControlMode()!=Robot::CTRLMODE_TORQUE) return;
    int jCnt = 0;
    switch(mRobot->GetLinks()[0]->mJoint->mControlMode){
    case Joint::JCTRLMODE_TORQUE:
        {

            Vector outpos(mSimRobot->GetDOFCount());
            Vector outvel(mSimRobot->GetDOFCount());
            Vector zero(mSimRobot->GetDOFCount());

            
            for(int i=0;i<int(mDynJoints.size());i++){
                //cout << mRobotJoints[i]->mControlMode<<endl;
                switch(dJointGetType(mDynJoints[i])){
                case dJointTypeHinge:
                    outpos(jCnt) = dJointGetHingeAngle(mDynJoints[i]) + mDynJointsOffset[i];
                    outvel(jCnt) = dJointGetHingeAngleRate(mDynJoints[i]);
                    jCnt++;
                    break;
                case dJointTypeSlider:
                    outpos(jCnt) = dJointGetSliderPosition(mDynJoints[i])*INVSCALING + mDynJointsOffset[i];
                    outvel(jCnt) = dJointGetSliderPosition(mDynJoints[i])*INVSCALING;
                    jCnt++;
                    break;
                default:
                    break;
                    //outpos(i) = 0.0;
                    //outvel(i) = 0.0;
                }
                if(mDynJointsFeedback[i]!=NULL){
                    ForceSensorJoint6DOFSensor* sensor = ForceSensorJoint6DOFSensor::Cast(mRobotJoints[i]->mSensor);
                    Vector3 sTrq(mDynJointsFeedback[i]->t2[0]*INVSCALING*INVSCALING,mDynJointsFeedback[i]->t2[1]*INVSCALING*INVSCALING,mDynJointsFeedback[i]->t2[2]*INVSCALING*INVSCALING);
                    Vector3 sFor(mDynJointsFeedback[i]->f2[0]*INVSCALING,mDynJointsFeedback[i]->f2[1]*INVSCALING,mDynJointsFeedback[i]->f2[2]*INVSCALING);
                    Vector3 dTrq,dFor;
                    mDynLinks[i]->mObject->GetReferenceFrame().GetInverse().GetOrient().Mult(sTrq,dTrq);
                    mDynLinks[i]->mObject->GetReferenceFrame().GetInverse().GetOrient().Mult(sFor,dFor);

                    //cout << i<< " " <<mDynLinks[i]->mObject->GetReferenceFrame().GetOrient()<<endl;


                    sensor->Set(SpatialForce(sTrq,dFor));
                }
            }

        //    cout << int(mDynJoints.size())<<" "<<mSimRobot->GetDOFCount()<<endl;
            mDifferentiator.SetInput(outvel);
            mDifferentiator.Update(dt);

            mSensorsGroup.SetJointAngles(outpos);
            mSensorsGroup.SetJointVelocities(outvel);
            mSensorsGroup.SetJointAccelerations(mDifferentiator.GetOutput(1));
        } break;

    case Joint::JCTRLMODE_POSITION:
        {
            Vector out(mSimRobot->GetDOFCount());
            Vector zero(mSimRobot->GetDOFCount());

            mDifferentiator.SetInput(mActuatorsGroup.GetJointAngles());
            mDifferentiator.Update(dt);

            mSensorsGroup.SetJointAngles(mActuatorsGroup.GetJointAngles());
            mSensorsGroup.SetJointVelocities(mDifferentiator.GetOutput(1));
            mSensorsGroup.SetJointAccelerations(mDifferentiator.GetOutput(2));
            //mSensorsGroup.WriteSensors();

            //mInvDynamics.Update();
            //mInvDynamics.GetTorques(out);
            mSensorsGroup.SetJointTorques(zero);

        } break;

    case Joint::JCTRLMODE_VELOCITY:
        {
            Vector out(mSimRobot->GetDOFCount());
            Vector zero(mSimRobot->GetDOFCount());

            mIntegrator.SetInput(0,mSensorsGroup.GetJointAngles());
            mIntegrator.SetInput(1,mActuatorsGroup.GetJointVelocities());
            mIntegrator.SetInput(2,zero);
            mIntegrator.Update(dt);

            mDifferentiator.SetInput(mActuatorsGroup.GetJointVelocities());
            mDifferentiator.Update(dt);

            mSensorsGroup.SetJointAngles(mIntegrator.GetOutput(0));
            mSensorsGroup.SetJointVelocities(mActuatorsGroup.GetJointVelocities());
            mSensorsGroup.SetJointAccelerations(mDifferentiator.GetOutput(1));

        } break;
    case Joint::JCTRLMODE_ACCELERATION:
        {
            Vector out(mSimRobot->GetDOFCount());
            Vector zero(mSimRobot->GetDOFCount());

            mIntegrator.SetInput(0,mSensorsGroup.GetJointAngles());
            mIntegrator.SetInput(1,mSensorsGroup.GetJointVelocities());
            mIntegrator.SetInput(2,mActuatorsGroup.GetJointAccelerations());
            mIntegrator.Update(dt);

            mSensorsGroup.SetJointAngles(mIntegrator.GetOutput(0));
            mSensorsGroup.SetJointVelocities(mIntegrator.GetOutput(1));
            mSensorsGroup.SetJointAccelerations(mActuatorsGroup.GetJointAccelerations());

            //mInvDynamics.Update();
            //mInvDynamics.GetTorques(out);
            mSensorsGroup.SetJointTorques(zero);
            //mSensorsGroup.WriteSensors();
        }break;
    default : // --> should not happen, and removes gcc warning Wall
      break;
    }




    //if(!mObject->IsSlave()){
        mSensorsGroup.WriteSensors();

        // Copy sensors to outside robot
        if(mSimRobot!=mRobot){
            SensorsList& senListSrc = mSimRobot->GetSensors();
            SensorsList& senListDst = mRobot->GetSensors();
            unsigned int size = MIN(senListSrc.size(),senListDst.size());
            for(unsigned int i=0;i<size;i++){
                senListDst[i]->Set(*senListSrc[i]);
            }
            mSimRobot->UpdateLinks();
        }
        mRobot->UpdateLinks();
    //}

}

void    DynamicRobot::Reset(){
    if(SimulatorDynamics::bAllowRobotReset){
        DynamicObject::Reset();
        for(int i=0;i<int(mDynLinks.size());i++){
            mDynLinks[i]->Reset();
        }
        mDifferentiator.Reset();
        mIntegrator.Reset();
    }
}

BaseDynamicObject*  DynamicRobot::GetDynamicObject(WorldObject *object){
    if(mObject == object) return this;
    for(int i=0;i<int(mDynLinks.size());i++){
        BaseDynamicObject* obj;
        //cout << object<<endl;
        if((obj=mDynLinks[i]->GetDynamicObject(object)))
            return obj;
    }

    return NULL;
}





SimulatorDynamics::SimulatorDynamics()
:BaseSimulatorDynamics(){

    dInitODE();

    mBTCollisionWorld   = NULL;
    mBTBroadphase = NULL;
    mBTDispatcher = NULL;
    mBTCollisionConfiguration = NULL;

    mGroundObject = NULL;
    mGroundShape = NULL;

    mODEContacts = NULL;
    mODEWorld = NULL;

    mGravity.Set(0,0,-9.81);
    Init();
}

SimulatorDynamics::~SimulatorDynamics(){
    Free();
    dCloseODE();
}

void    SimulatorDynamics::SetGravity(const Vector3 & gravity){
    mGravity = gravity;
}

void    SimulatorDynamics::SetDamping(double damping){
    mDamping = damping;
}

void    SimulatorDynamics::Init(){
    Free();


    mODEWorld       = dWorldCreate();
    mODEContacts    = dJointGroupCreate(0);

    dWorldSetGravity (mODEWorld,mGravity.cx() * SCALING,mGravity.cy() * SCALING,mGravity.cz() * SCALING);
    //dWorldSetGravity (mODEWorld,0,0,0 * SCALING);
    dWorldSetCFM(mODEWorld,1e-10);
    dWorldSetERP(mODEWorld,0.6);
    dWorldSetLinearDamping(mODEWorld, mDamping);
    dWorldSetAngularDamping(mODEWorld, mDamping);





    mBTCollisionConfiguration   = new btDefaultCollisionConfiguration();
    mBTDispatcher               = new btCollisionDispatcher(mBTCollisionConfiguration);
    mBTDispatcher->setNearCallback(nearCallback);

    mBTBroadphase               = new btDbvtBroadphase();

    mBTCollisionWorld           = new btCollisionWorld(mBTDispatcher,mBTBroadphase,mBTCollisionConfiguration);

    mBTCollisionWorld->getPairCache()->setOverlapFilterCallback(&SimulatorDynamics::mCustomOverlapFilterCallback);

    mBTBroadphase->resetPool(mBTDispatcher);

    /*
    mGroundShape = new btBoxShape(btVector3(btScalar(10.) * SCALING,btScalar(10.) * SCALING,btScalar(0.5) * SCALING));
    mBTCollisionShapes.push_back(mGroundShape);

    mGroundObject = new btCollisionObject();

    btTransform groundTransform;
    groundTransform.setIdentity();
    groundTransform.setOrigin(btVector3(0,0,-0.5 * SCALING));
    mGroundObject ->getWorldTransform() = groundTransform;
    mGroundShape->setMargin(0);

    mGroundObject->setCollisionShape(mGroundShape);

    mBTCollisionWorld->addCollisionObject(mGroundObject);
    */
}

void    SimulatorDynamics::Free(){

    //BaseSimulatorDynamics::Free();


    if(mBTCollisionWorld){
        /*
        for (int i=mBTCollisionWorld->getNumCollisionObjects()-2; i>=0 ;i--){
            cerr <<"Rem "<<i<<"/"<<mBTCollisionWorld->getNumCollisionObjects()<<endl;
            btCollisionObject* obj = mBTCollisionWorld->getCollisionObjectArray()[i];
            mBTCollisionWorld->removeCollisionObject( obj );
        }
        */
    }
    if(mBTCollisionWorld)               delete mBTCollisionWorld;               mBTCollisionWorld           = NULL;
    if(mBTBroadphase)                   delete mBTBroadphase;                   mBTBroadphase               = NULL;
    if(mBTDispatcher)                   delete mBTDispatcher;                   mBTDispatcher               = NULL;
    if(mBTCollisionConfiguration)       delete mBTCollisionConfiguration;       mBTCollisionConfiguration   = NULL;

    BaseSimulatorDynamics::Free();

    /*
    if(mGroundObject) delete mGroundObject; mGroundObject = NULL;
    if(mGroundShape) delete mGroundShape; mGroundShape = NULL;
    */

    if(mODEContacts)    dJointGroupDestroy(mODEContacts);   mODEContacts = NULL;
    if(mODEWorld)       dWorldDestroy(mODEWorld);           mODEWorld    = NULL;


}



BaseDynamicObject*  SimulatorDynamics::CreateDynamicObject(){
    BaseDynamicObject *obj = new DynamicObject(this);
    return obj;
}
BaseDynamicObject*  SimulatorDynamics::CreateDynamicRobot(){
    BaseDynamicObject *obj = new DynamicRobot(this);
    return obj;
}
void SimulatorDynamics::CreateFixedLink(BaseDynamicObject* objA, BaseDynamicObject* objB){
    if(objA&&objB){
        dJointID odeJoint = dJointCreateFixed(mODEWorld,0);
        dJointAttach(odeJoint, ((DynamicObject*)objB)->mODEBody, ((DynamicObject*)objA)->mODEBody);
        dJointSetFixed(odeJoint);
    }

}

void SimulatorDynamics::UpdateCore(REALTYPE dt){
    if(mODEWorld){

        if(mBTCollisionWorld){
            mBTCollisionWorld->performDiscreteCollisionDetection();

            int numManifolds = mBTCollisionWorld->getDispatcher()->getNumManifolds();
            for (int i=0;i<numManifolds;i++){
                btPersistentManifold* contactManifold = mBTCollisionWorld->getDispatcher()->getManifoldByIndexInternal(i);
                int numContacts = contactManifold->getNumContacts();
                if(numContacts == 0) continue;

				// These changes are necessary from at least 2.82, probably 2.81 ...
                //btCollisionObject* colObjA  = static_cast<btCollisionObject*>(contactManifold->getBody0());
                const btCollisionObject* colObjA  = contactManifold->getBody0();
                //btCollisionObject* colObjB  = static_cast<btCollisionObject*>(contactManifold->getBody1());
                const btCollisionObject* colObjB  = contactManifold->getBody1();
                DynamicObject*     objA     = (DynamicObject*)colObjA->getUserPointer();
                DynamicObject*     objB     = (DynamicObject*)colObjB->getUserPointer();

                if(!objA){
                    if((objB)&&(objB->bIsKinematic)){
                        continue;
                    }
                }
                if(!objB){
                    if((objA)&&(objA->bIsKinematic)){
                        continue;
                    }
                }

                bool bBothKinematic = false;
                if((objA->bIsKinematic)&&(objB->bIsKinematic)){
                    bBothKinematic = true;
                    //cout << "CT: "<<numContacts<<endl;
                    //continue;
                }
                    //cout << "CT: "<<numContacts<<endl;


                static const int MAX_CONTACTS = 16;
                dContact contact[MAX_CONTACTS];
                int  bValidContact[MAX_CONTACTS];
                int  bSetContact[MAX_CONTACTS];
                for(int j=0;j<MAX_CONTACTS;j++){
                    bValidContact[j] = j;
                    bSetContact[j] = 0;
                }
                
                numContacts = MIN(numContacts,MAX_CONTACTS);

                /*cout << "asd "<<numContacts;
                if(objA) cout <<objA->mObject->GetName();
                if(objB) cout <<objB->mObject->GetName();
                cout << endl;*/
                int bMergePts = 0;
                if(objA&&objA->mHeightField.RowSize()>0){
                    bMergePts = 1;
                }else if(objB&&objB->mHeightField.RowSize()>0){
                    bMergePts = 2;
                }
                if(bMergePts>0){
                    if(numContacts>1)
                    for(int j1=0;j1<numContacts;j1++){
                        for(int j2=j1+1;j2<numContacts;j2++){
                            //cout <<j1<<" "<<j2<<" "<<bValidContact[j1]<<" "<<bValidContact[j2]<<endl;
                            //if( (bValidContact[j2]>=0))
                            //    continue;

                            btManifoldPoint& ptmA = contactManifold->getContactPoint(j1);
                            btManifoldPoint& ptmB = contactManifold->getContactPoint(j2);
                            btVector3 ptA         = (bMergePts==1?ptmA.getPositionWorldOnA():ptmA.getPositionWorldOnB());
                            btVector3 ptB         = (bMergePts==1?ptmB.getPositionWorldOnA():ptmB.getPositionWorldOnB());
                            double d0 = (ptA[0]-ptB[0]);
                            double d1 = (ptA[1]-ptB[1]);
                            double d2 = (ptA[2]-ptB[2]);
                            double d  = d0*d0+d1*d1+d2*d2;
                            //cout <<j1<<" "<<j2<<" "<<d<<" "<<0.01*SCALING<<endl;
                            if(d<0.01*SCALING){
                                //bValidContact[j1] = j2;
                                bValidContact[j2] = bValidContact[j1];
                            }
                        }
                    }
                }
                /*
                if(bMergePts>0){
                for (int j=0;j<numContacts;j++){
                    cout << j<<" "<<bValidContact[j]<<endl;
                }
                }*/

                for (int k=0;k<numContacts;k++){
                    int j = bValidContact[k];
                    //if(bValidContact[j]>=0) continue;

                    btManifoldPoint& pt = contactManifold->getContactPoint(k);
                    btVector3 ptA       = pt.getPositionWorldOnA();
                    btVector3 ptB       = pt.getPositionWorldOnB();
                    //cout << k<<" "<<j<<" "<<pt.m_normalWorldOnB[0] <<" "<< pt.m_normalWorldOnB[1] <<" "<< pt.m_normalWorldOnB[2] <<endl;

                    if(bSetContact[j]==0){
                        contact[j].surface.mode         =  dContactSoftCFM | dContactSoftERP;
                        contact[j].surface.soft_erp     = 0.5;
                        contact[j].surface.soft_cfm     = 1e-3;

                        if(objA && objB){
                            if((isinf(objA->mObject->GetMaterialProperty().mStaticFrictionCoef))||
                               (isinf(objB->mObject->GetMaterialProperty().mStaticFrictionCoef))){
                                contact[j].surface.mu       = dInfinity;
                            }else{
                                contact[j].surface.mu       = 0.5*(objA->mObject->GetMaterialProperty().mStaticFrictionCoef+
                                                                   objB->mObject->GetMaterialProperty().mStaticFrictionCoef);
                            }
                            contact[j].surface.bounce       = 0.5*(objA->mObject->GetMaterialProperty().mBouncyness,
                                                                  objB->mObject->GetMaterialProperty().mBouncyness);
                        }else if(objA || objB){
                            DynamicObject *obj = (objA?objA:objB);
                            contact[j].surface.bounce       = obj->mObject->GetMaterialProperty().mBouncyness;
                            contact[j].surface.mu           = obj->mObject->GetMaterialProperty().mStaticFrictionCoef;
                        }else{
                            contact[j].surface.mu           = 0;//dInfinity;
                            contact[j].surface.bounce       = 0;
                        }
                        //cout << "MU "<<contact[j].surface.mu<<endl;
                        contact[j].surface.mu2          = contact[j].surface.mu;
                        contact[j].surface.bounce_vel   = 0;
                        if(contact[j].surface.bounce > 0){
                            contact[j].surface.mode |= dContactBounce ;
                        }else{
                            //contact[j].surface.mode |= dContactBounce;
                        }

                        /*contact[j].geom.pos[0]          = 0.5*(pt.m_positionWorldOnA[0]+pt.m_positionWorldOnB[0]);
                        contact[j].geom.pos[1]          = 0.5*(pt.m_positionWorldOnA[1]+pt.m_positionWorldOnB[1]);
                        contact[j].geom.pos[2]          = 0.5*(pt.m_positionWorldOnA[2]+pt.m_positionWorldOnB[2]);*/
                        contact[j].geom.pos[0]          = pt.m_positionWorldOnA[0];
                        contact[j].geom.pos[1]          = pt.m_positionWorldOnA[1];
                        contact[j].geom.pos[2]          = pt.m_positionWorldOnA[2];
                        contact[j].geom.normal[0]       = pt.m_normalWorldOnB[0];
                        contact[j].geom.normal[1]       = pt.m_normalWorldOnB[1];
                        contact[j].geom.normal[2]       = pt.m_normalWorldOnB[2];
                        contact[j].geom.depth           = -2*pt.m_distance1;
                        contact[j].fdir1[0]             = pt.m_lateralFrictionDir1[0];
                        contact[j].fdir1[1]             = pt.m_lateralFrictionDir1[1];
                        contact[j].fdir1[2]             = pt.m_lateralFrictionDir1[2];
                        contact[j].geom.g1              = 0;
                        contact[j].geom.g2              = 0;
                    }else{
                        contact[j].geom.pos[0]          += pt.m_positionWorldOnA[0];
                        contact[j].geom.pos[1]          += pt.m_positionWorldOnA[1];
                        contact[j].geom.pos[2]          += pt.m_positionWorldOnA[2];
                        /*contact[j].geom.pos[0]          += 0.5*(pt.m_positionWorldOnA[0]+pt.m_positionWorldOnB[0]);
                        contact[j].geom.pos[1]          += 0.5*(pt.m_positionWorldOnA[1]+pt.m_positionWorldOnB[1]);
                        contact[j].geom.pos[2]          += 0.5*(pt.m_positionWorldOnA[2]+pt.m_positionWorldOnB[2]);*/
                        contact[j].geom.depth           += -2*pt.m_distance1;
                        contact[j].geom.normal[0]       += pt.m_normalWorldOnB[0];
                        contact[j].geom.normal[1]       += pt.m_normalWorldOnB[1];
                        contact[j].geom.normal[2]       += pt.m_normalWorldOnB[2];
                        contact[j].fdir1[0]             += pt.m_lateralFrictionDir1[0];
                        contact[j].fdir1[1]             += pt.m_lateralFrictionDir1[1];
                        contact[j].fdir1[2]             += pt.m_lateralFrictionDir1[2];
                    }
                    bSetContact[j]++;

                }

                for (int j=0;j<numContacts;j++){
                    if(bSetContact[j]>0){
                        if(bSetContact[j]>1){
                            REALTYPE n = 1.0/REALTYPE(bSetContact[j]);
                            contact[j].geom.depth           *= n;

                            contact[j].geom.pos[0]          *= n;
                            contact[j].geom.pos[1]          *= n;
                            contact[j].geom.pos[2]          *= n;
                            contact[j].geom.depth           *= n;
                            n = 1.0/sqrt(contact[j].geom.normal[0]*contact[j].geom.normal[0]+
                                         contact[j].geom.normal[1]*contact[j].geom.normal[1]+
                                         contact[j].geom.normal[2]*contact[j].geom.normal[2]);
                            contact[j].geom.normal[0]       *= n;
                            contact[j].geom.normal[1]       *= n;
                            contact[j].geom.normal[2]       *= n;
                            n = 1.0/sqrt(contact[j].fdir1[0]*contact[j].fdir1[0]+
                                         contact[j].fdir1[1]*contact[j].fdir1[1]+
                                         contact[j].fdir1[2]*contact[j].fdir1[2]);
                            contact[j].fdir1[0]             *= n;
                            contact[j].fdir1[1]             *= n;
                            contact[j].fdir1[2]             *= n;
                        }

                        dBodyID b0 = NULL;
                        dBodyID b1 = NULL;
                        if(objA) b0 = ((DynamicObject*)colObjA->getUserPointer())->mODEBody;
                        if(objB) b1 = ((DynamicObject*)colObjB->getUserPointer())->mODEBody;
                        if(!bBothKinematic){
                            dJointID c = dJointCreateContact(mODEWorld, mODEContacts, contact + j);
                            dJointAttach(c, b0, b1);
                        }

                        Vector3 pos(contact[j].geom.pos[0]*INVSCALING,contact[j].geom.pos[1]*INVSCALING,contact[j].geom.pos[2]*INVSCALING);
                        Vector3 normal(contact[j].geom.normal[0],contact[j].geom.normal[1],contact[j].geom.normal[2]);
                        WorldObject *wobjA = ((DynamicObject*)colObjA->getUserPointer())->mObject;                        
                        wobjA->mContacts.push_back(wobjA->GetReferenceFrame().GetInverse().GetHMatrix().Transform(pos));
                        wobjA->mContactsNormal.push_back(wobjA->GetReferenceFrame().GetInverse().GetOrient().Mult(normal));

                        WorldObject *wobjB = ((DynamicObject*)colObjB->getUserPointer())->mObject;                        
                        wobjB->mContacts.push_back(wobjB->GetReferenceFrame().GetInverse().GetHMatrix().Transform(pos));
                        wobjB->mContactsNormal.push_back(wobjB->GetReferenceFrame().GetInverse().GetOrient().Mult(normal*(-1.0)));
                        //cout << wobjB->GetName()<< " "<<wobjA->GetName()<<endl;
                        /*
                        ((DynamicObject*)colObjA->getUserPointer())->mObject->mContactsNormal.push_back(Vector3(contact[j].geom.normal[0],contact[j].geom.normal[1],contact[j].geom.normal[2]));


                        ((DynamicObject*)colObjB->getUserPointer())->mObject->mContacts.push_back(Vector3(contact[j].geom.pos[0],contact[j].geom.pos[1],contact[j].geom.pos[2]));
                        ((DynamicObject*)colObjB->getUserPointer())->mObject->mContactsNormal.push_back(Vector3(contact[j].geom.normal[0],contact[j].geom.normal[1],contact[j].geom.normal[2]));
                        */
                        //cout << ((DynamicObject*)colObjA->getUserPointer())->mObject->GetName()<<endl;
                    }
                }
                contactManifold->clearManifold();
            }
        }
        dWorldStep(mODEWorld, dt);
        dJointGroupEmpty(mODEContacts);
    }
}


void SimulatorDynamics::Reset(){
    if(mODEWorld){
        BaseSimulatorDynamics::Reset();
        dRandSetSeed(0);
    }
    if(mBTCollisionWorld){
        mBTCollisionWorld->getBroadphase()->resetPool(mBTCollisionWorld->getDispatcher());
    }

}




void SimulatorDynamics::nearCallback(btBroadphasePair& collisionPair,
                                     btCollisionDispatcher& dispatcher, const btDispatcherInfo& dispatchInfo) {

    btBroadphaseProxy* proxy0 = collisionPair.m_pProxy0;
    btBroadphaseProxy* proxy1 = collisionPair.m_pProxy1;
    if(SimulatorDynamics::mCustomOverlapFilterCallback.needBroadphaseCollision(proxy0,proxy1)){
        // Do your collision logic here
        // Only dispatch the Bullet collision information if you want the physics to continue
        dispatcher.defaultNearCallback(collisionPair, dispatcher, dispatchInfo);
    }
}

SimulatorDynamics::customOverlapFilterCallback SimulatorDynamics::mCustomOverlapFilterCallback;

bool SimulatorDynamics::customOverlapFilterCallback::needBroadphaseCollision(btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1) const{
    bool collides = (proxy0->m_collisionFilterGroup & proxy1->m_collisionFilterMask) != 0;
    collides = collides && (proxy1->m_collisionFilterGroup & proxy0->m_collisionFilterMask);

    btCollisionObject *obj0 = (btCollisionObject*)proxy0->m_clientObject;
    if(obj0!=NULL){
        btCollisionObject *obj1 = (btCollisionObject*)proxy1->m_clientObject;
        if(obj1!=NULL){
            DynamicObject *dynObj0 = (DynamicObject*)obj0->getUserPointer();
            DynamicObject *dynObj1 = (DynamicObject*)obj1->getUserPointer();

            if(dynObj0==NULL){
                if((dynObj1!=NULL)&&(dynObj1->bIsKinematic)){
                    collides = false;
                    return collides;
                }
            }
            if(dynObj1==NULL){
                if((dynObj0!=NULL)&&(dynObj0->bIsKinematic)){
                    collides = false;
                    return collides;
                }
            }
            if(dynObj0!=NULL){
                if(dynObj1!=NULL){
                    if((dynObj0->bIsKinematic)&&(dynObj1->bIsKinematic)){
                        if((!dynObj0->bNeedContact)&&(!dynObj1->bNeedContact)){
                            collides = false;
                            return collides;
                        }
                    }
                    if((dynObj0->mCollisionGroupId>0)&&(dynObj1->mCollisionGroupId>0)){
                        if(dynObj0->mCollisionGroupId == dynObj1->mCollisionGroupId){
                            //collides = false;
                            int dist = dynObj0->mCollisionGroupPartId - dynObj1->mCollisionGroupPartId;
                            if(dist<0) dist = -dist;
                            if(dist<=2){
                                collides = false;
                            }
                        }
                    }
                }
            }
        }
    }

    return collides;
}

