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

#include "Link.h"


Link::Link(){
  mJoint  = NULL;
  mSpFrame.Set(mRefFrame);
  Free();
}

Link::~Link(){
  Free();
}

void Link::Free(){
  mId     = 0;
  mName   = "";

  if(mJoint!=NULL) delete mJoint;
  mJoint  = NULL;
  mChildrenJoints.clear();

  mInertia.Zero();
//  mCenterOfMass.Zero();

  mRefFrame.Identity();
}

bool Link::operator == (const Link & link){
  return mName==link.mName;
}

bool Link::Load(const pXmlTree tree){

  if(tree==NULL) return false;
  if(tree->GetName()!="Link") return false;

    WorldObject::Load(tree);

  //REALTYPE *array;
  /*
    mName = tree->GetData();
    tree->GetArray("CenterOfMass",&array);
    mCenterOfMass.Set(array);
    tree->GetArray("InertiaMatrix",&array);
    mInertia.mInertiaMoment.Set(array);

//#ifdef WAM_BADREF_CORRECTION
  if(tree->Find("RotorInertia")){
    Matrix dd(3,3);
    Matrix3 dd3;
    Vector d;
    d = Vector(array,tree->GetArray("RotorInertia",&array));
    dd.Diag(d);
    dd3.Set(dd.Array());
    mInertia.mInertiaMoment += dd3;
  }
//#endif

  mInertia.mMass = tree->Get("Mass",0.0);;
  mCenterOfMass.Mult(mInertia.mMass,mInertia.mLinearMoment);
  mInertia.mCenterOfMass = mCenterOfMass;
    */
  mJoint = JointConstructor::Create(tree->Find("Joint"));
  if(mJoint!=NULL)
    mJoint->SetChildLink(this);
  return true;
}

void Link::AddChildJoint(pJoint child){
  if(child!=NULL){
    mChildrenJoints.push_back(child);
  }
}

void Link::Update(){
  if(mJoint!=NULL){
    mJoint->Update();
    if(mJoint->mParentLink!=NULL)
        mJoint->mParentLink->mWorldReferenceFrame.Mult(mRefFrame,mWorldReferenceFrame);
  }
}

pJoint Link::GetJoint(){
  return mJoint;
}

void      Link::SetIndex(int index){
  mId = index;
}
/*
Vector3&      Link::GetFirstMoment(Vector3 &result, bool bRecursive, REALTYPE *cumMass){
    if((!bRecursive)||(mChildrenJoints.size()==0)){
        result = mCenterOfMass;
        result*= mInertia.mMass;
        if(cumMass!=NULL) *cumMass = mInertia.mMass;
    }else{
        result.Zero();
        REALTYPE totalMass = 0.0;
        Vector3 tmpV,tmpV2;
        REALTYPE tmpM;

        for(unsigned int i=0;i<mChildrenJoints.size();i++){
            if(mChildrenJoints[i]->mChildLink!=NULL){
                mChildrenJoints[i]->mChildLink->GetFirstMoment(tmpV,true,&tmpM);
                if(tmpM>0.0){
                    mChildrenJoints[i]->mChildLink->mRefFrame.GetHMatrix().Transform(tmpV,tmpV2);
                    totalMass += tmpM;
                    result+=tmpV2;
                }
                //tmpV2.ScaleAddTo(tmpM,result);
            }
        }

        totalMass += mInertia.mMass;
        mCenterOfMass.ScaleAddTo(mInertia.mMass,result);

        if(cumMass!=NULL) *cumMass = totalMass;
    }
    return result;
}
*/
Vector3&      Link::GetCenterOfMass(Vector3 &result, bool bRecursive, REALTYPE *cumMass){
    /*
    if((!bRecursive)||(mChildrenJoints.size()==0)){
        result = mCenterOfMass;
        if(cumMass!=NULL) *cumMass = mInertia.mMass;
    }else{
        REALTYPE totalMass = 0.0;
        GetFirstMoment(result,true,&totalMass);
        if(totalMass<=0.0){
            result.Zero();
        }else{
            result /= totalMass;
        }
        if(cumMass!=NULL) *cumMass = totalMass;
    }

    return result;
    */
    if((!bRecursive)||(mChildrenJoints.size()==0)){
        result = mInertia.mCenterOfMass;
        if(cumMass!=NULL) *cumMass = mInertia.mMass;
    }else{
        result.Zero();
        REALTYPE totalMass = 0.0;
        Vector3 tmpV,tmpV2;
        REALTYPE tmpM;

        for(unsigned int i=0;i<mChildrenJoints.size();i++){
            if(mChildrenJoints[i]->mChildLink!=NULL){
                mChildrenJoints[i]->mChildLink->GetCenterOfMass(tmpV,true,&tmpM);
                if(tmpM>0.0){
                    mChildrenJoints[i]->mChildLink->mRefFrame.GetHMatrix().Transform(tmpV,tmpV2);
                    totalMass += tmpM;
                    tmpV2.ScaleAddTo(tmpM,result);
                }
            }
        }

        totalMass += mInertia.mMass;
        mInertia.mCenterOfMass.ScaleAddTo(mInertia.mMass,result);

        if(totalMass<=0.0){
            result.Zero();
        }else{
            result /= totalMass;
        }

        if(cumMass!=NULL) *cumMass = totalMass;
    }
    return result;
}

REALTYPE      Link::GetMass(){
    REALTYPE result = mInertia.mMass;
    for(unsigned int i=0;i<mChildrenJoints.size();i++){
        if(mChildrenJoints[i]->mChildLink!=NULL)
            result += mChildrenJoints[i]->mChildLink->GetMass();
    }
    return result;
}






void      Link::Print(){
  cout <<"Link "<<mId<<": "<<mName<<endl;
  //cout <<"Inertia:"<<mInertia.mMass<<endl;
  //mInertia.mLinearMoment.Print();
  //mInertia.mInertiaMoment.Print();
  //mJoint->Print();
  //cout <<"******"<<endl;
  for(unsigned int i=0;i<mChildrenJoints.size();i++){
    cout << mChildrenJoints[i]->mChildLink->mName << endl;
  }
  cout <<"******"<<endl;

}


LinkTree::LinkTree():TTree<Link>(){}
LinkTree::LinkTree(const LinkTree & tree):TTree<Link>(tree){}
LinkTree::~LinkTree(){}

LinkTree* LinkTree::Load(const pXmlTree tree, int *currIndexPtr){

  Clear();

  int currIndex = 0;
  if(currIndexPtr!=NULL){
    currIndex = *currIndexPtr;
  }

  if(tree!=NULL){
    if(tree->GetName()=="Link"){
      mData->Load(tree);
      mData->SetIndex(currIndex);
      currIndex++;
    }

    pXmlTree tmp = tree->Find("Children");
    if(tmp!=NULL){

      pXmlTreeList tlist = tmp->GetSubTrees();
      for(unsigned int i=0;i<tlist->size();i++){
        if(tlist->at(i)->GetName()=="Link"){
          //cout << tlist->at(i)->GetData()<<endl;
          pLinkTree linkTree = (new LinkTree())->Load(tlist->at(i),&currIndex);
          if(linkTree->mData->mJoint!=NULL){
            linkTree->mData->mJoint->SetParentLink(mData);
            mData->AddChildJoint(linkTree->mData->mJoint);
          }
          AddSubTree(linkTree);
        }
      }
    }
  }

  if(currIndexPtr!=NULL){
    *currIndexPtr = currIndex;
  }

  return this;
}

void LinkTree::Update(){
  mData->Update();
  for(unsigned int i=0;i<mSubTrees.size();i++){
    ((LinkTree*)mSubTrees[i])->Update();
  }
}
void LinkTree::SetInitialState(){
    Update();
    mData->SetInitialState();
    for(unsigned int i=0;i<mSubTrees.size();i++){
        ((LinkTree*)mSubTrees[i])->SetInitialState();
    }
}

void LinkTree::Print(){
  mData->Print();
  for(unsigned int i=0;i<mSubTrees.size();i++){
    cout<<"Childrens"<<endl;
    ((LinkTree*)mSubTrees[i])->Print();
  }
}


