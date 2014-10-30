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

#include "ForwardKinematics.h"


ForwardKinematics::ForwardKinematics(){
    mRobot                          = NULL;
    mLinks                          = NULL;
    mJoints                         = NULL;  
    mParents                        = NULL;
    mLinksCount                     = 0;

    mFullKinematicRefFrames         = NULL;
    bFullKinematicRefFramesValid    = NULL;
    bFullKinematicRefFramesCreated  = NULL;  

    mWorldKinematicRefFrames         = NULL;
    bWorldKinematicRefFramesValid    = NULL;
    bWorldKinematicRefFramesCreated  = NULL;  
}


ForwardKinematics::~ForwardKinematics(){
    Free();
}

void ForwardKinematics::Free(){
    SetRobot(NULL);
}

void ForwardKinematics::SetRobot(pRobot robot){
    //if(mRobot==robot) return;

    if(mFullKinematicRefFrames!=NULL){
        for(unsigned int i=0;i<mLinksCount*mLinksCount;i++){
            if(bFullKinematicRefFramesCreated[i])
                delete mFullKinematicRefFrames[i];
        }
    }

    if(mFullKinematicRefFrames!=NULL)        delete [] mFullKinematicRefFrames;        mFullKinematicRefFrames        = NULL;
    if(bFullKinematicRefFramesValid!=NULL)   delete [] bFullKinematicRefFramesValid;   bFullKinematicRefFramesValid   = NULL;
    if(bFullKinematicRefFramesCreated!=NULL) delete [] bFullKinematicRefFramesCreated; bFullKinematicRefFramesCreated = NULL;

    if(mWorldKinematicRefFrames!=NULL){
        for(unsigned int i=0;i<mLinksCount;i++){
            if(bWorldKinematicRefFramesCreated[i])
                delete mWorldKinematicRefFrames[i];
        }
    }
    if(mWorldKinematicRefFrames!=NULL)        delete [] mWorldKinematicRefFrames;        mWorldKinematicRefFrames        = NULL;
    if(bWorldKinematicRefFramesValid!=NULL)   delete [] bWorldKinematicRefFramesValid;   bWorldKinematicRefFramesValid   = NULL;
    if(bWorldKinematicRefFramesCreated!=NULL) delete [] bWorldKinematicRefFramesCreated; bWorldKinematicRefFramesCreated = NULL;


    mRobot = robot;
    if(mRobot!=NULL){
        mLinksCount     =  mRobot->GetLinksCount();
        mLinks          = &mRobot->GetLinks();
        mJoints         = &mRobot->GetJoints();
        mParents        = &mRobot->GetParents();
    }else{
        mLinksCount     = 0;
        mLinks          = NULL;
        mJoints         = NULL;    
        mParents        = NULL;    
    }
  
  
    if(mLinksCount>0){
        mFullKinematicRefFrames             = new pReferenceFrame [mLinksCount*mLinksCount];
        bFullKinematicRefFramesValid        = new bool            [mLinksCount*mLinksCount];
        bFullKinematicRefFramesCreated      = new bool            [mLinksCount*mLinksCount];
        memset(mFullKinematicRefFrames,        0, mLinksCount*mLinksCount*sizeof(pReferenceFrame));
        memset(bFullKinematicRefFramesValid,   0, mLinksCount*mLinksCount*sizeof(bool));
        memset(bFullKinematicRefFramesCreated, 0, mLinksCount*mLinksCount*sizeof(bool));
        
        mWorldKinematicRefFrames        = new pReferenceFrame [mLinksCount];
        bWorldKinematicRefFramesValid   = new bool            [mLinksCount];
        bWorldKinematicRefFramesCreated = new bool            [mLinksCount];
        memset(mWorldKinematicRefFrames,        0, mLinksCount*sizeof(pReferenceFrame));
        memset(bWorldKinematicRefFramesValid,   0, mLinksCount*sizeof(bool));
        memset(bWorldKinematicRefFramesCreated, 0, mLinksCount*sizeof(bool));
               
        BuildFullKinematicRefFrames();
    }
}




ReferenceFrame& ForwardKinematics::GetReferenceFrame(unsigned int link){
    // Check for valid inputs
    

    if(link>=mLinksCount){
        return *mWorldKinematicRefFrames[0];    
    }
    if(bWorldKinematicRefFramesValid[link])
        return *mWorldKinematicRefFrames[link];    

    mWorldKinematicRefFrames[0]->Mult(GetReferenceFrame(link,0),*mWorldKinematicRefFrames[link]);
    bWorldKinematicRefFramesValid[link] = true;
    return *mWorldKinematicRefFrames[link];

}

ReferenceFrame& ForwardKinematics::GetReferenceFrame(unsigned int fromLink, unsigned int toLink){
  
    // Check for valid inputs
    if((fromLink>=mLinksCount)||(toLink>=mLinksCount)){
        return *mFullKinematicRefFrames[0];    
    }
  
    // Check for already valid transformation
    unsigned int resultIndex = toLink*mLinksCount + fromLink;         
    if(bFullKinematicRefFramesValid[resultIndex])
        return *mFullKinematicRefFrames[resultIndex];

    // Use inverse computations for from < to
    if(fromLink<toLink){
        return GetReferenceFrame(toLink,fromLink).GetInverse();
    }
    
    // Recursive backward transform computations
    unsigned int  currLink    = fromLink;
    int           parentLink  = mParents->at(currLink);
    int           currIndex = 0;
  
    pReferenceFrame baseRef = NULL;
  
    while((currLink!=toLink)&&(parentLink>=0)){

        currIndex      = parentLink*mLinksCount + fromLink;
        int bkwIndex   = fromLink*mLinksCount + parentLink;
    
        int transIndex = parentLink*mLinksCount + currLink;

        if(bFullKinematicRefFramesValid[currIndex]){
            // Should be always valid upon entering the loop      
            baseRef = mFullKinematicRefFrames[currIndex];
        }else{
            mFullKinematicRefFrames[transIndex]->Mult(*baseRef,*mFullKinematicRefFrames[currIndex]);
            bFullKinematicRefFramesValid[currIndex] = true;
            bFullKinematicRefFramesValid[bkwIndex]  = true;

            baseRef = mFullKinematicRefFrames[currIndex];
        }

        currLink   = parentLink;
        parentLink = mParents->at(currLink);    
    }

    // Return if success
    if(currLink==toLink)
        return *mFullKinematicRefFrames[resultIndex];

    // If not, get and/or compute the reverse transformation
    int invIndex = toLink;
    if(!bFullKinematicRefFramesValid[invIndex])
        GetReferenceFrame(toLink, 0);
    baseRef = &mFullKinematicRefFrames[invIndex]->GetInverse();  

    // Update the selected reference frame
    baseRef->Mult(*mFullKinematicRefFrames[currIndex],*mFullKinematicRefFrames[resultIndex]);
    bFullKinematicRefFramesValid[resultIndex] = true;
    unsigned int invResultIndex = fromLink*mLinksCount+toLink;         
    bFullKinematicRefFramesValid[invResultIndex] = true;

    // return the result
    return *mFullKinematicRefFrames[resultIndex];
}




void ForwardKinematics::Update(){
    ResetFullKinematicRefFrames();
}

void ForwardKinematics::BuildFullKinematicRefFrames(){
    // Set the identity diagonal
    for(unsigned int i=0;i<mLinksCount;i++){
        unsigned int index = i*mLinksCount+i;
        mFullKinematicRefFrames[index]        = new ReferenceFrame();
        bFullKinematicRefFramesCreated[index] = true;         
    }
  
    // from j to i (frame j represented in frame i)
    for(unsigned int i=0;i<mLinksCount;i++){
        for(unsigned int j=i+1;j<mLinksCount;j++){
            //unsigned int parent = mLinks->at(j)->mJoint->mParentLink->mId; 
            unsigned int parent = mJoints->at(j)->mParentLink->mId;

            unsigned int fwdIndex = i*mLinksCount+j;
            unsigned int bkwIndex = j*mLinksCount+i;

            if(i==parent){
                mFullKinematicRefFrames[fwdIndex]        = &mLinks->at(j)->mRefFrame;
                bFullKinematicRefFramesCreated[fwdIndex] = false;
                     
                mFullKinematicRefFrames[bkwIndex]        = &mLinks->at(j)->mRefFrame.GetInverse();
                bFullKinematicRefFramesCreated[bkwIndex] = false;
            }else{
                mFullKinematicRefFrames[fwdIndex]        = new ReferenceFrame();
                bFullKinematicRefFramesCreated[fwdIndex] = true;         
            
                mFullKinematicRefFrames[bkwIndex]        = &mFullKinematicRefFrames[fwdIndex]->GetInverse();
                bFullKinematicRefFramesCreated[bkwIndex] = false;         
            }        
        }        
    }

    if(mRobot->GetWorldInstance()==NULL){
        mWorldKinematicRefFrames[0] = new ReferenceFrame();
        bWorldKinematicRefFramesCreated[0] = true;   
    }else{
        mWorldKinematicRefFrames[0] = &(mRobot->GetWorldInstance()->GetReferenceFrame());
        bWorldKinematicRefFramesCreated[0] = false;   
    }
    
    for(unsigned int i=1;i<mLinksCount;i++){
        mWorldKinematicRefFrames[i] = new ReferenceFrame();
        bWorldKinematicRefFramesCreated[i] = true;
    }
    
    
    
    ResetFullKinematicRefFrames();
}

void ForwardKinematics::ResetFullKinematicRefFrames(){
    memset(bFullKinematicRefFramesValid, 0, mLinksCount*mLinksCount*sizeof(bool));
  
    bFullKinematicRefFramesValid[0] = true;         

    for(unsigned int i=1;i<mLinksCount;i++){
        unsigned int parent = mJoints->at(i)->mParentLink->mId; 
        unsigned int fwdIndex = i*mLinksCount+parent;
        unsigned int bkwIndex = parent*mLinksCount+i;
        unsigned int idIndex  = i*mLinksCount+i;
        bFullKinematicRefFramesValid[fwdIndex] = true;         
        bFullKinematicRefFramesValid[bkwIndex] = true;         
        bFullKinematicRefFramesValid[idIndex]  = true;         
    }  

    memset(bWorldKinematicRefFramesValid, 0, mLinksCount * sizeof(bool));
    bWorldKinematicRefFramesValid[0] = true;
    
}
