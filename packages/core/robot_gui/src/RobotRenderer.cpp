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

#include "RobotRenderer.h"
#include "StdTools/Various.h"

RobotRenderer::RobotRenderer(){
    mRobot          = NULL;
    mLinksCount     = 0;

    Configure(NULL);
}
RobotRenderer::~RobotRenderer(){
    Free();
}

void RobotRenderer::Free(){
    mRobot          = NULL;
    mLinksCount     = 0;

}

#if 0
void  RobotRenderer::Render(){

    AbstractRenderer::Render();
#if 0
    if(bDrawEndRef)
        GLT::DrawRef(mEndRefSize);

    int glPushPopCnt = 0;
    int currParent = 0;

    if(bDrawBones || bDrawRef || bDrawEndRef || bDrawCom){
        for(unsigned int i=0;i<mLinksCount;i++){

            bool badRef      = false;
            bool hasChildren = (mLinks->at(i)->mChildrenJoints.size()>0);
            if(mLinks->at(i)->mJoint->mParentLink!=NULL)
                badRef = (currParent != mLinks->at(i)->mJoint->mParentLink->mId);
            while(badRef){
                glPopMatrix();  glPushPopCnt--;
                currParent = mLinks->at(currParent)->mJoint->mParentLink->mId;
                badRef     = (currParent != mLinks->at(i)->mJoint->mParentLink->mId);
            }




            ReferenceFrame& ref = mLinks->at(i)->mRefFrame;

            Vector3 pos = mLinks->at(i)->mRefFrame.GetOrigin();
            if(bDrawBones){
                if(i>0){
                    if(sqrt(pos.Norm2())>0.001){
                        GLT::SetColor(mBoneColor[0],mBoneColor[1],mBoneColor[2],mBoneColor[3]);
                        GLT::DrawSegment(pos,mBoneRadius);
                    }
                }
            }
            glPushMatrix(); glPushPopCnt++;
            glMultMatrixf(ref.GetHMatrix().RowOrderForceFloat());


            if(hasChildren){
                if(bDrawRef){
                    GLT::DrawRef(mRefSize);
                }
            }else{
                if(bDrawEndRef){
                    GLT::DrawRef(mEndRefSize);
                }else if(bDrawRef){
                    GLT::DrawRef(mRefSize);
                }
            }
            if(bDrawCom){
                GLT::SetColor(mComColor[0],mComColor[1],mComColor[2],mComColor[3]);
                Matrix ine(3,3);
                Vector d(3);
                Matrix eg(3,3);
                Matrix3 egt;
                Matrix3 degt;
                Matrix  dd(3,3);
                Matrix3 ddd;
                ine = (mLinks->at(i)->mInertia.mInertiaMoment);
                ine.EigenValuesDecomposition(d, eg);
                egt.Set(eg);
                egt.STranspose();
                dd.Diag(d);
                ddd.Set(dd);

                egt.Mult(ddd,degt);

                glPushMatrix();
                    glTranslatef(mLinks->at(i)->mInertia.mCenterOfMass[0],mLinks->at(i)->mInertia.mCenterOfMass[1],mLinks->at(i)->mInertia.mCenterOfMass[2]);
                    GLT::DrawVector(degt.GetColumn(0),0.1);
                    GLT::DrawVector(degt.GetColumn(1),0.1);
                    GLT::DrawVector(degt.GetColumn(2),0.1);
                glPopMatrix();
            }
            currParent = i;
        }

        while(glPushPopCnt>0){
            glPopMatrix(); glPushPopCnt--;
        }
    }

    if(bDrawRobot){
        glPushPopCnt = 0;
        currParent = 0;
        for(unsigned int i=0;i<mLinksCount;i++){
            bool badRef = false;
            if(mLinks->at(i)->mJoint->mParentLink!=NULL)
                badRef = (currParent != mLinks->at(i)->mJoint->mParentLink->mId);
            while(badRef){
                glPopMatrix();  glPushPopCnt--;
                currParent = mLinks->at(currParent)->mJoint->mParentLink->mId;
                badRef     = (currParent != mLinks->at(i)->mJoint->mParentLink->mId);
            }

            ReferenceFrame& ref = mLinks->at(i)->mRefFrame;
            glPushMatrix(); glPushPopCnt++;
            glMultMatrixf(ref.GetHMatrix().RowOrderForceFloat());
            glColor4f(mRobotColor[0],mRobotColor[1],mRobotColor[2],mRobotColor[3]);
            glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,mRobotColor);
            if(bIsShapeValid[i]){
                mShapes[i].Render();
            }
            currParent = i;
        }
        while(glPushPopCnt>0){
            glPopMatrix(); glPushPopCnt--;
        }
    }
#endif
}

void    RobotRenderer::RenderShadow(int pass, const Vector3& light, const Vector3& observer){
    AbstractRenderer::RenderShadow(pass, light, observer);
#if 0
    int glPushPopCnt = 0;
    int currParent = 0;

    Vector3 objLight,tmpObjLight;
    objLight = light;
    tmpObjLight = objLight;

    Vector3 objObs,tmpObjObs;
    objObs = observer;
    tmpObjObs = objObs;


    if(bDrawRobot){

        //for(int pass=0;pass<2;pass++){
            //GL3DObject::RenderShadowInitPass(pass);

            glPushPopCnt = 0;
            currParent = 0;
            for(unsigned int i=0;i<mLinksCount;i++){
                bool badRef = false;
                if(mLinks->at(i)->mJoint->mParentLink!=NULL)
                    badRef = (currParent != mLinks->at(i)->mJoint->mParentLink->mId);
                while(badRef){
                    glPopMatrix();  glPushPopCnt--;
                    currParent = mLinks->at(currParent)->mJoint->mParentLink->mId;
                    badRef     = (currParent != mLinks->at(i)->mJoint->mParentLink->mId);
                }

                ReferenceFrame& ref = mLinks->at(i)->mRefFrame;
                glPushMatrix(); glPushPopCnt++;
                glMultMatrixf(ref.GetHMatrix().RowOrderForceFloat());

                if(i==0){
                    ref.GetInverse().GetHMatrix().Transform(tmpObjLight,objLight);
                    tmpObjLight = objLight;
                    ref.GetInverse().GetHMatrix().Transform(tmpObjObs,objObs);
                    tmpObjObs = objObs;
                }else{
                    mRobot->GetReferenceFrame(i,0).GetInverse().GetHMatrix().Transform(tmpObjLight,objLight);
                    mRobot->GetReferenceFrame(i,0).GetInverse().GetHMatrix().Transform(tmpObjObs,objObs);
                }

                if(bIsShapeValid[i]){
                    if(pass>=0){
                        GL3DObject::RenderShadowInitPass(pass +(objObs.Dot(objLight)>0?0:2));
                        mShapes[i].RenderShadowPass(pass,objLight);
                    }else{
                        mShapes[i].RenderOutline(objObs);
                    }
                }


                currParent = i;
            }
            while(glPushPopCnt>0){
                glPopMatrix(); glPushPopCnt--;
            }
        //}
        //GL3DObject::RenderShadowEnd();
    }

/*    glPushMatrix();

    Vector3 objLight;

    if(mObject!=NULL){
        glMultMatrixf(mObject->GetReferenceFrame().GetHMatrix().RowOrderForceFloat());
        mObject->GetReferenceFrame().GetInverse().GetHMatrix().Transform(light,objLight);
    }
    for(unsigned int i=0;i<mShapes.size();i++){
        glPushMatrix();
        //glMultMatrixf(mShapes[i]->refShape.GetHMatrix().RowOrderForceFloat());
        //mShapes[i]->refShape.GetInverse().GetHMatrix().Transform(tmp,objLight);

            if(mShapes[i]->shape){
                mShapes[i]->shape->RenderShadow(objLight);
            }
        glPopMatrix();
    }
    if(mObject!=NULL){
        glPopMatrix();
    }
*/
#endif
}
void  RobotRenderer::RenderOutline(const Vector3& observer){
    AbstractRenderer::RenderOutline(observer);
}
#endif




bool  RobotRenderer::Load(pXmlTree tree){
#if 0
    if(mRobot==NULL)    return false;

    if(tree==NULL)      return false;

    if(tree->GetName()!="ShapeList")
        return false;


    string robotName     = mRobot->GetType().c_str();
    string robotBaseName = GetPathFromFilename(robotName);

    pXmlTreeList tlist = tree->GetSubTrees();
    for(unsigned int i=0;i<tlist->size();i++){
        if(tlist->at(i)->GetName()=="Shape"){
            int id = mRobot->GetLinkIndex(tlist->at(i)->GetParamValue("id"));
            if(id>=0){
                if(id<int(mLinksCount)){
                    char txt[256];
                    bool inverse = ((tlist->at(i)->GetParamValue("inv"))=="true");

                    if(robotBaseName.length()==0){
                        sprintf(txt,"./data/Robots/%s/shapes/%s",robotName.c_str(),tlist->at(i)->GetData().c_str());
                    }else{
                        sprintf(txt,"./data/Robots/%s/shapes/%s",robotBaseName.c_str(),tlist->at(i)->GetData().c_str());
                    }
                    if(mShapes[id].LoadFromObjFile(txt,inverse)){
                        REALTYPE * array;
                        if(tlist->at(i)->Find("Orient")){
                            Matrix3 orient;
                            orient.Set(Matrix(array,tlist->at(i)->GetArray("Orient",&array)/3,3));
                            mShapes[id].Transform(orient);
                        }
                        if(tlist->at(i)->Find("Origin")){
                            Vector3 origin;
                            origin.Set(Vector(array,tlist->at(i)->GetArray("Origin",&array)));
                            mShapes[id].AddOffset(origin);
                        }
                        bIsShapeValid[id] = true;
                        pXmlTree shadowTree;
                        if((shadowTree=tlist->at(i)->Find("Shadow"))!=NULL){
                            if(robotBaseName.length()==0){
                                sprintf(txt,"./data/Robots/%s/shapes/%s",robotName.c_str(),shadowTree->GetData().c_str());
                            }else{
                                sprintf(txt,"./data/Robots/%s/shapes/%s",robotBaseName.c_str(),shadowTree->GetData().c_str());
                            }
                            GL3DObject *shadow = new GL3DObject();
                            shadow->LoadFromObjFile(txt,inverse);
                            //GL3DObject *shadow = mShapes[id].Clone();

                            if(tlist->at(i)->Find("Orient")){
                                Matrix3 orient;
                                orient.Set(Matrix(array,tlist->at(i)->GetArray("Orient",&array)/3,3));
                                shadow->Transform(orient);
                            }
                            if(tlist->at(i)->Find("Origin")){
                                Vector3 origin;
                                origin.Set(Vector(array,tlist->at(i)->GetArray("Origin",&array)));
                                shadow->AddOffset(origin);
                            }
                            mShapes[id].SetShadow(shadow);
                        }
                    }
                }
            }
        }
    }
#endif
    return true;
}


bool  RobotRenderer::LinkToRobot(Robot *robot){
    //#if 0
    gLOG.SetCurrentEntry("RobotRenderer");
    gLOG.LockCurrentEntry();
    mRobot = robot;
    if(mRobot!=NULL){
        gLOG.Append("Setting up renderer for robot: %s",mRobot->GetType().c_str());
        gLOG.SetDeltaIndent(2);

        mLinksCount     = mRobot->GetLinksCount();

        for(unsigned int i=0;i<mLinksCount;i++){
            ObjectRenderer *renderer = new ObjectRenderer();

            char shapePath[512];
            sprintf(shapePath,"Robots/%s/shapes",mRobot->GetType().c_str());
            renderer->SetBasePath(shapePath);
            renderer->LinkToObject(mRobot->GetLinks()[i]);

            AddRenderer(renderer);
        }

        /*
        char filename[256];
        XmlTree tree;
        sprintf(filename,"./data/Robots/%s/shape.xml",mRobot->GetType().c_str());
        tree.LoadFromFile(filename);

        bool res = Load(&tree);
        */
        gLOG.UnlockCurrentEntry();
        gLOG.SetDeltaIndent(-2);
        return true;
    }
    gLOG.SetDeltaIndent(-2);
    gLOG.UnlockCurrentEntry();
    return false;
    //#endif
    return true;
}

#define SET_COLOR4(array,r,g,b,a)   {(array)[0] = (r); (array)[1] = (g); (array)[2] = (b); (array)[3] = (a);}
#define COPY_COLOR4(array,src)   {(array)[0] = (src)[0]; (array)[1] = (src)[1]; (array)[2] = (src)[2]; (array)[3] = (src)[3];}
#define COPY_COLOR3(array,src)   {(array)[0] = (src)[0]; (array)[1] = (src)[1]; (array)[2] = (src)[2]; (array)[3] = 1.0;}

bool  RobotRenderer::Configure(const pXmlTree tree){

    AbstractRenderer::Configure(tree);

#if 0

    bDrawRobot      = true;
    SET_COLOR4(mRobotColor, 0.7,0.7,0.7,0.7);

    bDrawBones      = false;
    SET_COLOR4(mBoneColor, 1.0,1.0,0.0,0.7);
    mBoneRadius     = 0.004;

    bDrawCom        = false;
    SET_COLOR4(mComColor, 1.0,1.0,0.0,0.7);

    bDrawRef        = false;
    mRefSize        = 0.1;

    bDrawEndRef     = true;
    mEndRefSize     = 0.1;

    if(tree!=NULL){
        if(tree->GetName()!="Robot")
            return false;

        pXmlTree stree;
        float *array;

        mRefSize = tree->Get("RefSize",0.0f);
        cout<<"refSize"<<mRefSize<<endl;
        bDrawRef = (mRefSize>0.0);

        mEndRefSize = tree->Get("EndRefSize",0.0f);
        bDrawEndRef = (mEndRefSize>0.0);

        if((stree = tree->Find("Com"))!=NULL){
                 if(stree->GetArray("Color",&array)==3){   COPY_COLOR3(mComColor,array);}
            else if(stree->GetArray("Color",&array)==4){   COPY_COLOR4(mComColor,array);}
            else                                        {  SET_COLOR4 (mComColor,0,0,0,1);}
            bDrawCom = true;
        }else{
            bDrawCom = false;
        }

        if((stree = tree->Find("Bones"))!=NULL){
                 if(stree->GetArray("Color",&array)==3){   COPY_COLOR3(mBoneColor,array);}
            else if(stree->GetArray("Color",&array)==4){   COPY_COLOR4(mBoneColor,array);}
            else                                        {  SET_COLOR4 (mBoneColor,0,0,0,1);}
            mBoneRadius = stree->Get("Radius",0);
            bDrawBones = (mBoneRadius>0.0);
        }else{
            bDrawBones = false;
        }

        if((stree = tree->Find("Shape"))!=NULL){
                 if(stree->GetArray("Color",&array)==3){   COPY_COLOR3(mRobotColor,array);}
            else if(stree->GetArray("Color",&array)==4){   COPY_COLOR4(mRobotColor,array);}
            else                                        {  SET_COLOR4 (mRobotColor,0,0,0,1);}
            bDrawRobot = true;
        }else{
            bDrawRobot = false;
        }

    }
#endif
    return true;

}

