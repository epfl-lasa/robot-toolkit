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

#include "ObjectRenderer.h"
#include "StdTools/Various.h"

const string ObjectRenderer::mStrIntersectsShapeName = "_IntersectionShape_";

ObjectRenderer::ObjectRenderer(){

    mObject = NULL;
    Configure(NULL);
}
ObjectRenderer::~ObjectRenderer(){
    Free();
}

void ObjectRenderer::Free(){
    for(unsigned int i=0;i<mShapes.size();i++){
        if(mShapes[i]->shape) delete mShapes[i]->shape; mShapes[i]->shape=NULL;
        delete mShapes[i];
    }
    for(unsigned int i=0;i<mBBoxShapes.size();i++){
        if(mBBoxShapes[i]->shape) delete mBBoxShapes[i]->shape; mBBoxShapes[i]->shape=NULL;
        delete mBBoxShapes[i];
    }
    mShapes.clear();
}


void  ObjectRenderer::Render(){
    if(!bDrawObject) return;

    if(mObject!=NULL){
        glPushMatrix();
        glMultMatrixf(mObject->GetReferenceFrame().GetHMatrix().RowOrderForceFloat());
    }

    if(bDrawRef)
        GLT::DrawRef(mRefSize);

    for(int i=0;i<int(mShapes.size());i++){
        glPushMatrix();
        if(bDrawCom){
            GLT::SetColor(mComColor[0],mComColor[1],mComColor[2],mComColor[3]);
            Matrix ine(3,3);
            Vector d(3);
            Matrix eg(3,3);
            Matrix3 egt;
            Matrix3 degt;
            Matrix  dd(3,3);
            Matrix3 ddd;
            ine = (mObject->GetSpatialInertia().mInertiaMoment);
            ine.EigenValuesDecomposition(d, eg);
            egt.Set(eg);
            egt.STranspose();
            dd.Diag(d);
            ddd.Set(dd);

            egt.Mult(ddd,degt);

            glPushMatrix();
                Vector3 &com = mObject->GetSpatialInertia().mCenterOfMass;
                glTranslatef(com[0],com[1],com[2]);
                GLT::DrawVector(degt.GetColumn(0),0.1);
                GLT::DrawVector(degt.GetColumn(1),0.1);
                GLT::DrawVector(degt.GetColumn(2),0.1);
            glPopMatrix();
        }

        float col[4];
        //bUseDefaultColor is not needed since providing no color tag automatically sets the color to default
//        if(bUseDefaultColor){
//            col[0] = mDefaultColor[0];
//            col[1] = mDefaultColor[1];
//            col[2] = mDefaultColor[2];
//            col[3] = mDefaultColor[3];
//        }else{
            col[0] = mShapes[i]->color[0];
            col[1] = mShapes[i]->color[1];
            col[2] = mShapes[i]->color[2];
            col[3] = mShapes[i]->color[3];
//        }

        if(!bUseTransparency)
            col[3] = 1.0;


        glColor4fv(col);
        glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,col);

        if(mShapes[i]->culling)
        	glDisable(GL_CULL_FACE);


        if(mShapes[i]->shape){
            mShapes[i]->shape->Render();
        }


        if(mShapes[i]->culling)
        	glEnable(GL_CULL_FACE);

        glPopMatrix();
    }

    if(mObject!=NULL){
        glPopMatrix();
    }

    AbstractRenderer::Render();
}

void    ObjectRenderer::RenderShadow(int pass, const Vector3& light, const Vector3& observer){
    if(pass<0) return;

    glPushMatrix();

    Vector3 objLight;
    Vector3 objObserver;


    Vector3 obsObj = mObject->GetReferenceFrame().GetOrigin();
    obsObj -= observer;

    Vector3 ligObj = mObject->GetReferenceFrame().GetOrigin();
    ligObj -= light;

    glMultMatrixf(mObject->GetReferenceFrame().GetHMatrix().RowOrderForceFloat());

    mObject->GetReferenceFrame().GetInverse().GetHMatrix().Transform(light,     objLight);
    mObject->GetReferenceFrame().GetInverse().GetHMatrix().Transform(observer,  objObserver);

    for(int i=0;i<int(mShapes.size());i++){
        if(mShapes[i]->shape){
            GL3DObject::RenderShadowInitPass(pass +(obsObj.Dot(ligObj)>0?0:2));
            mShapes[i]->shape->RenderShadowPass(pass, objLight);
        }
    }

    glPopMatrix();

    AbstractRenderer::RenderShadow(pass,light,observer);
}

void  ObjectRenderer::RenderOutline(const Vector3& observer){
    glPushMatrix();

    glMultMatrixf(mObject->GetReferenceFrame().GetHMatrix().RowOrderForceFloat());

    Vector3 objObserver;
    mObject->GetReferenceFrame().GetInverse().GetHMatrix().Transform(observer,  objObserver);

    for(int i=0;i<int(mShapes.size());i++){
        if(mShapes[i]->shape){
            mShapes[i]->shape->RenderOutline(objObserver);
        }
    }
    glPopMatrix();

    AbstractRenderer::RenderOutline(observer);
}

void    ObjectRenderer::RenderBoundingBox(){
    glPushMatrix();
    glMultMatrixf(mObject->GetReferenceFrame().GetHMatrix().RowOrderForceFloat());

    glDisable(GL_LIGHTING);
    for(int i=0;i<int(mBBoxShapes.size());i++){
        glPushMatrix();

        float col[4];
        col[0] = mBBoxShapes[i]->color[0];
        col[1] = mBBoxShapes[i]->color[1];
        col[2] = mBBoxShapes[i]->color[2];
        col[3] = mBBoxShapes[i]->color[3];
        glColor4fv(col);
        glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,col);

        if(mBBoxShapes[i]->shape){
            mBBoxShapes[i]->shape->RenderWireframe();
        }

        glPopMatrix();
    }
    glEnable(GL_LIGHTING);

    glPopMatrix();
    AbstractRenderer::RenderBoundingBox();
}



bool  ObjectRenderer::Load(string name){
    char filename[256];
    XmlTree tree;
    sprintf(filename,"Worlds/Objects/%s.xml",name.c_str());
    if(FileFinder::Find(filename)){
        tree.LoadFromFile(FileFinder::GetCStr());
    }
    return Load(&tree);
}

bool  ObjectRenderer::Load(const pXmlTree tree){
    if(tree==NULL) return false;

    gLOG.SetCurrentEntry("ObjectRenderer");
    gLOG.Append("Setting up BBoxShape for object: %s",tree->GetData().c_str());
    gLOG.SetDeltaIndent(2);

    pXmlTree bbox = tree->Find("BBoxShape");
    if(bbox!=NULL){
        pXmlTreeList tmpList = bbox->GetSubTrees();
        for(unsigned int i=0;i<tmpList->size();i++){
            pShapeStruct shape = LoadShape(tmpList->at(i));
            if(shape!=NULL) mBBoxShapes.push_back(shape);
        }
    }else{
        gLOG.SetDeltaIndent(2);
        gLOG.Append("Warning: No <BBoxShape> found");
        gLOG.SetDeltaIndent(-2);
    }
    gLOG.SetDeltaIndent(-2);

    gLOG.Append("Setting up GfxShape for object: %s",tree->GetData().c_str());
    gLOG.SetDeltaIndent(2);

    pXmlTree gfx = tree->Find("GfxShape");
    if(gfx!=NULL){
        pXmlTreeList tmpList = gfx->GetSubTrees();
        for(unsigned int i=0;i<tmpList->size();i++){
            pShapeStruct shape = LoadShape(tmpList->at(i));
            if(shape!=NULL) mShapes.push_back(shape);
        }
    }else{
        gLOG.SetDeltaIndent(2);
        gLOG.Append("Warning: No <GfxShape> found");
        gLOG.SetDeltaIndent(-2);
    }
    gLOG.SetDeltaIndent(-2);
    return true;
}
ObjectRenderer::pShapeStruct    ObjectRenderer::LoadShape(const pXmlTree tree){
    if(tree==NULL) return NULL;

    REALTYPE *array;

    pXmlTree tmpTree = tree;

    pShapeStruct shape  = NULL;

    if((tmpTree->GetName()=="Shape")){
        gLOG.Append("Setting up Shape : %s",tmpTree->GetData().c_str());
        gLOG.SetDeltaIndent(2);

        int size;
        Matrix3 scale;
        scale.Identity();
        if(tmpTree->Find("Scale")){
            size=tmpTree->GetArray("Scale",&array);
            if(size==3){
                                scale.Diag(Vector3(array));
                                scale.RefNoCheck(0,0) = array[0];
                                scale.RefNoCheck(1,1) = array[1];
                                scale.RefNoCheck(2,2) = array[2];
            }else{
                                gLOG.Append("Error: Bad <Scale> array size (should be 3)");
            }
        }


        string params = tmpTree->Get("Params",string(""));
        vector<string> ptokens = Tokenize(RemoveSpaces(params));

        shape               = new ShapeStruct;
        shape->shape        = new GL3DObject();
        shape->strShapeName = tmpTree->GetData();
        shape->scale[0] = scale.At(0,0);
        shape->scale[1] = scale.At(1,1);
        shape->scale[2] = scale.At(2,2);

              if(tmpTree->GetData().length()==0){
                    gLOG.Append("Error: No shape specified");
        }else if(tmpTree->GetData() == "Cube"){
                shape->shape->GenerateCube();
        }else if(tmpTree->GetData() == "Cylinder"){
                if(ptokens.size()>=1){
                    shape->shape->GenerateCylinder(atoi(ptokens[0].c_str()));
                }else{
                    shape->shape->GenerateCylinder(16);
                }
        }else if(tmpTree->GetData() == "Sphere"){
                if(ptokens.size()>=2){
                    shape->shape->GenerateSphere(atoi(ptokens[0].c_str()),atoi(ptokens[1].c_str()));
                }else{
                    shape->shape->GenerateSphere(16,12);
                }
        }else if(tmpTree->GetData() == "Capsule"){
                if(ptokens.size()==1){
                    shape->shape->GenerateCapsule(atof(ptokens[0].c_str()),16,6);
                }else if(ptokens.size()>=3){
                    shape->shape->GenerateCapsule(atof(ptokens[0].c_str()),atoi(ptokens[1].c_str()),atoi(ptokens[2].c_str()));
                }else{
                    shape->shape->GenerateCapsule(0.5*(scale.RefNoCheck(0,0)+scale.RefNoCheck(1,1)),16,6);
                }
        }else if(tmpTree->GetData() == "HeightField"){
                if(tmpTree->Find("DataFile")){
                    string filename = tmpTree->GetBasePath()+string("/")+tmpTree->Find("DataFile")->GetData();
                    Matrix hf;
                    if(hf.Load(filename.c_str())){
                        shape->shape->GenerateHeightField(hf,1,1,1);
                    }else{
                        delete shape->shape; shape->shape=NULL;
                        gLOG.Append("Error: Height field file %s failed to open",filename.c_str());
                    }
                }else{
                    delete shape->shape; shape->shape=NULL;
                    gLOG.Append("Error: Height field: No <DataFile> specified...");
                }



                /*if(ptokens.size()==1){
                    shape->shape->GenerateCapsule(atof(ptokens[0].c_str()),16,6);
                }else if(ptokens.size()>=3){
                    shape->shape->GenerateCapsule(atof(ptokens[0].c_str()),atoi(ptokens[1].c_str()),atoi(ptokens[2].c_str()));
                }else{
                    shape->shape->GenerateCapsule(0.5*(scale.RefNoCheck(0,0)+scale.RefNoCheck(1,1)),16,6);
                }*/
        }else{
            bool bShapeFound = false;
            string shapeFile;
            if(!bShapeFound){
                shapeFile = tmpTree->GetData();
                bShapeFound = FileFinder::Find(shapeFile);
                if(bShapeFound) shapeFile = FileFinder::GetString();
            }
            if(!bShapeFound){
                shapeFile = mBasePath+"/"+tmpTree->GetData();
                bShapeFound = FileFinder::Find(shapeFile);
                if(bShapeFound) shapeFile = FileFinder::GetString();
            }
            if(!bShapeFound){
                shapeFile = tmpTree->GetBasePath()+"/"+tmpTree->GetData();
                bShapeFound = FileFinder::Find(shapeFile);
                if(bShapeFound) shapeFile = FileFinder::GetString();
            }
            if(bShapeFound){
                if(!shape->shape->LoadFromObjFile(shapeFile.c_str())){
                    delete shape->shape; shape->shape=NULL;
                    gLOG.Append("Error: Unable to load shape file: %s",shapeFile.c_str());
                }
            }else{
                gLOG.Append("Error: Unable to find shape file: %s",shapeFile.c_str());
            }
        }

        if(tmpTree->Find("Origin")){
            size=tmpTree->GetArray("Origin",&array);
            if(size==3){            shape->refShape.SetOrigin().Set(array);
            }else{                  shape->refShape.SetOrigin().Zero();
                                    gLOG.Append("Error: Bad <Origin> array size (should be 3)");
            }
        }else{
                                    shape->refShape.SetOrigin().Zero();
                                    gLOG.Append("Warning: No <Origin> defined");
        }

        if(tmpTree->Find("Orient")){
            size=tmpTree->GetArray("Orient",&array);
            if(size==9){            shape->refShape.SetOrient().Set(array);
                                    shape->refShape.SetOrient().Normalize();
            }else if(size==3){      shape->refShape.SetOrient().SRotationV(Vector3(array));
            }else{                  shape->refShape.SetOrient().Identity();
                                    gLOG.Append("Error: Bad <Orient> array size (should be 3(axis*angle) or 9(full rotation matrix))");
            }
        }else{
                                    shape->refShape.SetOrient().Identity();
                                    gLOG.Append("Warning: No <Orient> defined");
        }
        shape->refShape.Update();

        if(tmpTree->Find("Color")){
            size=tmpTree->GetArray("Color",&array);
            if(size==3){            memcpy(shape->color,array,3*sizeof(REALTYPE));
                                    shape->color[3] = 1.0;
            }else if(size==4){      memcpy(shape->color,array,4*sizeof(REALTYPE));
            }else{                  shape->color[0] = 1.0;
                                    shape->color[1] = 1.0;
                                    shape->color[2] = 1.0;
                                    shape->color[3] = 1.0;
                                    gLOG.Append("Error: Bad <Color> array size (should be 3 or 4)");
            }
        }else{
                                    shape->color[0] = 1.0;
                                    shape->color[1] = 1.0;
                                    shape->color[2] = 1.0;
                                    shape->color[3] = 1.0;
                                    gLOG.Append("Warning: No <Color> defined");
        }
        if(tmpTree->Find("DoubleFace")){
        	shape->culling = tmpTree->Get("DoubleFace",false);
        }
        else
        {
        	shape->culling = false;
        }

        if(shape->shape!=NULL){
            shape->shape->Transform(scale);
            shape->shape->Transform(shape->refShape.GetHMatrix());

            /*
            pXmlTree tmpShadowTree;
            if((tmpShadowTree=tmpTree->Find("Shadow"))!=NULL){
                string params = tmpShadowTree->Get("Params",string(""));
                vector<string> ptokens = Tokenize(RemoveSpaces(params));

                GL3DObject *shadow = new GL3DObject();
                      if(tmpShadowTree->GetData() == "Cube"){
                        shadow->GenerateCube();
                }else if(tmpShadowTree->GetData() == "Cylinder"){
                        if(ptokens.size()>=1){
                            shadow->GenerateCylinder(atoi(ptokens[0].c_str()));
                        }else{
                            shadow->GenerateCylinder(8);
                        }
                }else if(tmpShadowTree->GetData() == "Sphere"){
                        if(ptokens.size()>=2){
                            shadow->GenerateSphere(atoi(ptokens[0].c_str()),atoi(ptokens[1].c_str()));
                        }else{
                            shadow->GenerateSphere(4,4);
                        }
                }else if(tmpShadowTree->GetData() == "Clone"){
                        delete shadow;
                        shadow=shape->shape->Clone();
                }else{
                    if(!shadow->LoadFromObjFile(tmpShadowTree->GetData().c_str(),true)){
                        delete shadow; shadow=NULL;
                    }
                }
                if(shadow!=NULL){
                    shadow->Transform(shape->refShape.GetHMatrix());

                    if(tmpShadowTree->Find("Origin")){
                        Vector3 origin;
                        origin.Set(Vector(array,tmpShadowTree->GetArray("Origin",&array)));
                        shadow->AddOffset(origin);
                    }
                    if(tmpShadowTree->Find("Orient")){
                        Matrix3 orient;
                        orient.Set(Matrix(array,tmpShadowTree->GetArray("Orient",&array)/3,3));
                        shadow->Transform(orient);
                    }
                    shape->shape->SetShadow(shadow);
                }
            }
            */
        }
        gLOG.SetDeltaIndent(-2);

    }


    return shape;
}

void  ObjectRenderer::AddShapeFromRay(unsigned uShapeIndex, Vector3 vecRay1, Vector3 vecRay2, double dFactor, double dSphereScale){
    //std::cout << "Adding Sphere on the object's surface. Base point is " << vecRay1.x() << ", " << vecRay1.y() << ", " << vecRay1.z() << ". Vector is " << vecRay2.x() << ", " << vecRay2.y() << ", " << vecRay2.z() << ". Factor is " << dFactor << std::endl;

    // Compute intersection point
    Vector3 vecIntersection = vecRay1 + (vecRay2 - vecRay1)*dFactor;

    //std::cout << " Coordinates: " << vecIntersection.x() << ", " << vecIntersection.y() << ", " << vecIntersection.z() << std::endl;

    // Convert from absolute coordinates to coordinates respective to the object's referential
    Matrix matTmp = mObject->GetReferenceFrame().GetInverse().GetHMatrix();
    double pIntersectionH[4] = { vecIntersection.x(), vecIntersection.y(), vecIntersection.z(), 1.0 };
    Vector vecIntersectionH(pIntersectionH, 4);
    Vector vecTmp = matTmp.Mult(vecIntersectionH);
    vecIntersection.Set(vecTmp);

    //std::cout << "Transformed Coordinates: " << vecIntersection.x() << ", " << vecIntersection.y() << ", " << vecIntersection.z() << std::endl;

    // Create sphere shape
    pShapeStruct pNewShape  = new ShapeStruct;
    pNewShape->strShapeName = mStrIntersectsShapeName;
    pNewShape->color[0]     = 1.0;
    pNewShape->color[1]     = 0.0;
    pNewShape->color[2]     = 0.0;
    pNewShape->color[3]     = 1.0;
    pNewShape->refShape.SetOrigin(vecIntersection);

    double dScale = 0.02;
    double pScaleData[3] = { dScale, dScale, dScale };
    Matrix3 scale;
    scale.Diag(Vector3(pScaleData));

    pNewShape->shape        = new GL3DObject();
    pNewShape->shape->GenerateSphere(16,16);
    pNewShape->shape->Transform(scale);
    pNewShape->shape->Transform(pNewShape->refShape.GetHMatrix());

    mShapes.push_back(pNewShape);


    if(mObject){
        pXmlTree conf = mObject->GetConfigTree();
        pXmlTree ptList = conf->Find("PointList");
        if(ptList==NULL){
            ptList = new XmlTree("PointList");
            conf->AddSubTree(ptList);
        }

        // Apply scale transform if any
        if(mShapes[uShapeIndex]->scale[0]!=1.0 || mShapes[uShapeIndex]->scale[1]!=1.0 || mShapes[uShapeIndex]->scale[2]!=1.0){
            Matrix3 matScale;
            matScale.Diag(Vector3(mShapes[uShapeIndex]->scale[0]));
            matScale.RefNoCheck(0,0) = 1/mShapes[uShapeIndex]->scale[0];
            matScale.RefNoCheck(1,1) = 1/mShapes[uShapeIndex]->scale[1];
            matScale.RefNoCheck(2,2) = 1/mShapes[uShapeIndex]->scale[2];

            Vector3 vecTmp = matScale.Mult(vecIntersection);
            vecIntersection = vecTmp;

            //std::cout << "Scaled Coordinates: " << vecIntersection.x() << ", " << vecIntersection.y() << ", " << vecIntersection.z() << std::endl;
        }

        char txt[256];
        sprintf(txt,"%f %f %f",vecIntersection.x(),vecIntersection.y(),vecIntersection.z());
        ptList->AddSubTree(new XmlTree("Point",txt));
        //conf->Print();
    }
}

void  ObjectRenderer::RemoveShape(unsigned uShapeIndex){
    if(uShapeIndex >= mShapes.size())
        return;

    unsigned int shapeSize = mShapes.size();

    // Not very clean because doing assumption on iterator impl...
    vector<pShapeStruct>::iterator it=mShapes.begin();
    mShapes.erase(mShapes.begin()+uShapeIndex);

    if(mObject){
        pXmlTree conf = mObject->GetConfigTree();
        pXmlTree ptList = conf->Find("PointList");
        if(ptList==NULL){
            ptList = new XmlTree("PointList");
            conf->AddSubTree(ptList);
        }
        pXmlTreeList tlist = ptList->GetSubTrees();

        int id = int(uShapeIndex) - (int(shapeSize) - int(tlist->size()));
        if((id>=0)&&(id<int(tlist->size()))){
            ptList->DelSubTree((*tlist)[id]);
        }
        //cout << "DELETED"<<endl;
        //conf->Print();
    }

}

bool  ObjectRenderer::ComputeRayIntersection(Vector3 vecRay1, Vector3 vecRay2, double & dFactor, bool bIntersections, unsigned* pShapeIndex){
    dFactor                  = R_INFINITY;
    bool bFound              = false;
    double dTmpFactor;

    for(unsigned i=0;i<mShapes.size();i++){
        if( (mShapes[i]->strShapeName == mStrIntersectsShapeName) == bIntersections ){
            //std::cout << "Shape name: " << mShapes[i]->strShapeName << ", looking for intersections? " << ( bIntersections ? "true" : "false" ) << std::endl;
            bool bFoundTmp = mShapes[i]->shape->ComputeRayIntersection(vecRay1, vecRay2, mObject->GetReferenceFrame().GetHMatrix(), dTmpFactor);
            if(bFoundTmp && dFactor > dTmpFactor){
                //std::cout << "Found intersection with shape name: " << mShapes[i]->strShapeName << ", factor " << dTmpFactor << std::endl;
                dFactor = dTmpFactor;
                if(pShapeIndex)
                    *pShapeIndex = i;
                bFound = true;
            }
        }
    }

    return bFound;
}

bool  ObjectRenderer::LinkToObject(WorldObject *object){
    mObject = object;
    Load(mObject->GetConfigTree());
    return true;
}

void    ObjectRenderer::SetBasePath(string path){
    mBasePath = path;
}

#define SET_COLOR4(array,r,g,b,a)   {(array)[0] = (r); (array)[1] = (g); (array)[2] = (b); (array)[3] = (a);}
#define COPY_COLOR4(array,src)   {(array)[0] = (src)[0]; (array)[1] = (src)[1]; (array)[2] = (src)[2]; (array)[3] = (src)[3];}
#define COPY_COLOR3(array,src)   {(array)[0] = (src)[0]; (array)[1] = (src)[1]; (array)[2] = (src)[2]; (array)[3] = 1.0;}

bool    ObjectRenderer::Configure(const pXmlTree tree){

    bDrawObject         = true;
    bUseDefaultColor    = false;
    bUseTransparency    = false;
    SET_COLOR4(mDefaultColor, 0.7,0.7,0.7,1.0);

    bDrawCom            = false;
    SET_COLOR4(mComColor, 1.0,1.0,0.0,1.0);

    bDrawRef            = true;
    mRefSize 			= 0.04;

    if(tree!=NULL){
        pXmlTree stree;
        float *array;

        mRefSize = tree->Get("RefSize",0.0f);
        bDrawRef = (mRefSize>0.0001);

        if((stree = tree->Find("Com"))!=NULL){
                 if(stree->GetArray("Color",&array)==3){   COPY_COLOR3(mComColor,array);}
            else if(stree->GetArray("Color",&array)==4){   COPY_COLOR4(mComColor,array);}
            else                                        {  SET_COLOR4 (mComColor,1,1,0,1);}
            bDrawCom = true;
        }else{
            bDrawCom = false;
        }

        if((stree = tree->Find("Color"))!=NULL){
                 if(tree->GetArray("Color",&array)==3){   COPY_COLOR3(mDefaultColor,array);}
            else if(tree->GetArray("Color",&array)==4){   COPY_COLOR4(mDefaultColor,array);}
            else                                       {  SET_COLOR4 (mDefaultColor,0.7,0.7,0.7,1);}
            bUseDefaultColor = true;
        }else{
            bUseDefaultColor = false;
        }
        bUseTransparency = tree->Get("CurrTransparency",false);

    }
    return true;
}


