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

#include "Base3DObject.h"

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include "StdTools/Various.h"
#include "StdTools/LogStream.h"
using namespace std;


#define BASE3DOBJ_FILE_DEF_MAX_TRIANGLES   32768
#define BASE3DOBJ_FILE_DEF_MAX_VERTICES    32768

REALTYPE *Base3DObject::sVerData   = NULL;
int      *Base3DObject::sTriData   = NULL;
int      *Base3DObject::sPolySize  = NULL;
int      *Base3DObject::sPolyStart = NULL;
int       Base3DObject::sObjCount  = 0;
int       Base3DObject::sBASE3DOBJ_FILE_MAX_TRIANGLES  = BASE3DOBJ_FILE_DEF_MAX_TRIANGLES;
int       Base3DObject::sBASE3DOBJ_FILE_MAX_VERTICES   = BASE3DOBJ_FILE_DEF_MAX_VERTICES;

void Base3DObject::ResizeStaticVar(){
    if(sVerData==NULL){
        sVerData   = new REALTYPE[sBASE3DOBJ_FILE_MAX_VERTICES*3];
        sTriData   = new      int[sBASE3DOBJ_FILE_MAX_TRIANGLES*3];
        sPolySize  = new      int[sBASE3DOBJ_FILE_MAX_TRIANGLES];
        sPolyStart = new      int[sBASE3DOBJ_FILE_MAX_TRIANGLES];
    }else{
        void * tmp;
        //cout << "Resizing to: "<<2*sBASE3DOBJ_FILE_MAX_VERTICES<<" "<< 2*sBASE3DOBJ_FILE_MAX_TRIANGLES<<endl;
        tmp         = new REALTYPE[2*sBASE3DOBJ_FILE_MAX_VERTICES*3];
        memcpy(tmp,sVerData,sizeof(REALTYPE)*sBASE3DOBJ_FILE_MAX_VERTICES*3);
        delete sVerData; sVerData = (REALTYPE*)tmp;

        tmp         = new int[2*sBASE3DOBJ_FILE_MAX_TRIANGLES*3];
        memcpy(tmp,sTriData,sizeof(int)*sBASE3DOBJ_FILE_MAX_TRIANGLES*3);
        delete sTriData; sTriData = (int*)tmp;

        tmp         = new int[2*sBASE3DOBJ_FILE_MAX_TRIANGLES];
        memcpy(tmp,sPolySize,sizeof(int)*sBASE3DOBJ_FILE_MAX_TRIANGLES);
        delete sPolySize; sPolySize = (int*)tmp;

        tmp         = new int[2*sBASE3DOBJ_FILE_MAX_TRIANGLES];
        memcpy(tmp,sPolyStart,sizeof(int)*sBASE3DOBJ_FILE_MAX_TRIANGLES);
        delete sPolyStart; sPolyStart = (int*)tmp;

        sBASE3DOBJ_FILE_MAX_TRIANGLES    *= 2;
        sBASE3DOBJ_FILE_MAX_VERTICES     *= 2;
    }
}

Base3DObject::Base3DObject(){

    sObjCount++;

    if(sVerData==NULL){
        ResizeStaticVar();
        //ResizeStaticVar();
    }

    mNbVertices     = 0;
    mNbPolygons     = 0;
    mPolygons       = NULL;
    mPolygonSize    = NULL;
    mPolygonStart   = NULL;
    mEdgeNeighbours     = NULL;
    mVertexNeighbours       = NULL;
    mVertexNeighbourSize   = NULL;
    mVertexNeighbourStart   = NULL;
    mIsVisible      = NULL;
    bOnlyTriangles  = false;

    mVertices.Resize(0,0);
    mNormals.Resize(0,0);
}

Base3DObject::~Base3DObject(){
    Free();

    sObjCount--;
    if(sObjCount<=0){
        delete [] sVerData;
        delete [] sTriData;
        delete [] sPolySize;
        delete [] sPolyStart;
        sVerData    = NULL;
        sTriData    = NULL;
        sPolySize   = NULL;
        sPolyStart   = NULL;
        sObjCount   = 0;
    }
}

void Base3DObject::Free(){
    if(mPolygons!=NULL) delete [] mPolygons;
    mPolygons = NULL;
    if(mPolygonSize!=NULL) delete [] mPolygonSize;
    mPolygonSize = NULL;
    if(mPolygonStart!=NULL) delete [] mPolygonStart;
    mPolygonStart = NULL;
    if(mEdgeNeighbours!=NULL) delete [] mEdgeNeighbours;
    mEdgeNeighbours     = NULL;
    if(mVertexNeighbours!=NULL) delete [] mVertexNeighbours;
    mVertexNeighbours       = NULL;
    if(mVertexNeighbourSize!=NULL) delete [] mVertexNeighbourSize;
    mVertexNeighbourSize   = NULL;
    if(mVertexNeighbourStart!=NULL) delete [] mVertexNeighbourStart;
    mVertexNeighbourStart = NULL;
    if(mIsVisible!=NULL) delete [] mIsVisible;
    mIsVisible      = NULL;
    mVertices.Resize(0,0);
    mNormals.Resize(0,0);
    mNbVertices  = 0;
    mNbPolygons = 0;
    bOnlyTriangles = false;
}

bool  Base3DObject::SaveToObjFile(const char *filename){
    ofstream ofile;
    ofile.open(filename);
    if(ofile.is_open()){
        for(int i=0;i<mNbVertices;i++)
            ofile << "v "<< mVertices(i,0)<<" "<< mVertices(i,1)<<" "<< mVertices(i,2)<<endl;

        int cnt=0;
        for(int i=0;i<mNbPolygons;i++){
            ofile << "f ";
            for(int j=0;j<mPolygonSize[i]-1;j++){
                ofile << mPolygons[cnt++]+1 <<" ";
            }
            ofile << mPolygons[cnt++]+1 <<endl;
        }

        ofile.close();
        return true;
    }else{
        return false;
    }
 
}

bool    Base3DObject::LoadFromObjFile(const char *filename, bool invNormals){

    Free();

    ifstream ifile;
    ifile.open(filename);
    //cerr <<"Opening file <"<<filename<<">"<<endl;
    if(!ifile.is_open()){
        cerr <<"Error while opening file <"<<filename<<">"<<endl;
        return false;
    }

    char cline[512];
    char c;

    REALTYPE *verPos = sVerData;
    int      *triPos = sTriData;

    mNbVertices  = 0;
    mNbPolygons = 0;

    int polyIndex[256];
    int polyIndexCnt=0;
    ifile.getline(cline,512);
    while(!ifile.eof()){
        if(cline[0]=='v'){


            if(cline[1]==' '){
                float x,y,z;
                sscanf(cline,"%c %f %f %f",&c,&x,&y,&z);
                (*verPos++) = REALTYPE(x);
                (*verPos++) = REALTYPE(y);
                (*verPos++) = REALTYPE(z);
                mNbVertices++;
                if(mNbVertices>=sBASE3DOBJ_FILE_MAX_VERTICES){
                    int memOff = verPos - sVerData;
                    //cout << "Vertoff "<<memOff<<endl;
                    ResizeStaticVar();
                    verPos = sVerData + memOff;
                    triPos = sTriData;
                }
            }
        }else if (cline[0]=='f'){


            stringstream ss(RemoveSpaces(cline+1));
            int cnt = 0;
            char c;

            //float tmp;
            while(!ss.eof()){
	      //		  cout<<"eee"<<endl;    
                ss >> polyIndex[cnt];
                ss.get(c);
		//		cout<<polyIndex[cnt]<<endl;
		//		cout<<c<<endl;
                if(c=='/' || c=='\n' || '\r'){


                    while((c!=' ')&&(!ss.eof())){
                        ss.get(c);
                    }
                }else{
                    ss.unget();
                }
		//                cout << polyIndex[cnt]<<endl;
                cnt++;
            }

            if(cnt>2){
                sPolySize[mNbPolygons] = cnt;
                if(!invNormals){
                    for(int i=0;i<cnt;i++)
                        *(triPos++) = polyIndex[i]-1;
                }else{
                    for(int i=cnt-1;i>=0;i--)
                        *(triPos++) = polyIndex[i]-1;
                }
                polyIndexCnt += cnt;
                mNbPolygons++;
                if((mNbPolygons>=sBASE3DOBJ_FILE_MAX_TRIANGLES)||(triPos - sTriData >= sBASE3DOBJ_FILE_MAX_TRIANGLES*3)){
                    int memOff = triPos - sTriData;
                    //cout << "Polyoff "<<memOff<<endl;
                    ResizeStaticVar();
                    triPos = sTriData + memOff;
                }
            }
        }
        ifile.getline(cline,512);
    }
    ifile.close();

//    cout << "OK"<<endl;
    mPolygons   = new int      [polyIndexCnt];
    mPolygonSize = new int     [mNbPolygons];
    mVertices.Resize(mNbVertices,3,false);

    memcpy(mVertices.Array(), sVerData,  mNbVertices  *3 *sizeof(REALTYPE));
    memcpy(mPolygons,         sTriData,  polyIndexCnt    *sizeof(int));
    memcpy(mPolygonSize,      sPolySize, mNbPolygons    *sizeof(int));

    int cumSize = 0;
    mPolygonStart = new int      [mNbPolygons];
    for(int i=0;i<mNbPolygons;i++){
        mPolygonStart[i] = cumSize;
        cumSize += mPolygonSize[i];
    }

    gLOG.SetCurrentEntry("Base3DObject");
    gLOG.Append("Object  <%s> loaded. Found %d vertices and %d polygons",filename, mNbVertices,mNbPolygons);

    //cout <<"Loaded Object: "<<filename<<" Vertices: "<<mNbVertices<<" Polygons: "<<mNbPolygons<<endl;
    CalcNormals();

    return true;
}

#define NORMAL_MERGE_VALUE 0.866025404

void Base3DObject::CalcNormals(bool perPolyVertex, bool perVertex){
    if(perVertex) perPolyVertex= true;
    bNormalsPerVertex = perPolyVertex;

    if(bNormalsPerVertex){
        CalcNormals(false);
        bNormalsPerVertex = true;
        Matrix tmpNormals = mNormals;

        int cumSize = 0; for(int i=0;i<mNbPolygons;i++) cumSize += mPolygonSize[i];
        mNormals.Resize(cumSize,3,false);

        Matrix cumNormals(cumSize,3);
        Vector cumCount(cumSize);

        CalcNeighbours();

        if(perVertex)
            mNormalsPerVertex.Resize(mNbVertices,3);

        int cnt      = 0;
        int localCnt = 0;

        //REALTYPE *v1,*v2,*v3;
        Vector3 a,b,n;
        Vector cn;
        Vector3 n2;
        for(int i=0;i<mNbPolygons;i++){
            localCnt = cnt;

            for(int j=0;j<mPolygonSize[i];j++){

                n.x() = tmpNormals(i,0);
                n.y() = tmpNormals(i,1);
                n.z() = tmpNormals(i,2);
                a = n;
                n2 = n;

                for(int k=0;k<mVertexNeighbourSize[mPolygons[localCnt]];k++){
                    int id = mVertexNeighbours[mVertexNeighbourStart[mPolygons[localCnt]]+k];
                    if(id!=i){
                        b.x() = tmpNormals(id,0);
                        b.y() = tmpNormals(id,1);
                        b.z() = tmpNormals(id,2);
                        if(n.Dot(b)>=NORMAL_MERGE_VALUE){
                            a+=b;
                        }
                        n2 += b;
                    }
                }
                n = a;
                n.Normalize();
                mNormals(localCnt,0) = n.x();
                mNormals(localCnt,1) = n.y();
                mNormals(localCnt,2) = n.z();

                if(perVertex){
                    n2.Normalize();
                    mNormalsPerVertex(mPolygons[localCnt],0) = n2.x();
                    mNormalsPerVertex(mPolygons[localCnt],1) = n2.y();
                    mNormalsPerVertex(mPolygons[localCnt],2) = n2.z();
                }

                localCnt++;
            }
            cnt += mPolygonSize[i];
        }
    }else{
        mNormals.Resize(mNbPolygons,3,false);
        int cnt = 0;
        REALTYPE *v1,*v2,*v3;
        Vector3 a,b,n;
        for(int i=0;i<mNbPolygons;i++){
            v1 = mVertices.Array()+(mPolygons[cnt+0])*3;
            v2 = mVertices.Array()+(mPolygons[cnt+1])*3;
            v3 = mVertices.Array()+(mPolygons[cnt+2])*3;
            a.x() =  v2[0]-v1[0];
            a.y() =  v2[1]-v1[1];
            a.z() =  v2[2]-v1[2];
            b.x() =  v3[0]-v1[0];
            b.y() =  v3[1]-v1[1];
            b.z() =  v3[2]-v1[2];
            a.Cross(b,n);
            n.Normalize();
            mNormals(i,0) = n.x();
            mNormals(i,1) = n.y();
            mNormals(i,2) = n.z();
            cnt+=mPolygonSize[i];
        }
    }
}


void Base3DObject::CalcPlanes(){
    mPlanes.Resize(mNbPolygons,4,false);

    int cnt = 0;
    REALTYPE *v1,*v2,*v3;
    Vector3 a,b,n;
    for(int i=0;i<mNbPolygons;i++){
        v1 = mVertices.Array()+(mPolygons[cnt+0])*3;
        v2 = mVertices.Array()+(mPolygons[cnt+1])*3;
        v3 = mVertices.Array()+(mPolygons[cnt+2])*3;

        mPlanes(i,0) = v1[1]*(v2[2]-v3[2]) + v2[1]*(v3[2]-v1[2]) + v3[1]*(v1[2]-v2[2]);
        mPlanes(i,1) = v1[2]*(v2[0]-v3[0]) + v2[2]*(v3[0]-v1[0]) + v3[2]*(v1[0]-v2[0]);
        mPlanes(i,2) = v1[0]*(v2[1]-v3[1]) + v2[0]*(v3[1]-v1[1]) + v3[0]*(v1[1]-v2[1]);
        mPlanes(i,3) = -( v1[0]*( v2[1]*v3[2] - v3[1]*v2[2] ) + v2[0]*(v3[1]*v1[2] - v1[1]*v3[2]) + v3[0]*(v1[1]*v2[2] - v2[1]*v1[2]) );

        cnt+=mPolygonSize[i];
    }
    //mPlanes.Print();
}

void  Base3DObject::CalcNeighbours(){
    int cumSize = 0; for(int i=0;i<mNbPolygons;i++) cumSize += mPolygonSize[i];

    if(mEdgeNeighbours!=NULL) delete [] mEdgeNeighbours;
    mEdgeNeighbours     = new int [cumSize];
    for(int i=0;i<cumSize;i++) mEdgeNeighbours[i] = -1;


    if(mIsVisible!=NULL) delete [] mIsVisible;
    mIsVisible      = new int [mNbPolygons];


    if(mVertexNeighbours!=NULL) delete [] mVertexNeighbours;
    mVertexNeighbours     = new int [cumSize];

    if(mVertexNeighbourSize!=NULL) delete [] mVertexNeighbourSize;
    mVertexNeighbourSize     = new int [mNbVertices];
    memset(mVertexNeighbourSize,0,mNbVertices*sizeof(int));

    int cnt=0;
	for(int i=0;i<mNbPolygons;i++){
		for(int ki=0;ki<mPolygonSize[i];ki++){
            mVertexNeighbourSize[mPolygons[cnt]]++;
            cnt++;
        }
    }

    if(mVertexNeighbourStart!=NULL) delete [] mVertexNeighbourStart;
    mVertexNeighbourStart = new int [mNbVertices];

    int vcumSize = 0;
    for(int i=0;i<mNbVertices;i++){
        mVertexNeighbourStart[i] = vcumSize;
        vcumSize += mVertexNeighbourSize[i];
    }

    memset(mVertexNeighbourSize,0,mNbVertices*sizeof(int));

    cnt=0;
	for(int i=0;i<mNbPolygons;i++){
		for(int ki=0;ki<mPolygonSize[i];ki++){
            mVertexNeighbours[mVertexNeighbourStart[mPolygons[cnt]]+mVertexNeighbourSize[mPolygons[cnt]]] = i;
            mVertexNeighbourSize[mPolygons[cnt]]++;
            cnt++;
        }
    }


    int ioff = 0;
    int joff = 0;
	for(int i=0;i<mNbPolygons-1;i++){

		for(int ki=0;ki<mPolygonSize[i];ki++){

            if(mEdgeNeighbours[ioff+ki]<0){
                bool bFound = false;

                joff = ioff + mPolygonSize[i];
    		    for(int j=i+1;j<mNbPolygons;j++){

					for(int kj=0;kj<mPolygonSize[j];kj++){

						int p1i=mPolygons[ioff+ki];
						int p2i=mPolygons[ioff+((ki+1)%mPolygonSize[i])];

						int p1j=mPolygons[joff+kj];
						int p2j=mPolygons[joff+((kj+1)%mPolygonSize[j])];

                        //cout << i<<" "<<j<<": "<<ki<<" "<<kj<<": "<<p1i<<" "<<p2i<<" "<<p1j<<" "<<p2j<<endl;

                        if(((p1i==p1j) && (p2i==p2j)) || ((p1i==p2j)&&(p2i==p1j))){
                            mEdgeNeighbours[ioff+ki] = j;
                            mEdgeNeighbours[joff+kj] = i;
                            //cout << "found"<<endl;
                            bFound = true;
                            break;
	                    }
                    }
                    if(bFound) break;
                    joff += mPolygonSize[j];
                }
            }
        }
        ioff += mPolygonSize[i];
    }
}

bool RayTriangleIntersect(const Vector3 & p1, const Vector3 & p2, const Vector3 & v0, const Vector3 & v1, const Vector3 & v2, double & t)
{
    Vector3 D, e1, e2, P, T, Q;
    double det, inv_det, u, v;

    // calculate length of vector
    D = p2 - p1;

    // calculate triangle edges
    e1 = v1 - v0;
    e2 = v2 - v0;

    D.Cross(e2, P);

    det = (e1.x()*P.x()) + (e1.y()*P.y()) + (e1.z()*P.z());

    // parallel ray-triangles do not intersect.
    if (det==0.0)
        return false;

    inv_det = 1.0/det;

    T = p1 - v0;

    u = ((T.x()*P.x()) + (T.y()*P.y()) + (T.z()*P.z())) * inv_det;

    if (u<0.0 || u>1.0)
        return false; // u range error

    T.Cross(e1, Q);

    v = ((D.x()*Q.x()) + (D.y()*Q.y()) + (D.z()*Q.z())) * inv_det;

    if (v<0.0 || u+v>1.0)
        return false; // v range error

    t = ((e2.x()*Q.x()) + (e2.y()*Q.y()) + (e2.z()*Q.z())) * inv_det;
    return true;
}


bool Base3DObject::ComputeRayIntersection(Vector3 vecRay1, Vector3 vecRay2, const Matrix4 & objTransform, double & dFactor){
    ConvertToTriangles();

    dFactor          = R_INFINITY;
    bool bFound      = false;
    //int polyIndexCnt = 0;
    double dTmpFactor;
    Vector3 vec1, vec2, vec3;
    Vector vecTmp(4), vecSrc(4);
    Matrix matTmp(objTransform);
    REALTYPE pValue[4] = {0.0, 0.0, 0.0, 1.0};

    for(int i=0;i<mNbPolygons;i++){
        int iBasePolygonPos = mPolygonStart[i];

        pValue[0] = mVertices.At(mPolygons[iBasePolygonPos], 0);
        pValue[1] = mVertices.At(mPolygons[iBasePolygonPos], 1);
        pValue[2] = mVertices.At(mPolygons[iBasePolygonPos], 2);
        vecSrc.Set(pValue,4);
        vecTmp.Set(matTmp.Mult(vecSrc));
        vec1.Set(vecTmp.At(0), vecTmp.At(1), vecTmp.At(2));

        pValue[0] = mVertices.At(mPolygons[iBasePolygonPos+1], 0);
        pValue[1] = mVertices.At(mPolygons[iBasePolygonPos+1], 1);
        pValue[2] = mVertices.At(mPolygons[iBasePolygonPos+1], 2);
        vecSrc.Set(pValue,4);
        vecTmp.Set(matTmp.Mult(vecSrc));
        vec2.Set(vecTmp.At(0), vecTmp.At(1), vecTmp.At(2));

        pValue[0] = mVertices.At(mPolygons[iBasePolygonPos+2], 0);
        pValue[1] = mVertices.At(mPolygons[iBasePolygonPos+2], 1);
        pValue[2] = mVertices.At(mPolygons[iBasePolygonPos+2], 2);
        vecSrc.Set(pValue,4);
        vecTmp.Set(matTmp.Mult(vecSrc));
        vec3.Set(vecTmp.At(0), vecTmp.At(1), vecTmp.At(2));

        //std::cout << "Polygon " << i << ": " << vec1.x() << ", " << vec1.y() << ", " << vec1.z() << "." << std::endl;
        //std::cout << vec2.x() << ", " << vec2.y() << ", " << vec2.z() << "." << std::endl;
        //std::cout << vec3.x() << ", " << vec3.y() << ", " << vec3.z() << "." << std::endl;

        bool bFoundTmp = RayTriangleIntersect(
                vecRay1,
                vecRay2,
                vec1,
                vec2,
                vec3,
                dTmpFactor);

        if(bFoundTmp && dTmpFactor < dFactor){
            dFactor = dTmpFactor;
            bFound = true;
            //std::cout << "Found intersection on Polygon " << i << std::endl;
        }
    }

    return bFound;
}

void Base3DObject::ConvertToTriangles(){
    if(bOnlyTriangles)
        return;

    bOnlyTriangles = true;
    for(int i=0;i<mNbPolygons;i++){
        if(mPolygonSize[i]>3){
            bOnlyTriangles = false;
            break;
        }
    }
    if(bOnlyTriangles) return;

    int *triPos        = sTriData;
    int newNbTriangles = 0;
    int polyIndexCnt   = 0;
    for(int i=0;i<mNbPolygons;i++){
        for(int j=0;j<mPolygonSize[i]-2;j++){
            *(triPos++) = mPolygons[polyIndexCnt];
            *(triPos++) = mPolygons[polyIndexCnt+j+1];
            *(triPos++) = mPolygons[polyIndexCnt+j+2];
            sPolySize[newNbTriangles] = 3;
            sPolyStart[newNbTriangles] = 3*newNbTriangles;
            newNbTriangles++;
            if(newNbTriangles>=sBASE3DOBJ_FILE_MAX_TRIANGLES){
                int memOff = triPos - sTriData;
                //cout << "CTT Vertoff "<<memOff<<endl;
                ResizeStaticVar();
                triPos = sTriData + memOff;
            }

        }
        polyIndexCnt+=mPolygonSize[i];
    }
    mNbPolygons = newNbTriangles;
    delete [] mPolygons;
    delete [] mPolygonSize;
    delete [] mPolygonStart;
    mPolygons   = new int      [mNbPolygons*3];
    mPolygonSize = new int     [mNbPolygons];
    mPolygonStart = new int    [mNbPolygons];
    memcpy(mPolygons,        sTriData,   mNbPolygons *3*sizeof(int));
    memcpy(mPolygonSize,     sPolySize,  mNbPolygons   *sizeof(int));
    memcpy(mPolygonStart,    sPolyStart, mNbPolygons   *sizeof(int));

    //cout <<"Triangulation:  Vertices: "<<mNbVertices<<" Polygons: "<<mNbPolygons<<endl;

    CalcNormals();
}

void Base3DObject::AddOffset(const Vector3& offset){
    Vector off(offset.Array(),3);
    mVertices.SAddToRow(off);
}

void Base3DObject::Transform(const Matrix3& trans){
    SharedMatrix mat(trans);
    Matrix tmp(mNbPolygons,3,false);
    mVertices.MultTranspose2(mat,tmp);
    mVertices.Swap(tmp);
    mNormals.MultTranspose2(mat,tmp);
    mNormals.Swap(tmp);
}

void Base3DObject::Transform(const Matrix4& trans){
    Transform(trans.GetOrientation());
    AddOffset(trans.GetTranslation());
}

void  Base3DObject::Render(){}







void Base3DObject::CheckVisibility(const Vector3& point){
	//set visual parameter
    Vector p(4);
    p(0) = point.AtNoCheck(0);
    p(1) = point.AtNoCheck(1);
    p(2) = point.AtNoCheck(2);
    p(3) = 1.0;
    Vector res;
    mPlanes.Mult(p,res);
    for(int i=0;i<mNbPolygons;i++){
        if(res.AtNoCheck(i)>0.0)
            mIsVisible[i] = 1;
        else
            mIsVisible[i] = 0;
    }
    /*
    for(int i=0;i<mNbPolygons;i++){
        cout << i<<" "<<mIsVisible[i]<<endl;
    }
    */
}




Base3DObject*   Base3DObject::Clone(Base3DObject* res){
    if(res==NULL)
        res = new Base3DObject();

    res->mNbVertices    = mNbVertices;
    res->mNbPolygons    = mNbPolygons;

    res->mPolygonSize   = new int [mNbPolygons];
    memcpy(res->mPolygonSize,mPolygonSize,mNbPolygons*sizeof(int));

    res->mPolygonStart   = new int [mNbPolygons];
    memcpy(res->mPolygonStart,mPolygonStart,mNbPolygons*sizeof(int));

    int cumSize = 0; for(int i=0;i<mNbPolygons;i++) cumSize += mPolygonSize[i];

    res->mPolygons      = new int [cumSize];
    memcpy(res->mPolygons,mPolygons,cumSize*sizeof(int));

    res->mVertices      = mVertices;
    res->mNormals       = mNormals;

    cout <<"OBJ CLONE: "<<cumSize<<" "<<res->mNbPolygons<<endl;
    return res;
}



void Base3DObject::GenerateCylinder(int nbSlices,int nbStacks){
    Free();
    if(nbStacks<1) nbStacks = 1;

    mNbVertices = nbSlices*(2+nbStacks-1);
    mNbPolygons = nbSlices*nbStacks+2;
    mPolygons    = new int      [nbSlices*nbStacks*4+2*nbSlices];
    mPolygonSize = new int      [mNbPolygons];
    mPolygonStart = new int     [mNbPolygons];
    mVertices.Resize(mNbVertices,3,false);
    for(int j=0;j<nbStacks+1;j++){
        int off       = j*nbSlices;
        REALTYPE hoff = -0.5+REALTYPE(j)/REALTYPE(nbStacks);
        for(int i=0;i<nbSlices;i++){
            REALTYPE a = REALTYPE(i)/REALTYPE(nbSlices)*TWOPI;
            REALTYPE x = 0.5*cos(a);
            REALTYPE y = 0.5*sin(a);
            mVertices(i+off,0) = x;//mVertices(i+nbSlices,0) = x;
            mVertices(i+off,1) = y;//mVertices(i+nbSlices,1) = y;
            mVertices(i+off,2) = hoff;
            //mVertices(i+nbSlices,2) =  0.5;
        }
    }

    for(int j=0;j<nbStacks;j++){
        int pid = j*nbSlices;
        for(int i=0;i<nbSlices-1;i++){
            mPolygons[pid*4+i*4+0] = pid+i;
            mPolygons[pid*4+i*4+1] = pid+i+1;
            mPolygons[pid*4+i*4+2] = pid+nbSlices+i+1;
            mPolygons[pid*4+i*4+3] = pid+nbSlices+i;
            mPolygonSize[pid+i]  = 4;
            mPolygonStart[pid+i] = pid*4+4*i;
        }
        int i=nbSlices-1;
        mPolygons[pid*4+i*4+0] = pid+i;
        mPolygons[pid*4+i*4+1] = pid+0;
        mPolygons[pid*4+i*4+2] = pid+nbSlices+0;
        mPolygons[pid*4+i*4+3] = pid+nbSlices+i;
        mPolygonSize[pid+i]  = 4;
        mPolygonStart[pid+i] = pid*4+4*i;
    }

    int pid = nbSlices*nbStacks;
    for(int i=0;i<nbSlices;i++){
        mPolygons[pid*4+nbSlices*0+i] = nbSlices-i-1;
        mPolygons[pid*4+nbSlices*1+i] = nbStacks*nbSlices+i;
    }
    mPolygonSize[pid+0]  = nbSlices;
    mPolygonSize[pid+1]  = nbSlices;
    mPolygonStart[pid+0] = pid*4;//nbSlices*4;
    mPolygonStart[pid+1] = pid*4+nbSlices;

    CalcNormals();
}
void Base3DObject::GenerateSphere(int nbSlices, int nbStacks){
    Free();
    mNbVertices = (nbStacks-1)*nbSlices+2;
    mNbPolygons = nbSlices*(nbStacks);
    mPolygons    = new int      [2*nbSlices*3 + (nbStacks-2)*nbSlices*4];
    mPolygonSize = new int      [mNbPolygons];
    mPolygonStart = new int     [mNbPolygons];
    mVertices.Resize(mNbVertices,3,false);
    for(int j=0;j<nbStacks-1;j++){
        REALTYPE e = REALTYPE(j+1)/REALTYPE(nbStacks)*PI-PI/2.0;
        REALTYPE z = 0.5*sin(e);
        for(int i=0;i<nbSlices;i++){
            REALTYPE a = REALTYPE(i)/REALTYPE(nbSlices)*TWOPI;
            REALTYPE x = 0.5*cos(a)*cos(e);
            REALTYPE y = 0.5*sin(a)*cos(e);
            mVertices(i+j*nbSlices,0) = x;
            mVertices(i+j*nbSlices,1) = y;
            mVertices(i+j*nbSlices,2) = z;
        }
    }
    mVertices((nbStacks-1)*nbSlices+0,0) = mVertices((nbStacks-1)*nbSlices+1,0) = R_ZERO;
    mVertices((nbStacks-1)*nbSlices+0,1) = mVertices((nbStacks-1)*nbSlices+1,1) = R_ZERO;
    mVertices((nbStacks-1)*nbSlices+0,2) = -0.5;
    mVertices((nbStacks-1)*nbSlices+1,2) =  0.5;

    for(int j=0;j<nbStacks-2;j++){
        for(int i=0;i<nbSlices-1;i++){
            mPolygons[(i+j*nbSlices)*4+0] = j*nbSlices+i;
            mPolygons[(i+j*nbSlices)*4+1] = j*nbSlices+i+1;
            mPolygons[(i+j*nbSlices)*4+2] = (j+1)*nbSlices+i+1;
            mPolygons[(i+j*nbSlices)*4+3] = (j+1)*nbSlices+i;
            mPolygonSize[(i+j*nbSlices)]  = 4;
            mPolygonStart[(i+j*nbSlices)] = (i+j*nbSlices)*4;
        }
        int i=nbSlices-1;
        mPolygons[(i+j*nbSlices)*4+0] = j*nbSlices+i;
        mPolygons[(i+j*nbSlices)*4+1] = j*nbSlices+0;
        mPolygons[(i+j*nbSlices)*4+2] = (j+1)*nbSlices+0;
        mPolygons[(i+j*nbSlices)*4+3] = (j+1)*nbSlices+i;
        mPolygonSize[(i+j*nbSlices)]  = 4;
        mPolygonStart[(i+j*nbSlices)] = (i+j*nbSlices)*4;
    }
    for(int j=0;j<2;j++){
        int off = (nbStacks-2)*nbSlices;
        for(int i=0;i<nbSlices-1;i++){
            mPolygons[off*4+i*3+0] = (nbStacks-1)*nbSlices+0;
            mPolygons[off*4+i*3+1] = i+1;
            mPolygons[off*4+i*3+2] = i;
            mPolygonSize[off+i]  = 3;
            mPolygonStart[off+i] = off*4+i*3;

            mPolygons[off*4+(nbSlices+i)*3+0] = (nbStacks-1)*nbSlices+1;
            mPolygons[off*4+(nbSlices+i)*3+1] = (nbStacks-2)*nbSlices+i;
            mPolygons[off*4+(nbSlices+i)*3+2] = (nbStacks-2)*nbSlices+i+1;
            mPolygonSize[off+(nbSlices+i)]    = 3;
            mPolygonStart[off+(nbSlices+i)]   = off*4+(nbSlices+i)*3;
        }
        int i= nbSlices-1;
        mPolygons[off*4+i*3+0] = (nbStacks-1)*nbSlices+0;
        mPolygons[off*4+i*3+1] = 0;
        mPolygons[off*4+i*3+2] = i;
        mPolygonSize[off+i]  = 3;
        mPolygonStart[off+i] = off*4+i*3;
        mPolygons[off*4+(nbSlices+i)*3+0] = (nbStacks-1)*nbSlices+1;
        mPolygons[off*4+(nbSlices+i)*3+1] = (nbStacks-2)*nbSlices+i;
        mPolygons[off*4+(nbSlices+i)*3+2] = (nbStacks-2)*nbSlices+0;
        mPolygonSize[off+(nbSlices+i)]    = 3;
        mPolygonStart[off+(nbSlices+i)]   = off*4+(nbSlices+i)*3;
    }
    CalcNormals();
}

void Base3DObject::GenerateCapsule(REALTYPE capRatio, int nbSlices, int nbSideStacks){
    Free();

    int nbStacks = 2*nbSideStacks;

    mNbVertices = (nbStacks-1)*nbSlices+2   + nbSlices;
    mNbPolygons = nbSlices*(nbStacks)       + nbSlices;
    mPolygons    = new int      [2*nbSlices*3 + (nbStacks-2)*nbSlices*4 + nbSlices*4];
    mPolygonSize = new int      [mNbPolygons];
    mPolygonStart = new int     [mNbPolygons];
    mVertices.Resize(mNbVertices,3,false);
    for(int j=0;j<nbStacks;j++){
        int jj = (j<nbSideStacks?j:j-1);

        REALTYPE e = REALTYPE(jj+1)/REALTYPE(nbStacks)*PI-PI/2.0;
        REALTYPE z = 0.5*capRatio*(j<nbSideStacks?-1.0:1.0)+0.5*sin(e);
        for(int i=0;i<nbSlices;i++){
            REALTYPE a = REALTYPE(i)/REALTYPE(nbSlices)*TWOPI;
            REALTYPE x = 0.5*cos(a)*cos(e);
            REALTYPE y = 0.5*sin(a)*cos(e);
            mVertices(i+j*nbSlices,0) = x;
            mVertices(i+j*nbSlices,1) = y;
            mVertices(i+j*nbSlices,2) = z;
        }
    }
    mVertices((nbStacks)*nbSlices+0,0) = mVertices((nbStacks)*nbSlices+1,0) = R_ZERO;
    mVertices((nbStacks)*nbSlices+0,1) = mVertices((nbStacks)*nbSlices+1,1) = R_ZERO;
    mVertices((nbStacks)*nbSlices+0,2) = -0.5*capRatio -0.5;
    mVertices((nbStacks)*nbSlices+1,2) =  0.5*capRatio +0.5;

    for(int j=0;j<nbStacks-1;j++){
        for(int i=0;i<nbSlices-1;i++){
            mPolygons[(i+j*nbSlices)*4+0] = j*nbSlices+i;
            mPolygons[(i+j*nbSlices)*4+1] = j*nbSlices+i+1;
            mPolygons[(i+j*nbSlices)*4+2] = (j+1)*nbSlices+i+1;
            mPolygons[(i+j*nbSlices)*4+3] = (j+1)*nbSlices+i;
            mPolygonSize[(i+j*nbSlices)]  = 4;
            mPolygonStart[(i+j*nbSlices)] = (i+j*nbSlices)*4;
        }
        int i=nbSlices-1;
        mPolygons[(i+j*nbSlices)*4+0] = j*nbSlices+i;
        mPolygons[(i+j*nbSlices)*4+1] = j*nbSlices+0;
        mPolygons[(i+j*nbSlices)*4+2] = (j+1)*nbSlices+0;
        mPolygons[(i+j*nbSlices)*4+3] = (j+1)*nbSlices+i;
        mPolygonSize[(i+j*nbSlices)]  = 4;
        mPolygonStart[(i+j*nbSlices)] = (i+j*nbSlices)*4;
    }
    //for(int j=0;j<2;j++){
        int off = (nbStacks-1)*nbSlices;
        for(int i=0;i<nbSlices-1;i++){
            mPolygons[off*4+i*3+0] = (nbStacks)*nbSlices+0;
            mPolygons[off*4+i*3+1] = i+1;
            mPolygons[off*4+i*3+2] = i;
            mPolygonSize[off+i]    = 3;
            mPolygonStart[off+i]   = off*4+i*3;

            mPolygons[off*4+(nbSlices+i)*3+0] = (nbStacks)*nbSlices+1;
            mPolygons[off*4+(nbSlices+i)*3+1] = (nbStacks-1)*nbSlices+i;
            mPolygons[off*4+(nbSlices+i)*3+2] = (nbStacks-1)*nbSlices+i+1;
            mPolygonSize[off+(nbSlices+i)]    = 3;
            mPolygonStart[off+(nbSlices+i)]   = off*4+(nbSlices+i)*3;
        }
        int i= nbSlices-1;
        mPolygons[off*4+i*3+0] = (nbStacks)*nbSlices+0;
        mPolygons[off*4+i*3+1] = 0;
        mPolygons[off*4+i*3+2] = i;
        mPolygonSize[off+i]    = 3;
        mPolygonStart[off+i]   = off*4+i*3;

        mPolygons[off*4+(nbSlices+i)*3+0] = (nbStacks)*nbSlices+1;
        mPolygons[off*4+(nbSlices+i)*3+1] = (nbStacks-1)*nbSlices+i;
        mPolygons[off*4+(nbSlices+i)*3+2] = (nbStacks-1)*nbSlices+0;
        mPolygonSize[off+(nbSlices+i)]    = 3;
        mPolygonStart[off+(nbSlices+i)]   = off*4+(nbSlices+i)*3;
    //}
    CalcNormals();
}

/*
void Base3DObject::GenerateCapsule(REALTYPE capRatio, int nbSlices, int nbSideStacks){
    Free();
    int offset=0;

    mNbVertices = nbSlices*2 + (nbSlices*(nbSideStacks-1)+1)*2;
    mNbPolygons = nbSlices   + (nbSlices*(nbSideStacks))    *2;
    mPolygons    = new int      [nbSlices*4+    (nbSlices*3 + (nbSideStacks-1)*nbSlices*4)*2];
    mPolygonSize = new int      [mNbPolygons];
    mVertices.Resize(mNbVertices,3,false);
    for(int i=0;i<nbSlices;i++){
        REALTYPE a = REALTYPE(i)/REALTYPE(nbSlices)*TWOPI;
        REALTYPE x = 0.5*cos(a);
        REALTYPE y = 0.5*sin(a);
        mVertices(i,0) = mVertices(i+nbSlices,0) = x;
        mVertices(i,1) = mVertices(i+nbSlices,1) = y;
        mVertices(i,2)                = -0.5;
        mVertices(i+nbSlices,2) =  0.5;
    }

    offset += 2*nbSlices;

    for(int j=0;j<nbSideStacks-1;j++){
        REALTYPE e = REALTYPE(j+1)/REALTYPE(nbSideStacks)*PI/2.0;
        REALTYPE z = 0.5+0.5*capRatio*sin(e);
        for(int i=0;i<nbSlices;i++){
            REALTYPE a = REALTYPE(i)/REALTYPE(nbSlices)*TWOPI;
            REALTYPE x = 0.5*cos(a)*cos(e);
            REALTYPE y = 0.5*sin(a)*cos(e);
            mVertices(offset+i+j*nbSlices,0) = x;
            mVertices(offset+i+j*nbSlices,1) = y;
            mVertices(offset+i+j*nbSlices,2) = z;
        }
    }
    mVertices(offset+(nbSideStacks-1)*nbSlices+0,0) = R_ZERO;
    mVertices(offset+(nbSideStacks-1)*nbSlices+0,1) = R_ZERO;
    mVertices(offset+(nbSideStacks-1)*nbSlices+0,2) = 0.5+0.5*capRatio;

    offset += (nbSideStacks-1)*nbSlices+1;

    for(int j=0;j<nbSideStacks-1;j++){
        REALTYPE e = REALTYPE(j+1)/REALTYPE(nbSideStacks)*PI/2.0;
        REALTYPE z = -0.5-0.5*capRatio*sin(e);
        for(int i=0;i<nbSlices;i++){
            REALTYPE a = REALTYPE(i)/REALTYPE(nbSlices)*TWOPI;
            REALTYPE x = 0.5*cos(a)*cos(e);
            REALTYPE y = 0.5*sin(a)*cos(e);
            mVertices(offset+i+j*nbSlices,0) = x;
            mVertices(offset+i+j*nbSlices,1) = y;
            mVertices(offset+i+j*nbSlices,2) = z;
        }
    }
    mVertices(offset+(nbSideStacks-1)*nbSlices+0,0) = R_ZERO;
    mVertices(offset+(nbSideStacks-1)*nbSlices+0,1) = R_ZERO;
    mVertices(offset+(nbSideStacks-1)*nbSlices+0,2) = -0.5-0.5*capRatio;








    for(int i=0;i<nbSlices-1;i++){
        mPolygons[i*4+0] = i;
        mPolygons[i*4+1] = i+1;
        mPolygons[i*4+2] = nbSlices+i+1;
        mPolygons[i*4+3] = nbSlices+i;
        mPolygonSize[i]  = 4;
    }
    int i=nbSlices-1;
    mPolygons[i*4+0] = i;
    mPolygons[i*4+1] = 0;
    mPolygons[i*4+2] = nbSlices+0;
    mPolygons[i*4+3] = nbSlices+i;
    mPolygonSize[i]  = 4;








    CalcNormals();

}
*/


void Base3DObject::GenerateCube(){
    Free();
    mNbVertices = 8;
    mNbPolygons = 6;
    mPolygons    = new int      [mNbPolygons*4];
    mPolygonSize = new int      [mNbPolygons];
    mPolygonStart = new int     [mNbPolygons];
    mVertices.Resize(mNbVertices,3,false);
    REALTYPE v[8][3] = {
        {-0.5, -0.5, -0.5},
        {-0.5, -0.5,  0.5},
        {-0.5,  0.5,  0.5},
        {-0.5,  0.5, -0.5},
        { 0.5, -0.5, -0.5},
        { 0.5, -0.5,  0.5},
        { 0.5,  0.5,  0.5},
        { 0.5,  0.5, -0.5}};
    int f[6][4] = {
        {0,1,2,3},
        {4,5,1,0},
        {5,6,2,1},
        {7,3,2,6},
        {4,7,6,5},
        {7,4,0,3}};
    memcpy(mVertices.Array(),v,mNbVertices*3*sizeof(REALTYPE));
    memcpy(mPolygons,f,mNbPolygons*4*sizeof(int));
    for(int i=0;i<mNbPolygons;i++){
        mPolygonSize[i]  = 4;
        mPolygonStart[i] = 4*i;
    }
    CalcNormals();
}

void Base3DObject::GenerateHeightField(const Matrix &heightData,REALTYPE sx,REALTYPE sy,REALTYPE sz){
    Free();
    mNbVertices = heightData.RowSize()*heightData.ColumnSize() + 2*heightData.RowSize() + 2*heightData.ColumnSize() - 4;
    mNbPolygons = +1 + 2*(heightData.ColumnSize()-1) + 2*(heightData.RowSize()-1) +(heightData.RowSize()-1)*(heightData.ColumnSize()-1);

                  //5;
    mPolygons    = new int      [ (heightData.RowSize()-1)*(heightData.ColumnSize()-1)*4 +
                                  2*(heightData.ColumnSize()-1)*4+
                                  2*(heightData.RowSize()-1)*4+
                                  2*heightData.RowSize() + 2*heightData.ColumnSize() - 4];/* +
                                  2*(heightData.ColumnSize()+2) +
                                  4];*/
    mPolygonSize = new int      [mNbPolygons];
    mPolygonStart = new int     [mNbPolygons];
    mVertices.Resize(mNbVertices,3,false);

    int row = int(heightData.RowSize());
    int col = int(heightData.ColumnSize());

    REALTYPE cxo = (REALTYPE(col)-1.0)/2;
    REALTYPE cyo = (REALTYPE(row)-1.0)/2;
    int off = 0;
    for(int i=0;i<row;i++){
        for(int j=0;j<col;j++){
            mVertices(off,0) = (REALTYPE(j)-cxo)*sx;
            mVertices(off,1) = (REALTYPE(i)-cyo)*sy;
            mVertices(off,2) = heightData.AtNoCheck(i,j)*sz;
            off++;
        }
    }
    for(int j=0;j<col;j++){
        mVertices(off,0) = (REALTYPE(j)-cxo)*sx;
        mVertices(off,1) = -cyo*sy;
        mVertices(off,2) = -0.0;
        off++;
    }
    for(int i=1;i<row;i++){
        mVertices(off,0) = +cxo*sx;
        mVertices(off,1) = (REALTYPE(i)-cyo)*sy;
        mVertices(off,2) = -0.0;
        off++;
    }
    for(int j=col-2;j>=0;j--){
        mVertices(off,0) = (REALTYPE(j)-cxo)*sx;
        mVertices(off,1) = +cyo*sy;
        mVertices(off,2) = -0.0;
        off++;
    }
    for(int i=row-2;i>0;i--){
        mVertices(off,0) = -cxo*sx;
        mVertices(off,1) = (REALTYPE(i)-cyo)*sy;
        mVertices(off,2) = -0.0;
        off++;
    }

    int poff = 0;
    off = 0;
    for(int i=0;i<row-1;i++){
        for(int j=0;j<col-1;j++){
            mPolygons[off+0] = i*col+j;
            mPolygons[off+1] = i*col+(j+1);
            mPolygons[off+2] = (i+1)*col+(j+1);
            mPolygons[off+3] = (i+1)*col+j;

            mPolygonSize[poff] = 4;
            mPolygonStart[poff] = off;

            off+=4;
            poff++;
        }
    }

    int goff = row*col;
    for(int j=0;j<col-1;j++){
        mPolygons[off+0] = j+1;
        mPolygons[off+1] = j;
        mPolygons[off+2] = goff+j;
        mPolygons[off+3] = goff+j+1;

        mPolygonSize[poff] = 4;
        mPolygonStart[poff] = off;

        off+=4;
        poff++;
    }

    goff = row*col+col-1;
    for(int j=0;j<row-1;j++){
        mPolygons[off+0] = col-1+(j+1)*col;
        mPolygons[off+1] = col-1+j*col;
        mPolygons[off+2] = goff+j;
        mPolygons[off+3] = goff+j+1;

        mPolygonSize[poff] = 4;
        mPolygonStart[poff] = off;

        off+=4;
        poff++;
    }

    goff = row*col+col-1+row-1;
    for(int j=0;j<col-1;j++){
        mPolygons[off+0] = row*col-2-j;
        mPolygons[off+1] = row*col-j-1;
        mPolygons[off+2] = goff+j;
        mPolygons[off+3] = goff+j+1;

        mPolygonSize[poff] = 4;
        mPolygonStart[poff] = off;

        off+=4;
        poff++;
    }

    goff = row*col+col-1+row-1+col-1;
    for(int j=0;j<row-2;j++){
        mPolygons[off+0] = (row-2-j)*col;
        mPolygons[off+1] = (row-j-1)*col;
        mPolygons[off+2] = goff+j;
        mPolygons[off+3] = goff+j+1;

        mPolygonSize[poff] = 4;
        mPolygonStart[poff] = off;

        off+=4;
        poff++;
    }
    mPolygons[off+0] = col;
    mPolygons[off+1] = goff+row-2;
    mPolygons[off+2] = row*col;
    mPolygons[off+3] = 0;

    mPolygonSize[poff] = 4;
    mPolygonStart[poff] = off;

    off+=4;
    poff++;


    goff = row*col;
    for(int j=2*row+2*col-4-1;j>=0;j--){
        mPolygons[off++] = goff+j;
    }
    mPolygonSize[poff++] = 2*row+2*col-4;


    CalcNormals();
}
