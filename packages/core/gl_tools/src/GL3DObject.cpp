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

#include "GL3DObject.h"

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
using namespace std;




GL3DObject::GL3DObject()
:Base3DObject()
{
    mCallListId             =  0;
    mWireframeCallListId    =  0;
    mShadowObject   = NULL;
    bShadowReady    = false;
}

GL3DObject::~GL3DObject(){
    Free();
}

void GL3DObject::Free(){
    if(mCallListId>0)           glDeleteLists(mCallListId,1);
    mCallListId = 0;
    if(mWireframeCallListId>0)  glDeleteLists(mWireframeCallListId,1);
    mWireframeCallListId = 0;

    if(mShadowObject) delete mShadowObject;
    mShadowObject = NULL;

    Base3DObject::Free();
}

#ifdef MATHLIB_USE_DOUBLE_AS_REAL
#define MYGLNORMAL3 glNormal3d
#define MYGLVERTEX3 glVertex3d
#else
#define MYGLNORMAL3 glNormal3f
#define MYGLVERTEX3 glVertex3f
#endif
int GL3DObject::BuildDisplayList(){
  if(mCallListId>0)
    return TRUE;

    if(mShadowObject!=NULL){
        mShadowObject->CalcPlanes();
        mShadowObject->CalcNeighbours();
    }

  ConvertToTriangles();
  CalcNormals(bNormalsPerVertex);

  mCallListId = glGenLists(1);

  if(mCallListId>0)
    glNewList(mCallListId,GL_COMPILE);

    glBegin(GL_TRIANGLES);
    int cnt = 0;
    REALTYPE *v;
    if(bNormalsPerVertex){

        for(int i=0;i<mNbPolygons;i++){
            v = mNormals.Array()+(cnt+0)*3;
            MYGLNORMAL3(*v,*(v+1),*(v+2));

            v = mVertices.Array()+(mPolygons[cnt+0])*3;
            MYGLVERTEX3(*v,*(v+1),*(v+2));

            v = mNormals.Array()+(cnt+1)*3;
            MYGLNORMAL3(*v,*(v+1),*(v+2));

            v = mVertices.Array()+(mPolygons[cnt+1])*3;
            MYGLVERTEX3(*v,*(v+1),*(v+2));

            v = mNormals.Array()+(cnt+2)*3;
            MYGLNORMAL3(*v,*(v+1),*(v+2));

            v = mVertices.Array()+(mPolygons[cnt+2])*3;
            MYGLVERTEX3(*v,*(v+1),*(v+2));

            cnt+=3;
        }
        /*
        glEnd();

        cnt = 0;
        glBegin(GL_LINES);
        for(int i=0;i<mNbPolygons;i++){
            v = mVertices.Array()+(mPolygons[cnt+0])*3;
            MYGLVERTEX3(*v,*(v+1),*(v+2));
            n = mNormals.Array()+(cnt+0)*3;
            MYGLVERTEX3(*v+(*n)*0.1,*(v+1)+(*(n+1))*0.1,*(v+2)+(*(n+2))*0.1);

            v = mVertices.Array()+(mPolygons[cnt+1])*3;
            MYGLVERTEX3(*v,*(v+1),*(v+2));
            n = mNormals.Array()+(cnt+1)*3;
            MYGLVERTEX3(*v+(*n)*0.1,*(v+1)+(*(n+1))*0.1,*(v+2)+(*(n+2))*0.1);

            v = mVertices.Array()+(mPolygons[cnt+2])*3;
            MYGLVERTEX3(*v,*(v+1),*(v+2));
            n = mNormals.Array()+(cnt+2)*3;
            MYGLVERTEX3(*v+(*n)*0.1,*(v+1)+(*(n+1))*0.1,*(v+2)+(*(n+2))*0.1);

            cnt+=3;
        }
        */
    }else{
        for(int i=0;i<mNbPolygons;i++){
            v = mNormals.Array()+i*3;
            MYGLNORMAL3(*v,*(v+1),*(v+2));

            v = mVertices.Array()+(mPolygons[cnt+0])*3;

            MYGLVERTEX3(*v,*(v+1),*(v+2));

            v = mVertices.Array()+(mPolygons[cnt+1])*3;
            MYGLVERTEX3(*v,*(v+1),*(v+2));

            v = mVertices.Array()+(mPolygons[cnt+2])*3;
            MYGLVERTEX3(*v,*(v+1),*(v+2));

            cnt+=3;
        }
        /*
        glEnd();

        cnt = 0;
        glBegin(GL_LINES);
        for(int i=0;i<mNbPolygons;i++){
            n = mNormals.Array()+i*3;

            v = mVertices.Array()+(mPolygons[cnt+0])*3;
            MYGLVERTEX3(*v,*(v+1),*(v+2));
            MYGLVERTEX3(*v+(*n)*0.1,*(v+1)+(*(n+1))*0.1,*(v+2)+(*(n+2))*0.1);

            v = mVertices.Array()+(mPolygons[cnt+1])*3;
            MYGLVERTEX3(*v,*(v+1),*(v+2));
            MYGLVERTEX3(*v+(*n)*0.1,*(v+1)+(*(n+1))*0.1,*(v+2)+(*(n+2))*0.1);

            v = mVertices.Array()+(mPolygons[cnt+2])*3;
            MYGLVERTEX3(*v,*(v+1),*(v+2));
            MYGLVERTEX3(*v+(*n)*0.1,*(v+1)+(*(n+1))*0.1,*(v+2)+(*(n+2))*0.1);

            cnt+=3;
        }
        */
    }

    glEnd();



  if(mCallListId>0)
    glEndList();

  if(mCallListId>0)
    return TRUE;

  return FALSE;
}








int GL3DObject::BuildWireframeDisplayList(){
    if(mWireframeCallListId>0)
        return TRUE;

    //ConvertToTriangles();
    CalcNormals(bNormalsPerVertex);

    mWireframeCallListId = glGenLists(1);

    if(mWireframeCallListId>0)
        glNewList(mWireframeCallListId,GL_COMPILE);

    glBegin(GL_LINES);
    int cnt = 0;
    REALTYPE *v;

    if(bNormalsPerVertex){
        int offset      = 0;
        for(int i=0;i<mNbPolygons;i++){
            int localOffset = offset;
            for(int j=0;j<mPolygonSize[i]-1;j++){
                v = mNormals.Array()+(localOffset+0)*3;
                MYGLNORMAL3(*v,*(v+1),*(v+2));

                v = mVertices.Array()+(mPolygons[localOffset+0])*3;
                MYGLVERTEX3(*v,*(v+1),*(v+2));

                v = mNormals.Array()+(localOffset+1)*3;
                MYGLNORMAL3(*v,*(v+1),*(v+2));

                v = mVertices.Array()+(mPolygons[localOffset+1])*3;
                MYGLVERTEX3(*v,*(v+1),*(v+2));

                localOffset++;
            }
            v = mNormals.Array()+(localOffset+0)*3;
            MYGLNORMAL3(*v,*(v+1),*(v+2));

            v = mVertices.Array()+(mPolygons[localOffset+0])*3;
            MYGLVERTEX3(*v,*(v+1),*(v+2));

            v = mNormals.Array()+(offset)*3;
            MYGLNORMAL3(*v,*(v+1),*(v+2));

            v = mVertices.Array()+(mPolygons[offset])*3;
            MYGLVERTEX3(*v,*(v+1),*(v+2));

            offset+=mPolygonSize[i];
        }
    }else{
        for(int i=0;i<mNbPolygons;i++){
            v = mNormals.Array()+i*3;
            MYGLNORMAL3(*v,*(v+1),*(v+2));

            v = mVertices.Array()+(mPolygons[cnt+0])*3;

            MYGLVERTEX3(*v,*(v+1),*(v+2));

            v = mVertices.Array()+(mPolygons[cnt+1])*3;
            MYGLVERTEX3(*v,*(v+1),*(v+2));

            v = mVertices.Array()+(mPolygons[cnt+2])*3;
            MYGLVERTEX3(*v,*(v+1),*(v+2));

            cnt+=3;
        }
    }

    glEnd();



  if(mWireframeCallListId>0)
    glEndList();

  if(mWireframeCallListId>0)
    return TRUE;

  return FALSE;
}







//#define HIDE

void  GL3DObject::Render(){
  if(BuildDisplayList()){
#ifndef HIDE
    glCallList(mCallListId);
#endif
  }
}

void  GL3DObject::RenderWireframe(){
  if(BuildWireframeDisplayList()){
#ifndef HIDE
    glCallList(mWireframeCallListId);
#endif
  }
}




void    GL3DObject::RenderShadow(const Vector3& light){
    if(mShadowObject==NULL) return;

    RenderShadowInitPass(0);
    mShadowObject->RenderShadowPass(0,light);
    RenderShadowInitPass(1);
    mShadowObject->RenderShadowPass(1,light);
    RenderShadowEnd();
}

void    GL3DObject::RenderOutline(const Vector3& observer){
   if(!bShadowReady){
        CalcPlanes();
        CalcNeighbours();
        bShadowReady = true;
    }

    CheckVisibility(observer);

    int p1,p2,p3;
    int ioff = 0;
    double *vert = mVertices.Array();

	glEnable (GL_BLEND);
	glBlendFunc (GL_SRC_ALPHA ,GL_ONE_MINUS_SRC_ALPHA);
    glColor3f(0.0,0.0,0.0);
    glLineWidth (3);
    glPolygonMode (GL_BACK, GL_LINE);
    glCullFace (GL_FRONT);
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(0.0f, 100.0f);

    glBegin(GL_TRIANGLES);
    for(int i=0;i<mNbPolygons;i++){
        if(!mIsVisible[i]){
            bool bDraw = false;
            for(int j=0;j<mPolygonSize[i];j++){
                if(mEdgeNeighbours[ioff+j]>=0);
                bDraw = (mIsVisible[mEdgeNeighbours[ioff+j]]);

                if(bDraw) break;
            }
            if(bDraw){
				p1 = mPolygons[ioff+0]*3;
				p2 = mPolygons[ioff+1]*3;
				p3 = mPolygons[ioff+2]*3;
			    glVertex3f( *( vert+p1+0),*( vert+p1+1),*( vert+p1+2));
			    glVertex3f( *( vert+p2+0),*( vert+p2+1),*( vert+p2+2));
			    glVertex3f( *( vert+p3+0),*( vert+p3+1),*( vert+p3+2));
            }
        }
        ioff += mPolygonSize[i];
    }
    glEnd();

    glPolygonMode (GL_BACK, GL_FILL);
    glCullFace (GL_BACK);
    glDisable(GL_POLYGON_OFFSET_FILL);
    glLineWidth(1);
}


#define M_INFINITY  2
#define M_OFFSET      0.00

void    GL3DObject::RenderShadowPass(int pass, const Vector3& light){

    if(pass==0){
        if(!bShadowReady){
            CalcPlanes();
            CalcNeighbours();
            bShadowReady = true;
        }

        CheckVisibility(light);

        Vector vlight(3);
        vlight.Set(light.Array(),3);
        vlight *= -M_INFINITY;
        mVertices.Mult(1.0+M_INFINITY,mShadowVertices);
        mShadowVertices.SAddToRow(vlight);
    }


    int p1,p2,jj;
    Vector3 v1,v2,v1n,v2n;
    int ioff = 0;
    double *vert = mVertices.Array();
    double *svert = mShadowVertices.Array();

    for(int i=0;i<mNbPolygons;i++){
        if(mIsVisible[i]){
            for(int j=0;j<mPolygonSize[i];j++){
                bool bDraw = (mEdgeNeighbours[ioff+j]<0);
                if(!bDraw) bDraw = !(mIsVisible[mEdgeNeighbours[ioff+j]]);
                if(bDraw){
					// here we have an edge, we must draw a polygon
					p1 = mPolygons[ioff+j]*3;
					jj = (j+1)%mPolygonSize[i];
					p2 = mPolygons[ioff+jj]*3;

					//draw the polygon
					glBegin(GL_TRIANGLE_STRIP);
						glVertex3f( *( vert+p1+0),*( vert+p1+1),*( vert+p1+2));
						glVertex3f( *(svert+p1+0),*(svert+p1+1),*(svert+p1+2));
						glVertex3f( *( vert+p2+0),*( vert+p2+1),*( vert+p2+2));
						glVertex3f( *(svert+p2+0),*(svert+p2+1),*(svert+p2+2));
					glEnd();
                }
            }
        }else{

			glBegin(GL_TRIANGLE_FAN);
                for(int k=0;k<mPolygonSize[i];k++){
					p1 = mPolygons[ioff+k]*3;
				    glVertex3f( *(svert+p1+0),*(svert+p1+1),*(svert+p1+2));
                }
			glEnd();

			glBegin(GL_TRIANGLE_FAN);
                for(int k=mPolygonSize[i]-1;k>=0;k--){
					p1 = mPolygons[ioff+k]*3;
					glVertex3f( *( vert+p1+0),*( vert+p1+1),*( vert+p1+2));
                }
			glEnd();
        }
        ioff += mPolygonSize[i];
    }

}

void    GL3DObject::SetShadow(GL3DObject* shadow){
    if(mShadowObject) delete mShadowObject;
    mShadowObject = shadow;
}

GL3DObject*    GL3DObject::Clone(GL3DObject* res){
    if(res==NULL)
        res = new GL3DObject();

    Base3DObject::Clone(res);

    return res;
}












void    GL3DObject::RenderShadowInit(){
    glClear(GL_STENCIL_BUFFER_BIT);
	glDisable(GL_LIGHTING);
    glDepthMask(GL_FALSE);
    glDepthFunc(GL_LEQUAL);

    glEnable(GL_STENCIL_TEST);
    glColorMask(0, 0, 0, 0);
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(0.0f, 2.0f);
}

void    GL3DObject::RenderShadowInitPass(int pass){

    int passId = pass%2;

    if(passId==0){
	    //glClear(GL_STENCIL_BUFFER_BIT);

        if(pass<2){
    	    glStencilFunc(GL_ALWAYS, 0x1, 0xFF);
            //glStencilMask(0xF);
	        glFrontFace(GL_CCW);
            glStencilOp(GL_KEEP, GL_KEEP, GL_INCR);
        }else{
    	    glStencilFunc(GL_ALWAYS, 1, 0xFF);
            //glStencilMask(0x0F);
    	    glFrontFace(GL_CW);
            glStencilOp(GL_KEEP, GL_INCR, GL_KEEP);
        }
    }else{
        if(pass<2){
    	    glStencilFunc(GL_ALWAYS, 0x1, 0xFF);
            //glStencilMask(0xF);
	        glFrontFace(GL_CW);
            glStencilOp(GL_KEEP, GL_KEEP, GL_DECR);
        }else{
    	    glStencilFunc(GL_ALWAYS, 1, 0xFF);
            //glStencilMask(0x0F);
    	    glFrontFace(GL_CCW);
            glStencilOp(GL_KEEP, GL_DECR, GL_KEEP);
        }

    }
}


void    GL3DObject::RenderShadowEnd(){
    glDisable(GL_POLYGON_OFFSET_FILL);

	glFrontFace(GL_CCW);
	glColorMask(1, 1, 1, 1);

	//draw a shadowing rectangle covering the entire screen
	glColor4f(0.0f, 0.0f, 0.0f, 0.5f);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glStencilFunc(GL_NOTEQUAL, 0, 0xffffffff);
    glStencilMask(0xffffffff);
	glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);
	glPushMatrix();
	glLoadIdentity();
	glBegin(GL_TRIANGLE_STRIP);
		glVertex3f(-0.1f, 0.1f,-0.10f);
		glVertex3f(-0.1f,-0.1f,-0.10f);
		glVertex3f( 0.1f, 0.1f,-0.10f);
		glVertex3f( 0.1f,-0.1f,-0.10f);
	glEnd();
	glPopMatrix();
//	glDisable(GL_BLEND);

	glDepthFunc(GL_LEQUAL);
	glDepthMask(GL_TRUE);
	glEnable(GL_LIGHTING);
	glDisable(GL_STENCIL_TEST);
	glShadeModel(GL_SMOOTH);
}


