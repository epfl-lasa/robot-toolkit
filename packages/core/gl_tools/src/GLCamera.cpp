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

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
using namespace std;


#include "GLCamera.h"

GLCamera::GLCamera(){
    Clear();
    m_ref.SetOrigin(Vector3(0.0,0.0,-10.0));
    m_ref.SetOrient(Matrix3::SRotationX(DEG2RAD(90)));
    m_ref.Update();
}
void GLCamera::Clear(){
  m_lookAtPoint.Zero();
  m_lookAtPoint.z() = 1.0;
  m_position.Zero();
  m_orient.Identity();
  m_ref.Identity();
  m_hold       		= false;
  m_X          		= 0;
  m_Y          		= 0;
  m_Width      		= 100;
  m_Height     		= 100;
  m_Near       		= 0.01f;
  m_Far        		= 20.0f;
  m_ViewAngle  		= 60.0f;
  m_ImWidth    		= 100;
  m_ImHeight   		= 100;
  m_PrincipalX 		= 0;
  m_PrincipalY 		= 0;
  m_FocalX    		= 1;
  m_FocalY     		= 1;
  m_closeupDistance = 0.2;
  m_Mode       = CAMMODE_Centered;
}

void GLCamera::Apply(bool setIdentity){
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();

  glViewport(m_X, m_Y, m_Width, m_Height);

  if(m_Height == 0)
    m_Height = 1;

  float ratio = float(m_Width) / float(m_Height);

  if(m_ViewAngle>0.0){
    if(ratio>1.0)
        gluPerspective(m_ViewAngle/ratio,ratio,m_Near,m_Far);
    else
        gluPerspective(m_ViewAngle,ratio,m_Near,m_Far);
  }
  else{
    glFrustum(-m_Near *  m_PrincipalX               / m_FocalX,
               m_Near * (m_ImWidth - m_PrincipalX)  / m_FocalX,
               m_Near * (m_PrincipalY - m_ImHeight) / m_FocalY,
               m_Near *  m_PrincipalY               / m_FocalY,
               m_Near,  m_Far);
  }

  glMatrixMode(GL_MODELVIEW);
  if(setIdentity)
    glLoadIdentity();


  m_ref.SetOrigin(m_position);
  m_ref.SetOrient(m_orient);
  m_ref.Update();
  glMultMatrixf(m_ref.GetInverse().GetHMatrix().RowOrderForceFloat());
}


void GLCamera::SetViewport(int w, int h){
  m_Width   = w;
  m_Height  = h;
}

void GLCamera::SetViewport(int x, int y, int w, int h){
  m_X       = x;
  m_Y       = y;
  m_Width   = w;
  m_Height  = h;
}

void GLCamera::SetProjection(float viewAngle, float mnear, float mfar){
  m_ViewAngle = viewAngle;
  m_Near      = mnear;
  m_Far       = mfar;
}

void GLCamera::SetProjection(float im_width,
                             float im_height,
                             float principal_x,
                             float principal_y,
                             float focal_x,
                             float focal_y,
                             float mnear,
                             float mfar){
  m_PrincipalX = principal_x;
  m_PrincipalY = principal_y;
  m_FocalX     = focal_x;
  m_FocalY     = focal_y;
  m_Near       = mnear;
  m_Far        = mfar;
  m_ImWidth    = im_width;
  m_ImHeight   = im_height;

  m_ViewAngle  = 0;
}

void GLCamera::SetOrientation(const Matrix3 & orient){
  m_orient = orient;
}

void GLCamera::SetPosition(const Vector3 & pos){
  m_position = pos;
}
void GLCamera::SetLookAt(const Vector3 & lookat){
    m_lookAtPoint = lookat;
}

void GLCamera::Hold(){
  m_hold      = true;
  m_holdRef=m_ref;
}

void GLCamera::Accept(){
  m_hold = false;
}

void GLCamera::Move(float dx, float dy, float dz){
  //if(m_hold)
  //  m_ref = m_holdRef;
    Vector3 shift;
    Vector3 absShift;
    Vector3 relPos;
    Vector3 up(0,0,1);

    switch(m_Mode){
    case CAMMODE_FreeMove:

        shift.Set(0,0,dz);
        shift *= 1/50.0;
        m_orient.Mult(shift,absShift);

        relPos  = m_lookAtPoint - m_position;
        if((relPos+absShift).Dot(m_orient.GetColumn(2))<-1.0){
            m_lookAtPoint += absShift;
        }else{
            m_lookAtPoint = m_position - Vector3(relPos.Norm()*m_orient(0,2), relPos.Norm()*m_orient(1,2), relPos.Norm()*m_orient(2,2));
        }
        shift.Set(dx,dy,0);
        shift *= 1/50.0;
        m_orient.Mult(shift,absShift);
        m_lookAtPoint += absShift;

        relPos  = m_position-m_lookAtPoint;
        m_orient.SetColumn(relPos,2);
        m_orient.SetColumn(up,1);
        m_orient.Normalize();
        break;

    case CAMMODE_Centered:
        relPos  = m_position-m_lookAtPoint;
        float norm      = relPos.Norm();
        float origNorm  = norm;
        shift.Set(dx,dy,0);
        shift *= norm /100.0;
        m_orient.Mult(shift,absShift);
        relPos  += absShift;
        if(dz!=0.0){
            norm += 0.05*dz/norm;
            norm = MAX(m_closeupDistance,fabs(norm));
            relPos *= norm/origNorm;
        }else{
            relPos *= origNorm/relPos.Norm();
        }

        //m_position *= norm/origNorm;
        m_position = m_lookAtPoint + relPos;
        m_orient.SetColumn(relPos,2);
        m_orient.SetColumn(up,1);
        m_orient.Normalize();
        break;
    }
}

void GLCamera::GetRayVectors(int iMouseX, int iMouseY, Vector3 & vecRay1, Vector3 & vecRay2){
    Vector vec1(4), vec2(4), vecTmp;
    double pVal[4] = { 0.0, 0.0, 0.0, 1.0 };
    vec1.Set(pVal, 4);

    double centered_x     = iMouseX - m_Width/2;
    double centered_y     = (m_Height - iMouseY) - m_Height/2;
    double unit_x         = centered_x/(m_Width/2);
    double unit_y         = centered_y/(m_Height/2);
    double dAspect        = double( m_Width ) / double( m_Height );
    double dVerticalAngle = m_ViewAngle/2;

    if(dAspect>1.0)
        dVerticalAngle /= dAspect;

    double full_height = tan(dVerticalAngle * PI / 180.0);
    double pVal2[4] = { unit_x * full_height * dAspect, unit_y * full_height, -1.0, 1.0 };
    vec2.Set(pVal2, 4);

    // Now transform the points into world coordinates
    Matrix matTransform = m_ref.GetHMatrix();

    vecTmp.Set(matTransform.Mult(vec1));
    vecRay1.Set(vecTmp.At(0), vecTmp.At(1), vecTmp.At(2));
    //std::cout << "Origin: " << vecTmp.At(0) << ", " << vecTmp.At(1) << ", " << vecTmp.At(2) << ", " << vecTmp.At(3) << std::endl;

    vecTmp.Set(matTransform.Mult(vec2));
    vecRay2.Set(vecTmp.At(0), vecTmp.At(1), vecTmp.At(2));
    //std::cout << "Vector: " << vecTmp.At(0) << ", " << vecTmp.At(1) << ", " << vecTmp.At(2) << ", " << vecTmp.At(3) << std::endl;
}

void GLCamera::SetCameraMode(GLCamera::CameraMode mode){
    m_Mode = mode;
}
/*
void GLCamera::Move(float dx, float dy, float dz, float ay, float ax,float az) {
  if(m_hold)
    m_ref.Copy(&m_holdRef);

  CVector3_t tmpSrc,tmpDst;
  CMatrix3_t tmpInv;
  m_inverse(m_ref.m_orient,tmpInv);

  v_set(dx,dy,dz,tmpSrc);
  v_transform(tmpSrc,tmpInv,tmpDst);
  v_add(m_ref.m_origin,tmpSrc,m_ref.m_origin);

  CMatrix3_t tmp1,tmp2;

  m_rotation_x(-ax,tmp1);
  m_copy(m_ref.m_orient,tmp2);
  m_multiply(tmp1,tmp2,m_ref.m_orient);

  m_rotation_y(-ay,tmp1);
  m_copy(m_ref.m_orient,tmp2);
  m_multiply(tmp2,tmp1,m_ref.m_orient);

  m_rotation_z(-az,tmp1);
  m_copy(m_ref.m_orient,tmp2);
  m_multiply(tmp2,tmp1,m_ref.m_orient);

  m_ref.Update();
  glLoadIdentity();
  glMultMatrixf(*m_ref.GetRef());
}




void GLCamera::Save(string fname){
  ofstream f;
  f.open(fname.c_str());
  if(f.is_open()){
    f << m_ref.m_origin[0] << " "<< m_ref.m_origin[1] << " "<< m_ref.m_origin[2] << endl;
    f << m_ref.m_orient[ 0] << " "<< m_ref.m_orient[ 1] << " "<< m_ref.m_orient[ 2] << " "<<m_ref.m_orient[ 3] << endl;
    f << m_ref.m_orient[ 4] << " "<< m_ref.m_orient[ 5] << " "<< m_ref.m_orient[ 6] << " "<<m_ref.m_orient[ 7] << endl;
    f << m_ref.m_orient[ 8] << " "<< m_ref.m_orient[ 9] << " "<< m_ref.m_orient[10] << " "<<m_ref.m_orient[11] << endl;
    f << m_ref.m_orient[12] << " "<< m_ref.m_orient[13] << " "<< m_ref.m_orient[14] << " "<<m_ref.m_orient[15] << endl;
    f.close();
  }
}

void GLCamera::Load(string fname){
  ifstream f;
  f.open(fname.c_str());
  if(f.is_open()){
    m_ref.Identity();
    f >> m_ref.m_origin[0] >>  m_ref.m_origin[1] >>  m_ref.m_origin[2] ;
    f >> m_ref.m_orient[ 0] >>  m_ref.m_orient[ 1] >>  m_ref.m_orient[ 2] >> m_ref.m_orient[ 3] ;
    f >> m_ref.m_orient[ 4] >>  m_ref.m_orient[ 5] >>  m_ref.m_orient[ 6] >> m_ref.m_orient[ 7] ;
    f >> m_ref.m_orient[ 8] >>  m_ref.m_orient[ 9] >>  m_ref.m_orient[10] >> m_ref.m_orient[11] ;
    f >> m_ref.m_orient[12] >>  m_ref.m_orient[13] >>  m_ref.m_orient[14] >> m_ref.m_orient[15] ;
    f.close();
    m_ref.Update();
  }
}
*/
