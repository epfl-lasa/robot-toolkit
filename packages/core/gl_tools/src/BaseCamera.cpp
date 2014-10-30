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


#include "BaseCamera.h"

BaseCamera::BaseCamera(){
    Clear();
}
BaseCamera::~BaseCamera(){
}
void BaseCamera::Clear(){
    mRef.Identity();
    mWidth              = 0;
    mHeight             = 0;
    mNearPlane          = 0.01;
    mFarPlane           = 100.0;
    mPrincipalPointX    = 0;
    mPrincipalPointY    = 0;
    mFocalX             = 1.0;
    mFocalY             = 1.0;
}
/*
void BaseCamera::Apply(bool setIdentity){
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();

  glViewport(m_X, m_Y, m_Width, m_Height);

  if(m_Height == 0)
    m_Height = 1;

  float ratio = float(m_Width) / float(m_Height);

  if(m_ViewAngle>0.0)
    gluPerspective(m_ViewAngle/ratio,ratio,m_Near,m_Far);
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
  glMultMatrixf(m_ref.GetInverse().RowOrderForceFloat());
}
*/

void BaseCamera::SetViewport(int w, int h){
    mWidth   = w;
    mHeight  = h;
}


void BaseCamera::SetProjection( int im_width,
                                int im_height,
                                REALTYPE principal_x,
                                REALTYPE principal_y,
                                REALTYPE focal_x,
                                REALTYPE focal_y,
                                REALTYPE mnear,
                                REALTYPE mfar){
    mWidth              = im_width;
    mHeight             = im_height;
    mNearPlane          = mnear;
    mFarPlane           = mfar;
    mPrincipalPointX    = principal_x;
    mPrincipalPointY    = principal_y;
    mFocalX             = focal_x;
    mFocalY             = focal_y;
}

void BaseCamera::SetOrientation (const Matrix3 & orient){
    mRef.SetOrientation(orient);
}
void BaseCamera::SetPosition    (const Vector3 & pos){
    mRef.SetTranslation(pos);
}
void BaseCamera::SetPose        (const Matrix4 & pose){
    mRef.Set(pose);
}
/*
void BaseCamera::Hold(){
  m_hold      = true;
  m_holdRef=m_ref;
}

void BaseCamera::Accept(){
  m_hold = false;
}

void BaseCamera::Move(float dx, float dy, float dz){
  //if(m_hold)
  //  m_ref = m_holdRef;

  switch(m_Mode){
  case FreeMove:
    printf("Free\n");
    break;
  case Centered:
    float norm = m_position.Norm();
    Vector3 shift(dx,dy,0);
    shift *=norm /100.0f;
    Vector3 absShift;
    m_orient.Mult(shift,absShift);
    m_position += absShift;
    norm += dz/norm;
    norm = MAX(1.0f,fabs(norm));
    m_position *= norm/m_position.Norm();
    m_orient.SetColumn(m_position,2);
    m_orient.Normalize();
    break;
  }
}


void BaseCamera::Move(float dx, float dy, float dz, float ay, float ax,float az) {
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




void BaseCamera::Save(string fname){
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

void BaseCamera::Load(string fname){
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
