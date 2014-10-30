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

#ifndef __GLTOOLS_H__
#define __GLTOOLS_H__
#ifdef WIN32
#include <windows.h>
#endif

#include <GL/gl.h>
#include <GL/glu.h>

#include "GLFont.h"
#include "MathLib/MathLib.h"
using namespace MathLib;
#include <vector>
using namespace std;

#ifdef LOW_GL
#define DEF_SLICES  3
#define DEF_STACKS  2
#else
#define DEF_SLICES  8
#define DEF_STACKS  8
#endif


class GLTools
{
protected:
  static float  m_red;
  static float  m_green;
  static float  m_blue;
  static float  m_alpha;

  static bool   m_outline;
  static bool   m_solid;

  static GLFont m_Font;

public:
  static void SetColor(float r, float g, float b, float a=1.0f);

  static void DrawOutline (bool state);
  static void DrawSolid   (bool state);

//  static void DrawArc   (float min, float max, Matrix4 & ref);
//  static void Draw3DArc (CVector3_List_t * vl, Matrix4 & ref);


  static void DrawCube        (float sideLength);
  static void DrawCone        (float radius, float height, int slices=DEF_SLICES);
  static void DrawSphere      (float radius,               int slices=DEF_SLICES, int stacks=DEF_STACKS, float radHeight = PI);
  static void DrawHalfSphere  (float radius,               int slices=DEF_SLICES, int stacks=DEF_STACKS);
  static void DrawCylinder    (float radius, float height, int slices=DEF_SLICES, int stacks=DEF_STACKS);
  static void DrawFullCylinder(float radius, float height, int slices=DEF_SLICES, int stacks=DEF_STACKS);

  static void DrawLines       (MathLib::Matrix &verticesList,int offset=0);

  static void DrawVector(const Vector3 &v, float radius = 1,const Matrix4 *ref=NULL);
  static void DrawSegment(const Vector3 &v,float radius = 1);
  static void DrawPlane (const Vector3 &v, const Matrix4 *ref=NULL);
  static void DisplayText(float x, float y, const char * text, float fontHeight = 1.0f,int maxLen = -1);
  static void DisplayText(int x, int y, const char * text, int fontHeight = 16,int maxLen = -1);

  static void DrawRef(float scale = 1,const Matrix4 *ref = NULL);


  protected:
    static int sDrawRefCallListId;
};

#define GLT GLTools
#endif
