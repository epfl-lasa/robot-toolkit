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

#ifndef BASE3DOBJECT_H_
#define BASE3DOBJECT_H_

#include "MathLib/MathLib.h"
using namespace MathLib;

#define BASE3DOBJ_FILE_MAX_TRIANGLES   3276800
#define BASE3DOBJ_FILE_MAX_VERTICES    3276800

class Base3DObject
{
protected:
    static  int      sObjCount;
    static  REALTYPE *sVerData;
    static  int      *sTriData;
    static  int      *sPolySize;
    static  int      *sPolyStart;

    static int        sBASE3DOBJ_FILE_MAX_TRIANGLES;
    static int        sBASE3DOBJ_FILE_MAX_VERTICES;

    static void       ResizeStaticVar();

public:
    int         mNbVertices;
    int         mNbPolygons;

    int         *mPolygonSize;
    int         *mPolygons;

    int         *mPolygonStart;

    Matrix      mVertices;
    Matrix      mNormals;
    bool        bNormalsPerVertex;

    Matrix      mNormalsPerVertex;


    // Shadows stuff
    Matrix      mPlanes;
    int         *mEdgeNeighbours;

    int         *mVertexNeighbours;
    int         *mVertexNeighbourSize;
    int         *mVertexNeighbourStart;

    int         *mIsVisible;

    bool        bOnlyTriangles;

public:
            Base3DObject();
    virtual ~Base3DObject();

    virtual void Free();

            bool  LoadFromObjFile(const char *filename, bool invNormals=false);
            bool  SaveToObjFile(const char *filename);
            void  CalcNormals(bool perPolyVertex = true, bool perVertex = false);
            void  ConvertToTriangles();
    virtual void  Render();
            void  AddOffset(const Vector3& offset);
            void  Transform(const Matrix3& trans);
            void  Transform(const Matrix4& trans);


            void  CalcPlanes();
            void  CalcNeighbours();
            void  CheckVisibility(const Vector3& point);


            void GenerateCylinder(int nbSlices, int nbStacks=1);
            void GenerateSphere(int nbSlices, int nbStacks);
            void GenerateCube();
            void GenerateCapsule(REALTYPE capRatio, int nbSlices, int nbStacks);

            void GenerateHeightField(const Matrix &heightData,REALTYPE sx,REALTYPE sy,REALTYPE sz);

			// Courtesy of M. Duvanel
            bool ComputeRayIntersection(Vector3 vecRay1, Vector3 vecRay2, const Matrix4 & objTransform, double & dFactor);

    virtual Base3DObject*   Clone(Base3DObject *res=NULL);
};


#endif /*Base3DOBJECT_H_*/
