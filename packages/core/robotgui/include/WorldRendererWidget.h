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

#ifndef __WorldRendererWidget_H__
#define __WorldRendererWidget_H__

#include <QObject>
#include <QGLWidget>
#include <QMouseEvent>
#include <QKeyEvent>
#include <QMutex>


#include "StdTools/XmlTree.h"
#include "StdTools/Timer.h"
#include "GLTools/GLTools.h"
#include "GLTools/GLCamera.h"
#include "MathLib/MathLib.h"

#include "AbstractRenderer.h"
#include "ObjectRenderer.h"
#include "RobotRenderer.h"

#include "SimulatorDynamics/SimulatorDynamics.h"
#include "SimulatorDynamics/SimulatorDynamicsInterface.h"
#include "RobotLib/World.h"

#include "FreeImage.h"


#define MAX_NUM_LIGHTS 8



class WorldRendererWidget : public QGLWidget
{
    Q_OBJECT
protected:

    int                         mGLWidth;
    int                         mGLHeight;


    Chrono                      mChrono;

    PerformanceEstimator        mRenderingTime;

    Chrono                      mFPSChrono;
    int                         mFPS;
    int                         mFPSCounter;




    QTimer                     *mIdleTimer;
    QTimer                     *myPaintQtimer;

    QMutex                      mRenderingMutex;

    vector<pAbstractRenderer>   mRenderers;


    GLCamera                    mCamera;
    Vector3                     mCameraInitPos;
    Vector3                     mCameraInitLookAt;


    Qt::MouseButton             mCurrentMouseButton;
    int                         mCurrentMouseX;
    int                         mCurrentMouseY;
    GLCamera::CameraMode        mCameraMode;


    char                        mCurrentKey;

    float                       mBackgroundColor[4];

    bool                        bUseFog;
    float                       mFogColor[4];
    float                       mFogNear, mFogFar;

    bool                        bUseFloor;
    float                       mFloorStepSize;
    float                       mFloorStepCount;
    float                       mFloorColor[2][4];
    int                         mFloorCallListId;
    double						mFloorHeight;
    int							mNumRobots;

    bool                        bUseGrid;
    int                         mGridStepCount;
    int                         mGridHeightStepCount;
    float                       mGridColor[4];
    int                         mGridCallListId;


    int                         mAntialiasingLevel;

    int                         mNumLights;
    float                       mLightColor[MAX_NUM_LIGHTS][3][4];
    bool                        bUseLightPosition[MAX_NUM_LIGHTS];
    float                       mLightPosition[MAX_NUM_LIGHTS][4];
    bool                        bUseLightDirection[MAX_NUM_LIGHTS];
    float                       mLightDirection[MAX_NUM_LIGHTS][4];

    int                         mColorMaterial;

    GL3DObject                  mLightObject;
    GL3DObject                  mCameraObject;
    GL3DObject                  mCameraObjectLine;

    World                      *mWorld;
    SimulatorDynamicsInterface             *mSimInterface;
    XmlTree                    *mConfigTree;

    bool                        bInitGL;

    bool                        bShowHelp;
    bool                        bShowStats;
    bool                        bShowCredits;
    bool                        bRenderShadows;
    bool                        bRenderOutlines;
    bool                        bRenderBBoxes;
    bool                        bRenderTransparency;
    bool                        bRenderCameraLookAt;
    bool						bRenderFrames;

    bool                        bIsFullScreen;

    bool                        bCtrlModifierOn;
    bool                        bShiftModifierOn;

    bool                        bSlowFrameRate;

    bool						bIsRecording;
    int 						mFrameNum;
    std::vector<BYTE*>			recorded_pixels;
    std::vector<int>			mGLWidthList;
    std::vector<int>			mGLHeightList;
    BYTE*						rec_one_frame;
    FIBITMAP* image;
    char*						tmp_filename;

public:
  WorldRendererWidget(pXmlTree config=NULL,QWidget *parent=NULL);
  ~WorldRendererWidget();


  void  AddRenderer(pAbstractRenderer renderer);
  void  ClearRenderer();

  void  SetWorld();

  void  SetSimulatorInterface(SimulatorDynamicsInterface* simInterface);

  void  SetProcessingPeriod(REALTYPE period);

  char  GetCurrentKey();

  void  LoadConfig(pXmlTree config);

  //void  SetInitCameraParams(const Vector3& pos,const Vector3& lookat);

  void  DrawFloor();
  void  DrawGrid();
  void  DrawCameraLookAt();
  void  DrawStats();
  void  DrawHelp();
  void  DrawCredits();
  void  DrawNoWorldMessage();

public slots:
  void  OnIdle();
  void  OnPaint();

protected:
  void InitGL();
  void initializeGL();
  void resizeGL(int w, int h);
  void paintGL();

public:
  virtual void keyPressEvent (QKeyEvent * event);
  virtual void keyReleaseEvent (QKeyEvent * event);
protected:
  virtual void mouseMoveEvent(QMouseEvent * event);
  virtual void mousePressEvent(QMouseEvent * event);
  virtual void mouseReleaseEvent(QMouseEvent * event);


};
#endif

