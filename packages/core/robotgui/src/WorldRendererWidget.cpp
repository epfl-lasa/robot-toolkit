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

#include "WorldRendererWidget.h"
#include "SimulatorDynamics/SimulatorDynamics.h"
#include "SimulatorDynamics/SimulatorDynamicsInterface.h"
#include <QTimer>
#include <QGLFormat>
#include <QMutexLocker>
#include <unistd.h>
#include <iostream>
using namespace std;


WorldRendererWidget::WorldRendererWidget(pXmlTree config, QWidget *parent)
:QGLWidget(parent) {
	bInitGL = true;
	mConfigTree = NULL;

	recorded_pixels.resize(1000);
	GLint m_viewport[4];
	glGetIntegerv( GL_VIEWPORT, m_viewport );
	rec_one_frame=NULL;
	tmp_filename = new char[25];

	//mConfigTree->Print();
	LoadConfig(config);

	setWindowTitle("RobotSimulator (CTRL+<h> for help)");

	QGLFormat desiredFormat(QGL::DoubleBuffer|
			QGL::DepthBuffer|
			QGL::Rgba|
			QGL::AlphaChannel|
			//QGL::AccumBuffer|
			QGL::StencilBuffer|
			//QGL::StereoBuffers|
			QGL::DirectRendering|
			//QGL::HasOverlay|
			QGL::SampleBuffers);
	//                            (mAntialiasingLevel>1?QGL::SampleBuffers:QGL::None));

	switch(mAntialiasingLevel){
	case 2: desiredFormat.setSamples(1); break;
	case 3: desiredFormat.setSamples(2); break;
	case 4: desiredFormat.setSamples(4); break;
	}
	setFormat(desiredFormat);


	mIdleTimer = new QTimer(this);
	connect(mIdleTimer, SIGNAL(timeout()), this, SLOT(OnIdle()));
	mIdleTimer->start(0);

	myPaintQtimer = new QTimer(this);
	connect(myPaintQtimer, SIGNAL(timeout()), this, SLOT(OnPaint()));
	myPaintQtimer->start(33);

	mCurrentKey = 0;

	mFloorCallListId    = 0;
	mGridCallListId     = 0;


	mLightObject.GenerateSphere(8,8);
	Matrix3 id;
	id.Identity();
	id*=0.1;
	mLightObject.Transform(id);

	mCameraObject.GenerateSphere(16,12);
	id*=0.5;
	mCameraObject.Transform(id);

	mCameraObjectLine.GenerateCube();
	//id.Identity();
	id(0,0)*=0.1;
	id(1,1)*=0.1;
	id(2,2)*=200;
	mCameraObjectLine.Transform(id);

	mRenderingTime.SetCount(30);

	mGLWidth    = 0;
	mGLHeight   = 0;

	mFPS        = 0;
	mFPSCounter = 0;
	mFPSChrono.Start();

	/*
    mPPSChrono.Start();
    mPPS        = 0;
    mPPSCounter = 0;
	 */

	/*
    mProcessingPeriod = 0.001;
    mRunTime          = 0.0;

    mRunChrono.Start();
    mRunChrono.Pause();

    bIsPausing      = false;
    mTimeToPause    = 0.0;
    bIsPaused       = true;
	 */

	bShowHelp       = false;
	bShowStats      = true;
	bShowCredits    = false;
	bRenderShadows  = false;
	bRenderOutlines = false;
	bRenderBBoxes   = false;
	bRenderTransparency = false;
	bRenderCameraLookAt = true;
	bCtrlModifierOn  = false;
	bShiftModifierOn = false;
	bRenderFrames = true;

	bIsFullScreen   = false;

	bIsRecording    = false;

	mSimInterface  = NULL;


}

WorldRendererWidget::~WorldRendererWidget(){
	if(mConfigTree!=NULL) delete mConfigTree;
	mConfigTree = NULL;

	ClearRenderer();
	delete mIdleTimer;
	delete myPaintQtimer;
}

void  WorldRendererWidget::AddRenderer(pAbstractRenderer renderer){
	if(renderer!=NULL)
		mRenderers.push_back(renderer);
}

void  WorldRendererWidget::ClearRenderer(){
	for(size_t i=0;i<mRenderers.size();i++)
		delete mRenderers[i];
	mRenderers.clear();
}

void  WorldRendererWidget::SetWorld(){
	QMutexLocker locker(&mRenderingMutex);
	ClearRenderer();
	if(mSimInterface!=NULL){
		mWorld = mSimInterface->GetWorld();
		if(mWorld !=NULL){
			bUseFloor = false;
			int objCnt = mWorld->GetObjectCount();

			WorldObject * object;
			//Checking for floor to determine the indexing of subsequent renderers
			for(int i=0;i<objCnt;i++)
			{
				object = mWorld->GetObject(i);
				if(!object->IsRobot())
				{
					pXmlTree floorCheck = object->GetConfigTree()->Find("BBoxShape.Shape");
					if((floorCheck)&&(floorCheck->GetData()=="Floor"))
						bUseFloor = true;
				}
			}

			//The second renderer after floor must be the robot's. This is to ensure almost "correct" behavior under transparency.
			mNumRobots=0;
			for(int i=0;i<objCnt;i++)
			{
				object = mWorld->GetObject(i);
				if(object->IsRobot())
				{
					RobotRenderer *renderer = new RobotRenderer();
					renderer->LinkToRobot(object->GetRobot());
					//					mConfigTree->Find("Robot")->Set("Transparency",true);
					renderer->Configure(mConfigTree->Find("Robot"));
					AddRenderer(renderer);
					mNumRobots++;
				}
			}

			//Renderers for everything else.
			for(int i=0;i<objCnt;i++){
				object = mWorld->GetObject(i);
				if(!object->IsRobot()){

					pXmlTree floorCheck = object->GetConfigTree()->Find("BBoxShape.Shape");
					if((floorCheck)&&(floorCheck->GetData()=="Floor")){
						bUseFloor = true;		//not needed. already done
					}else{
						ObjectRenderer *renderer = new ObjectRenderer();
						renderer->LinkToObject(object);
						renderer->Configure(mConfigTree->Find("Object"));
						AddRenderer(renderer);
					}
				}
			}
		}
	}

	/* TO CHECK
    bool crPaused = mRunChrono.IsPaused();
    mRunChrono.Start();
    if(crPaused) mRunChrono.Pause();
    mRunTime = mSimInterface->GetSimulationTime();;
	 */

}

void  WorldRendererWidget::SetSimulatorInterface(SimulatorDynamicsInterface* simInterface){
	mSimInterface     = simInterface;
	if(simInterface){
		SetWorld();
	}
}


/*
void  WorldRendererWidget::SetProcessingPeriod(REALTYPE period){
    mProcessingPeriod = period;
}
 */
void  WorldRendererWidget::OnIdle(){

	if(mSimInterface){
		if(!mSimInterface->IsRunning()){
			usleep(10000);
		}
		double maxTotalProcessingTime = (1.0 - (mRenderingTime.GetTime()* REALTYPE(mFPS)*1e-6))-0.05;
		mSimInterface->Process(maxTotalProcessingTime);
	}else{
		usleep(10000);
	}
	/*
    if(bIsPaused){

    }

    REALTYPE realTime = mRunChrono.ElapsedTimeUs()*1e-6;
    if(bIsPausing){
        if(realTime>=mTimeToPause){
            bIsPausing  = false;
            bIsPaused   = true;
            mRunChrono.Pause();
        }
    }

    mRunSpeed = 1.0;
    if(!bIsPaused){
        double maxTotalProcessingTime = (1.0 - (mRenderingTime.GetTime()* REALTYPE(mFPS)*1e-6))-0.05;

        int steps = 0;
        mChrono.Start();
        if(mRunSpeed>=0.0){
            if(realTime*mRunSpeed-mRunTime>0.0){
                steps = mSimInterface->SimulationStep(realTime,maxTotalProcessingTime);
            }
        }else{
            steps = mSimInterface->SimulationStep(-1,maxTotalProcessingTime);
        }
        double mctime = mChrono.ElapsedTimeUs();

        if(steps>0){
            mProcessingTime.AddMeasurement(REALTYPE(mctime/double(steps)));
            mRunTime = mSimInterface->GetSimulationTime();
            mPPSCounter+=steps;
        }
    }
    if(mPPSChrono.ElapsedTimeMs()>1000){
        mPPS        = MAX(0,mPPSCounter-1);
        mPPSCounter = 0;
        mPPSChrono.Start();
    }
	 */
}

void  WorldRendererWidget::OnPaint(){
	update();

	if(bIsRecording)
	{
#ifdef USE_V_MEM_REC
		rec_one_frame = new BYTE[ 3 * mGLWidth * mGLHeight];
#endif
		glReadPixels(0, 0, mGLWidth, mGLHeight, GL_BGR, GL_UNSIGNED_BYTE, rec_one_frame);
#ifdef USE_V_MEM_REC
		if(recorded_pixels.size() <= mFrameNum)
			recorded_pixels.resize(recorded_pixels.size()+1000);
		if(mGLWidthList.size() <= mFrameNum)
			mGLWidthList.resize(mGLWidthList.size()+1000);
		if(mGLHeightList.size() <= mFrameNum)
			mGLHeightList.resize(mGLHeightList.size()+1000);

		recorded_pixels[mFrameNum] = rec_one_frame;
		mGLWidthList[mFrameNum] = mGLWidth;
		mGLHeightList[mFrameNum++] = mGLHeight;

#else
		image = FreeImage_ConvertFromRawBits(rec_one_frame, mGLWidth, mGLHeight, 3 * mGLWidth, 24, FI_RGBA_RED_MASK, FI_RGBA_GREEN_MASK, FI_RGBA_BLUE_MASK, false);
		sprintf(tmp_filename,"data/Misc/frame_%06d.bmp",mFrameNum++);
		FreeImage_Save(FIF_BMP, image, tmp_filename, 0);
		FreeImage_Unload(image);
#endif


	}
}
void WorldRendererWidget::initializeGL(){
	InitGL();
}
void WorldRendererWidget::InitGL(){

	glEnable(GL_DEPTH_TEST);

	for(int i=0;i<mNumLights;i++){
		glLightfv(GL_LIGHT0+i, GL_AMBIENT,  mLightColor[i][0]);
		glLightfv(GL_LIGHT0+i, GL_DIFFUSE,  mLightColor[i][1]);
		glLightfv(GL_LIGHT0+i, GL_SPECULAR, mLightColor[i][2]);
		glEnable(GL_LIGHT0+i);
	}
	for(int i=mNumLights;i<MAX_NUM_LIGHTS;i++){
		glDisable(GL_LIGHT0+i);
	}
	if(mNumLights>0)
		glEnable(GL_LIGHTING);


	switch(mColorMaterial){
	case 0: glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT);             break;
	case 1: glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);             break;
	case 2: glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE); break;
	case 3: glColorMaterial(GL_FRONT_AND_BACK, GL_SPECULAR);            break;
	case 4: glColorMaterial(GL_FRONT_AND_BACK, GL_EMISSION);            break;
	}
	glEnable(GL_COLOR_MATERIAL);


	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);
	glFrontFace(GL_CCW);
	glEnable(GL_DEPTH_TEST);
	glDepthMask(GL_TRUE);


	if(mAntialiasingLevel == 1){
		glEnable(GL_LINE_SMOOTH);
		glHint (GL_LINE_SMOOTH_HINT, GL_NICEST);
		glEnable(GL_POLYGON_SMOOTH);
		glHint (GL_POLYGON_SMOOTH_HINT, GL_NICEST);
	}
	glHint (GL_LINE_SMOOTH_HINT, GL_NICEST);
	glEnable (GL_LINE_SMOOTH);

	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_BLEND);
	glShadeModel(GL_SMOOTH);

	glEnable(GL_TEXTURE_2D);

	glEnable(GL_NORMALIZE);
	glClearColor(mBackgroundColor[0],mBackgroundColor[1],mBackgroundColor[2],mBackgroundColor[3]);

	if(bUseFog){
		glFogi(GL_FOG_MODE,GL_LINEAR);//GL_EXP; // GL_EXP2,
		glFogfv(GL_FOG_COLOR,mFogColor);
		glFogf(GL_FOG_DENSITY,0.01);
		glHint(GL_FOG_HINT,GL_DONT_CARE);
		glFogf(GL_FOG_START,mFogNear);
		glFogf(GL_FOG_END,mFogFar);
		glEnable(GL_FOG);
	}


	float zero[]={0.0,0.0,0.0,1.0};
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT,zero);

	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);	// Really Nice Perspective Calculations



	mCurrentMouseButton = Qt::NoButton;
	mCurrentMouseX=0;
	mCurrentMouseY=0;
	mCamera.SetPosition(mCameraInitPos);
	mCamera.SetLookAt(mCameraInitLookAt);
	mCamera.Move(0,0,0);
	mCamera.Apply();

	bInitGL =false;
}

void WorldRendererWidget::resizeGL(int w, int h){
	mGLWidth    = w;
	mGLHeight   = h;

	rec_one_frame = new BYTE[ 3 * mGLWidth * mGLHeight];

	mCamera.SetViewport(w,h);
	mCamera.Apply();

	//mBaseWindow.BaseResize(w,h);
}

void WorldRendererWidget::paintGL(){
	QMutexLocker locker(&mRenderingMutex);

	if(bInitGL){
		InitGL();
	}

	mChrono.Start();

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

	glPushMatrix();
	glLoadIdentity();
	mCamera.Apply(false);

	for(int i=0;i<mNumLights;i++){
		if(bUseLightPosition[i]){
			glLightfv(GL_LIGHT0+i,GL_POSITION,mLightPosition[i]);
		}
		if(bUseLightDirection[i]){
			glLightfv(GL_LIGHT0+i,GL_SPOT_DIRECTION,mLightDirection[i]);
			glLightf(GL_LIGHT0+i,GL_SPOT_CUTOFF,30.0);
		}
	}
	glDisable(GL_LIGHTING);
	for(int i=0;i<mNumLights;i++){
		glColor3f(1,1,0);
		glPushMatrix();
		glTranslatef(mLightPosition[i][0],mLightPosition[i][1],mLightPosition[i][2]);
		mLightObject.Render();
		glPopMatrix();
	}
	glEnable(GL_LIGHTING);




	if(bUseFloor)
		DrawFloor();

	if(bUseGrid)
		DrawGrid();

	for(unsigned int i=0;i<mRenderers.size();i++){
		mRenderers[i]->Render();
	}

	if(bRenderCameraLookAt &&  (mCurrentMouseButton != Qt::NoButton))
		DrawCameraLookAt();

	if(bRenderOutlines){
		for(unsigned int i=0;i<mRenderers.size();i++){
			mRenderers[i]->RenderOutline(mCamera.m_position);
		}
	}
	if(bRenderBBoxes){
		for(unsigned int i=0;i<mRenderers.size();i++){
			mRenderers[i]->RenderBoundingBox();
		}
	}
	if(bRenderShadows){
		for(int j=0;j<mNumLights;j++){
			Vector3 lp(mLightPosition[j][0],mLightPosition[j][1],mLightPosition[j][2]);

			Vector3 obs = mCamera.m_position;

			if(bUseLightPosition[j]){
				GL3DObject::RenderShadowInit();
				for(unsigned int i=0;i<mRenderers.size();i++){
					for(int pass=0;pass<2;pass++){
						mRenderers[i]->RenderShadow(pass,lp,mCamera.m_position);
					}
				}
				GL3DObject::RenderShadowEnd();
			}
		}
	}

	if((mSimInterface)&&(mSimInterface->GetSystemState()==ModuleInterface::SYSSTATE_STARTED))
		mSimInterface->Draw();

	glPopMatrix();
	//mBaseWindow.BaseRender();


	if(bShowHelp){
		if((mWorld==NULL)||(mWorld->IsEmpty())){
			DrawNoWorldMessage();
		}
		DrawHelp();
	}else{
		if(bShowCredits){
			DrawCredits();
		}else if(bShowStats){
			DrawStats();
		}
		if((mWorld==NULL)||(mWorld->IsEmpty())){
			DrawNoWorldMessage();
		}
	}


	mRenderingTime.AddMeasurement(REALTYPE(mChrono.ElapsedTimeUs()));

	mFPSCounter++;
	if(mFPSChrono.ElapsedTimeMs()>1000){
		mFPS        = MAX(0,mFPSCounter-1);
		mFPSCounter = 0;
		mFPSChrono.Start();
	}

}

void  WorldRendererWidget::DrawCredits(){
	int fontSize = 14;

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	gluOrtho2D(0, mGLWidth/float(fontSize), mGLHeight/float(fontSize), 0);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_LIGHTING);
	glDisable(GL_CULL_FACE);

	glColor4f(0,0,0,0.7);
	glBegin(GL_QUADS);
	glVertex2f(0,0);
	glVertex2f(mGLWidth/float(fontSize),0);
	glVertex2f(mGLWidth/float(fontSize),mGLHeight/float(fontSize));
	glVertex2f(0,mGLHeight/float(fontSize));
	glEnd();

	char txt[256];

	sprintf(txt,"RobotToolKit version 0.8 (c) 2011");                                     GLT::DisplayText(0, 0,txt,1);
	sprintf(txt,"  (or yet another robot toolkit)");                                      GLT::DisplayText(0, 1,txt,1);

	sprintf(txt,"Autor: Eric Sauser"                                                  );  GLT::DisplayText(0, 3,txt,1);
	sprintf(txt,"  email: eric.sauser@a3.epfl.ch"                                     );  GLT::DisplayText(0, 4,txt,1);
	sprintf(txt,"Learning Algorithms and Systems Laboratory (LASA)"                   );  GLT::DisplayText(0, 6,txt,1);
	sprintf(txt,"Swiss Federal Institute of Technology Lausanne (EPFL)"               );  GLT::DisplayText(0, 7,txt,1);


	sprintf(txt,"Credits:"                                                            );  GLT::DisplayText(0, 9,txt,1);
	sprintf(txt,"  Physics simulation engine:"                                        );  GLT::DisplayText(0,10,txt,1);
	sprintf(txt,"    Open Dynamic Engine (ODE v0.11.1) for simulation"                );  GLT::DisplayText(0,11,txt,1);
	sprintf(txt,"      www.ode.org"                                                   );  GLT::DisplayText(0,12,txt,1);
	sprintf(txt,"    Bullet Physics Library (v2.76) for collision detection"          );  GLT::DisplayText(0,13,txt,1);
	sprintf(txt,"      www.bulletphysics.org"                                         );  GLT::DisplayText(0,14,txt,1);


	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);
	glEnable(GL_CULL_FACE);
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
}

void  WorldRendererWidget::DrawStats(){
	int fontSize = 12;

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	gluOrtho2D(0, mGLWidth/float(fontSize), mGLHeight/float(fontSize), 0);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_LIGHTING);
	glDisable(GL_CULL_FACE);

	char txt[256];

	if(mSimInterface){
		sprintf(txt,"RunTime:  %1.1f (%1.1f)",mSimInterface->GetSimulationTime(),mSimInterface->GetSimulationRunTime());
		GLT::DisplayText(0,0,txt,1);
	}
	sprintf(txt,"FPS: %d",mFPS);
	GLT::DisplayText(0,1,txt,1);
	if(mSimInterface){
		sprintf(txt,"PPS: %d",mSimInterface->GetPPS());
		GLT::DisplayText(0,2,txt,1);
	}
	sprintf(txt,"Timers:");
	GLT::DisplayText(0,4,txt,1);

	sprintf(txt," Rendering:  %1.0fus (%1.2fs)",mRenderingTime.GetTime(),mRenderingTime.GetTime()*mFPS*1e-6);
	GLT::DisplayText(0,5,txt,1);

	if(mSimInterface){
		sprintf(txt," Processing: %1.0fus (%1.2fs)",mSimInterface->GetPTime(),mSimInterface->GetPTime()*mSimInterface->GetPPS()*1e-6);
		GLT::DisplayText(0,6,txt,1);
	}
	if(mWorld!=NULL){
		sprintf(txt,"Modules:");
		GLT::DisplayText(0,8,txt,1);
		int cpos = 9;

		const vector<WorldInterface*> & winterfaces = mWorld->GetWorldInterfaces();
		for(int i=0;i<int(winterfaces.size());i++){
			sprintf(txt," %s: %1.0f - %1.0f us",
					winterfaces[i]->GetInterfaceName().c_str(),
					winterfaces[i]->GetPerformanceEstimatorUpdateCore().GetTime()*1e6,
					winterfaces[i]->GetPerformanceEstimatorUpdate().GetTime()*1e6);
			GLT::DisplayText(0,cpos++,txt,1);
		}
		const vector<Robot*> & wrobots = mWorld->GetRobots();
		for(int j=0;j<int(wrobots.size());j++){
			const vector<RobotInterface*> & rinterfaces = wrobots[j]->GetRobotInterfaces();
			for(int i=0;i<int(rinterfaces.size());i++){
				sprintf(txt," %s: %1.0f - %1.0f us",
						rinterfaces[i]->GetInterfaceName().c_str(),
						rinterfaces[i]->GetPerformanceEstimatorUpdateCore().GetTime()*1e6,
						rinterfaces[i]->GetPerformanceEstimatorUpdate().GetTime()*1e6);
				GLT::DisplayText(0,cpos++,txt,1);
			}
		}
	}
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);
	glEnable(GL_CULL_FACE);
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
}


void  WorldRendererWidget::DrawNoWorldMessage(){
	int fontSize = 14;

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	float sx = mGLWidth/float(fontSize);
	float sy = mGLHeight/float(fontSize);
	gluOrtho2D(0, mGLWidth/float(fontSize), mGLHeight/float(fontSize), 0);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_LIGHTING);
	glDisable(GL_CULL_FACE);

	glColor4f(0,0,0,0.5);
	glBegin(GL_QUADS);
	glVertex2f(0,0);
	glVertex2f(mGLWidth/float(fontSize),0);
	glVertex2f(mGLWidth/float(fontSize),mGLHeight/float(fontSize));
	glVertex2f(0,mGLHeight/float(fontSize));
	glEnd();

	char txt[256];

	sprintf(txt,"<Warning: No world loaded>");
	GLT::DisplayText((sx-float(strlen(txt))*0.5f)*0.5f-1.0f, sy*0.5f,txt,1.0f);

	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);
	glEnable(GL_CULL_FACE);
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
}
void  WorldRendererWidget::DrawHelp(){
	int fontSize = 14;

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	gluOrtho2D(0, mGLWidth/float(fontSize), mGLHeight/float(fontSize), 0);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_LIGHTING);
	glDisable(GL_CULL_FACE);

	glColor4f(0,0,0,0.7);
	glBegin(GL_QUADS);
	glVertex2f(0,0);
	glVertex2f(mGLWidth/float(fontSize),0);
	glVertex2f(mGLWidth/float(fontSize),mGLHeight/float(fontSize));
	glVertex2f(0,mGLHeight/float(fontSize));
	glEnd();

	char txt[256];

	sprintf(txt,"Help: All commands below must be used with CTRL+<key>:");
	GLT::DisplayText(0,0,txt,1);

	sprintf(txt,"<key>   : <action>"                                                );  GLT::DisplayText(0, 2,txt,1);
	sprintf(txt,"-------------------------------------------"                       );  GLT::DisplayText(0, 3,txt,1);
	sprintf(txt,"h       : this help"                                               );  GLT::DisplayText(0, 4,txt,1);
	sprintf(txt,"SPACE   : run/pause simulation"                                    );  GLT::DisplayText(0, 5,txt,1);
	sprintf(txt,"s       : run a step of 0.1 sec"                                   );  GLT::DisplayText(0, 6,txt,1);
	sprintf(txt,"r       : reset the world"                                         );  GLT::DisplayText(0, 7,txt,1);
	sprintf(txt,"w       : reset the world but the robot"                           );  GLT::DisplayText(0, 8,txt,1);

	sprintf(txt,"v       : toggle video recording state"                            );  GLT::DisplayText(0, 9,txt,1);

	sprintf(txt,"1       : toggle stats"                                            );  GLT::DisplayText(0,11,txt,1);
	sprintf(txt,"2       : toggle shadows"                                          );  GLT::DisplayText(0,12,txt,1);
	sprintf(txt,"3       : toggle outlines"                                         );  GLT::DisplayText(0,13,txt,1);
	sprintf(txt,"4       : toggle bounding boxes"                                   );  GLT::DisplayText(0,14,txt,1);
	sprintf(txt,"5       : toggle transparency"                                     );  GLT::DisplayText(0,15,txt,1);
	sprintf(txt,"6       : toggle camera look at point"                             );  GLT::DisplayText(0,16,txt,1);
	sprintf(txt,"7       : toggle reference frames"                                 );  GLT::DisplayText(0,17,txt,1);
	sprintf(txt,"f       : toggle slow frame rate"                                  );  GLT::DisplayText(0,18,txt,1);
	sprintf(txt,"c       : credits"                                                 );  GLT::DisplayText(0,19,txt,1);


	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);
	glEnable(GL_CULL_FACE);
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
}


void WorldRendererWidget::keyPressEvent ( QKeyEvent * event ){
	if(event->modifiers() & Qt::ShiftModifier){
		bShiftModifierOn = true;
	}else{
		bShiftModifierOn = false;
	}

	if(event->modifiers() & Qt::ControlModifier){
		bCtrlModifierOn = true;

		switch(event->key()){
		case Qt::Key_Space:
			if(mSimInterface){
				if(mSimInterface->IsRunning()){
					mSimInterface->Pause();
				}else{
					mSimInterface->Run();
				}
			}
			/*
            bIsPaused = !bIsPaused;
            if(bIsPaused)
                mRunChrono.Pause();
            else
                mRunChrono.Resume();
            bIsPausing = false;
			 */
			break;
		case Qt::Key_S:
			if(mSimInterface){
				mSimInterface->Step(0.1);
			}
			/*
            mTimeToPause = (mRunChrono.ElapsedTimeMs()+50)*1e-3;
            mRunChrono.Resume();
            bIsPaused   = false;
            bIsPausing  = true;
            //mRunChrono.Pause();
			 */
			break;

		case Qt::Key_V:
			if(bIsRecording)
			{
				bIsRecording = false;
				cout<<"Done"<<endl;



#ifdef USE_V_MEM_REC
				system("rm -f data/Misc/movie.avi");
				system("rm -f data/Misc/frame_*.bmp");
				cout<<endl<<"Converting pixel data to bmp...."<<endl;
				fflush(stdout);
				int totalFrames = mFrameNum;

				for(int i=0;i<totalFrames;i++)
				{
					image = FreeImage_ConvertFromRawBits(recorded_pixels[i], mGLWidthList[i], mGLHeightList[i], 3 * mGLWidthList[i], 24, FI_RGBA_RED_MASK, FI_RGBA_GREEN_MASK, FI_RGBA_BLUE_MASK, false);
					sprintf(tmp_filename,"data/Misc/frame_%06d.bmp",i);
					FreeImage_Save(FIF_BMP, image, tmp_filename, 0);
					FreeImage_Unload(image);
				}
				for (int i=0;i<totalFrames;i++)
					delete recorded_pixels[i];
				recorded_pixels.clear();
				mGLWidthList.clear();
				mGLHeightList.clear();
#endif
				system("rm -f data/Misc/movie.avi");
				cout<<endl<<"Encoding movie...."<<endl;
				fflush(stdout);
				if(system("ffmpeg -f image2 -i data/Misc/frame_%06d.bmp -b 12000 -sameq data/Misc/movie.avi"))
				{
					cout<<"***************************** ENCODING FAILED *********************************\n";
					cout<<"ffmpeg not found....please install it by typing : <sudo apt-get install ffmpeg>\n";
					cout<<"*******************************************************************************\n";
				}
				system("rm -f data/Misc/frame_*.bmp");



				cout<<"Done."<<endl;


			}
			else
			{
				cout<<endl<<"Capturing video....";

				fflush(stdout);
				bIsRecording = true;
				mFrameNum = 0;
			}

			break;

		case Qt::Key_H:
			bShowHelp = !bShowHelp;
			break;
		case Qt::Key_C:
			bShowCredits = !bShowCredits;
			break;
		case Qt::Key_F:
			bSlowFrameRate = !bSlowFrameRate;
			if(bSlowFrameRate){
				myPaintQtimer->setInterval(200);
			}else{
				myPaintQtimer->setInterval(33);
			}
			break;

		case Qt::Key_R:
			if(mSimInterface){
				mSimInterface->mSimulatorDynamics.AllowRobotReset(true);
				mSimInterface->Stop();
				mSimInterface->Start();
			}
			break;
		case Qt::Key_W:
			if(mSimInterface){
				mSimInterface->mSimulatorDynamics.AllowRobotReset(false);
				mSimInterface->Stop();
				mSimInterface->Start();
			}
			break;
		case Qt::Key_1:
			bShowStats      = !bShowStats;
			break;
		case Qt::Key_2:
			bRenderShadows      = !bRenderShadows;
			break;
		case Qt::Key_3:
			bRenderOutlines      = !bRenderOutlines;
			break;
		case Qt::Key_4:
			bRenderBBoxes      = !bRenderBBoxes;
			break;
		case Qt::Key_5:

			bRenderTransparency = !bRenderTransparency;
			if(mWorld!=NULL){


				for(unsigned int i=0;i<mRenderers.size();i++){
					if(i < mNumRobots){
						if(mConfigTree->Find("Robot"))
						{

							if(mConfigTree->Find("Robot")->Get("Transparency",true))
							{
								mConfigTree->Find("Robot")->Set("CurrTransparency",bRenderTransparency);
								mRenderers[i]->Configure(mConfigTree->Find("Robot"));

							}

							//                            cout<<"going for orobot2 "<<i<<endl;
							//                            mRenderers[curr_robot_index+1]->Configure(mConfigTree->Find("Robot"));
						}
					}else{

						if(mConfigTree->Find("Object"))
						{
							if(mConfigTree->Find("Object")->Get("Transparency",true))
							{
								mConfigTree->Find("Object")->Set("CurrTransparency",bRenderTransparency);
								mRenderers[i]->Configure(mConfigTree->Find("Object"));
							}
						}

					}
				}
			}

			break;
		case Qt::Key_6:
			bRenderCameraLookAt = ! bRenderCameraLookAt;
			break;
		case Qt::Key_7:
			bRenderFrames = ! bRenderFrames;
			//        	     for(int i=0;i<objCnt;i++)
			//        	     {
			//        	    	 WorldObject *object = mWorld->GetObject(i);
			//        	    	 if(!object->IsRobot())
			//        	    	 {
			//        	    		 if(mConfigTree->Find("Object"))
			//        	    		 {
			//        	    			 if(bRenderFrames)
			//        	    				 mConfigTree->Find("Object")->Set("RefSize",0.02);
			//        	    			 else
			//        	    				 mConfigTree->Find("Object")->Set("RefSize",0.0);
			//        	    		 }
			//
			//        	    		 mRenderers[i]->Configure(mConfigTree->Find("Object"));
			//        	    	 }
			//        	     }

			for(unsigned int i=0;i<mRenderers.size();i++){
				if(i < mNumRobots){
					if(bRenderFrames)
						mConfigTree->Find("Robot")->Set("RefSize",0.04);
					else
						mConfigTree->Find("Robot")->Set("RefSize",0.0);

					mRenderers[i]->Configure(mConfigTree->Find("Robot"));
				}
				else
				{

					if(bRenderFrames)
						mConfigTree->Find("Object")->Set("RefSize",0.04);
					else
						mConfigTree->Find("Object")->Set("RefSize",0.0);

					mRenderers[i]->Configure(mConfigTree->Find("Object"));
				}
			}


			break;
		}
	}else{
		bCtrlModifierOn = false;
		//QGLWidget::keyPressEvent(event);
	}


	//QGLWidget::keyPressEvent(event);
}

char WorldRendererWidget::GetCurrentKey(){
	char res = mCurrentKey;
	mCurrentKey = 0;
	return res;
}

void WorldRendererWidget::keyReleaseEvent (QKeyEvent * event){
	if(event->modifiers() & Qt::ShiftModifier){
		bShiftModifierOn = true;
	}else{
		bShiftModifierOn = false;
	}

	if(event->modifiers() & Qt::ControlModifier){
		bCtrlModifierOn = true;
	}else{
		bCtrlModifierOn = false;
	}
	//QGLWidget::keyReleaseEvent(event);
}

void WorldRendererWidget::mouseMoveEvent(QMouseEvent * event){
	//mBaseWindow.BaseInputMouseMotion(event->x(), event->y());

	if(!bShiftModifierOn){
		if(mCurrentMouseButton == Qt::LeftButton){
			mCamera.Move((float)(event->x()-mCurrentMouseX),-(float)(event->y()-mCurrentMouseY),0);
			mCurrentMouseX = event->x();
			mCurrentMouseY = event->y();
		}
		if(mCurrentMouseButton == Qt::RightButton){
			mCamera.Move(0,0,(float)(event->y()-mCurrentMouseY));
			mCurrentMouseX = event->x();
			mCurrentMouseY = event->y();
		}
	}
}

#define GLBW_BTNUP      0
#define GLBW_BTNDOWN    1
#define GLBW_LEFTBTN    0
#define GLBW_RIGHTBTN   1

void WorldRendererWidget::mousePressEvent(QMouseEvent * event){
	/*
  int res = 0;
  if(event->button() == Qt::LeftButton)
    res = mBaseWindow.BaseInputMouseButton(GLBW_LEFTBTN, GLBW_BTNDOWN, event->x(), event->y());
  if(event->button() == Qt::RightButton)
    res = mBaseWindow.BaseInputMouseButton(GLBW_RIGHTBTN, GLBW_BTNDOWN, event->x(), event->y());

  if(res) return;
	 */

	if(mCurrentMouseButton == Qt::NoButton){
		if(!bShiftModifierOn){
			if(bCtrlModifierOn){
				mCameraMode = GLCamera::CAMMODE_FreeMove;
			}else{
				mCameraMode = GLCamera::CAMMODE_Centered;
			}
			mCamera.SetCameraMode(mCameraMode);

			mCurrentMouseButton = event->button();
			mCurrentMouseX      = event->x();
			mCurrentMouseY      = event->y();
		}
	}

	// Duduche's mods in this if...
	if(bShiftModifierOn && (!bCtrlModifierOn))
	{
		bool bIntersects = (event->button() == Qt::RightButton);

		// We get the ray corresponding to the user's click
		Vector3 vecRay1, vecRay2;
		mCamera.GetRayVectors(event->x(), event->y(), vecRay1, vecRay2);

		//std::cout << "Clicked coordinates are " << mCurrentMouseX << ", " << mCurrentMouseY;
		//std::cout << ", button clicked is " << ( bIntersects ? "Right" : "Left" ) << std::endl;
		//std::cout << "Ray vectors are " << vecRay1.x() << ", " << vecRay1.y() << ", " << vecRay1.z();
		//std::cout << " and " << vecRay2.x() << ", " << vecRay2.y() << ", " << vecRay2.z() << std::endl;

		// Now let's find which object the user clicked
		double dFactor, dSmallestFactor = R_INFINITY;
		unsigned uClickedObjectIndex;
		unsigned uClickedShapeIndexTmp, uClickedShapeIndex;
		for(unsigned int i=0;i<mRenderers.size();i++){
			if(mRenderers[i]->ComputeRayIntersection(vecRay1, vecRay2, dFactor, bIntersects, &uClickedShapeIndexTmp)){
				if(dFactor<dSmallestFactor){
					dSmallestFactor = dFactor;
					uClickedObjectIndex = i;
					uClickedShapeIndex = uClickedShapeIndexTmp;
					//std::cout << "Validating intersection, renderer index = " << uClickedObjectIndex << std::endl;
				}
			}
		}

		if(dSmallestFactor != R_INFINITY){
			//std::cout << "Intersection found, index " << uClickedObjectIndex << std::endl;

			if(!bIntersects){
				double dSphereScale = 0.005;
				mRenderers[uClickedObjectIndex]->AddShapeFromRay(uClickedShapeIndex, vecRay1, vecRay2, dSmallestFactor, dSphereScale);
			}else{
				mRenderers[uClickedObjectIndex]->RemoveShape(uClickedShapeIndex);
			}
		}
	}

}
void WorldRendererWidget::mouseReleaseEvent(QMouseEvent * event){
	/*
  int res = 0;
  if(event->button() == Qt::LeftButton)
    res = mBaseWindow.BaseInputMouseButton(GLBW_LEFTBTN, GLBW_BTNUP, event->x(), event->y());
  if(event->button() == Qt::RightButton)
    res = mBaseWindow.BaseInputMouseButton(GLBW_RIGHTBTN, GLBW_BTNUP, event->x(), event->y());
  if(res) return;
	 */
	if(mCurrentMouseButton == event->button()){
		mCurrentMouseButton = Qt::NoButton;
	}
}


#define SET_COLOR4(array,r,g,b,a)   {(array)[0] = (r); (array)[1] = (g); (array)[2] = (b); (array)[3] = (a);}
#define COPY_COLOR4(array,src)   {(array)[0] = (src)[0]; (array)[1] = (src)[1]; (array)[2] = (src)[2]; (array)[3] = (src)[3];}
#define COPY_COLOR3(array,src)   {(array)[0] = (src)[0]; (array)[1] = (src)[1]; (array)[2] = (src)[2]; (array)[3] = 1.0;}

void  WorldRendererWidget::LoadConfig(pXmlTree config){
	if(mConfigTree!=NULL) delete mConfigTree;
	mConfigTree = NULL;

	if(config)
		mConfigTree = config->Clone();
	else
		mConfigTree = new XmlTree("Aspect");

	SET_COLOR4(mBackgroundColor,    0.0,0.0,0.1,1.0);

	bUseFog             = true;
	SET_COLOR4(mFogColor,           0.0,0.0,0.1,1.0);
	mFogNear            =  8.0;
	mFogFar             = 12.0;

	bUseFloor           = false;
	mFloorStepSize      = 0.5;
	mFloorStepCount     = 40;
	mFloorHeight		= 0;
	SET_COLOR4(mFloorColor[0],      0.4,0.4,0.4,1.0);
	SET_COLOR4(mFloorColor[1],      0.1,0.1,0.1,1.0);

	mAntialiasingLevel  = 0;

	mNumLights          = 2;

	SET_COLOR4(mLightColor[0][0],   0.5,0.5,0.5,1.0);
	SET_COLOR4(mLightColor[0][1],   1.0,1.0,1.0,1.0);
	SET_COLOR4(mLightColor[0][2],   0.0,0.0,0.0,1.0);
	SET_COLOR4(mLightPosition[0],    8.0, 8.0, 10.0,1.0);
	SET_COLOR4(mLightDirection[0],  -8.0,-8.0,-10.0,1.0);
	bUseLightDirection[0] = true;
	bUseLightPosition[0]  = true;

	SET_COLOR4(mLightColor[1][0],   0.0,0.0,0.0,1.0);
	SET_COLOR4(mLightColor[1][1],   0.8,0.8,0.8,1.0);
	SET_COLOR4(mLightColor[1][2],   0.0,0.0,0.0,1.0);
	SET_COLOR4(mLightPosition[1],  -8.0,-8.0, 10.0,1.0);
	SET_COLOR4(mLightDirection[1],  8.0, 8.0,-10.0,1.0);
	bUseLightDirection[1] = true;
	bUseLightPosition[1]  = true;

	bUseGrid                = false;
	mGridStepCount          = 1*2;
	mGridHeightStepCount    = 2*2;
	SET_COLOR4(mGridColor,  1.0,1.0,0.0,1.0);


	mColorMaterial = 2;

	mCameraInitLookAt.Set(0,0,0.5);
	mCameraInitPos.Set(3,-3,1);


	if(config){
		pXmlTree tree;
		float *array;

		mAntialiasingLevel = config->Get("AntialiasingLevel",0);
		mAntialiasingLevel = TRUNC(mAntialiasingLevel,0,4);


		mColorMaterial = config->Get("ColorMaterial",2);
		mColorMaterial = TRUNC(mColorMaterial,0,4);

		if(config->GetArray("BackgroundColor",&array)==3){   COPY_COLOR3(mBackgroundColor,array);}
		else if(config->GetArray("BackgroundColor",&array)==4){   COPY_COLOR4(mBackgroundColor,array);}
		else                                                  {   SET_COLOR4 (mBackgroundColor,0,0,0,1);}

		if((tree=config->Find("Fog"))!=NULL){
			bUseFog = true;
			if(tree->GetArray("Color",&array)==3){      COPY_COLOR3(mFogColor,array);}
			else if(tree->GetArray("Color",&array)==4){      COPY_COLOR4(mFogColor,array);}
			else                                      {      SET_COLOR4 (mFogColor,0,0,0,1);}

			mFogNear    = tree->Get("Near",mFogNear);
			mFogFar     = tree->Get("Far", mFogFar);
		}else{
			bUseFog = false;
		}

		if((tree=config->Find("Floor"))!=NULL){
			//bUseFloor = true;
			if(tree->GetArray("PrimaryColor",&array)==3){    COPY_COLOR3(mFloorColor[0],array);}
			else if(tree->GetArray("PrimaryColor",&array)==4){    COPY_COLOR4(mFloorColor[0],array);}
			else                                             {    SET_COLOR4 (mFloorColor[0],0,0,0,1);}

			if(tree->GetArray("SecondaryColor",&array)==3){  COPY_COLOR3(mFloorColor[1],array);}
			else if(tree->GetArray("SecondaryColor",&array)==4){  COPY_COLOR4(mFloorColor[1],array);}
			else                                               {  SET_COLOR4 (mFloorColor[1],0,0,0,1);}

			mFloorHeight = tree->Get("Height",mFloorHeight);
			mFloorStepSize  = tree->Get("StepSize", mFloorStepSize);
			mFloorStepCount = tree->Get("StepCount",mFloorStepCount);
		}else{
			//bUseFloor = false;
		}

		if((tree=config->Find("Grid"))!=NULL){
			bUseGrid = true;
			if(tree->GetArray("Color",&array)==3){           COPY_COLOR3(mGridColor,array);}
			else if(tree->GetArray("Color",&array)==4){           COPY_COLOR4(mGridColor,array);}
			else                                             {    SET_COLOR4 (mGridColor,0,0,0,1);}

			mGridStepCount       = tree->Get("StepCount", 0);
			mGridHeightStepCount = tree->Get("VertStepCount",0);
			mGridHeightStepCount*=2;

			if(mGridStepCount*mGridHeightStepCount==0)
				bUseGrid = false;

		}else{
			bUseGrid = false;
		}


		if((tree=config->Find("Lights"))!=NULL){
			pXmlTreeList lightList = tree->GetSubTrees();
			unsigned int cnt=0;

			for(unsigned int i=0;i<lightList->size();i++){
				pXmlTree ltree = lightList->at(i);
				if(ltree->GetName()=="Light"){
					if(ltree->GetArray("AmbientColor",&array)==3){     COPY_COLOR3(mLightColor[cnt][0],array);}
					else if(ltree->GetArray("AmbientColor",&array)==4){     COPY_COLOR4(mLightColor[cnt][0],array);}
					else                                              {     SET_COLOR4 (mLightColor[cnt][0],0,0,0,0);}

					if(ltree->GetArray("DiffuseColor",&array)==3){     COPY_COLOR3(mLightColor[cnt][1],array);}
					else if(ltree->GetArray("DiffuseColor",&array)==4){     COPY_COLOR4(mLightColor[cnt][1],array);}
					else                                              {     SET_COLOR4 (mLightColor[cnt][1],0,0,0,0);}

					if(ltree->GetArray("SpecularColor",&array)==3){    COPY_COLOR3(mLightColor[cnt][2],array);}
					else if(ltree->GetArray("SpecularColor",&array)==4){    COPY_COLOR4(mLightColor[cnt][2],array);}
					else                                               {    SET_COLOR4 (mLightColor[cnt][2],0,0,0,0);}

					if(ltree->Find("Position")){
						if(ltree->GetArray("Position",&array)==3){         COPY_COLOR3(mLightPosition[cnt],array);}
						else if(ltree->GetArray("Position",&array)==4){         COPY_COLOR4(mLightPosition[cnt],array);}
						else                                          {         SET_COLOR4 (mLightPosition[cnt],0,0,0,0);}
						bUseLightPosition[cnt]  = true;
					}else{
						bUseLightPosition[cnt]  = false;
					}
					if(ltree->Find("Direction")){
						if(ltree->GetArray("Direction",&array)==3){         COPY_COLOR3(mLightDirection[cnt],array);}
						else if(ltree->GetArray("Direction",&array)==4){         COPY_COLOR4(mLightDirection[cnt],array);}
						else                                           {         SET_COLOR4 (mLightDirection[cnt],0,0,0,0);}
						bUseLightDirection[cnt]  = true;
					}else{
						bUseLightDirection[cnt]  = false;
					}
					cnt++;
					if(cnt>=MAX_NUM_LIGHTS)
						break;
				}
			}
			mNumLights = cnt;
		}else{
			mNumLights = 0;
		}

		if((tree=config->Find("Camera"))!=NULL){
			if(tree->Find("Position")){
				REALTYPE *array;
				int size=tree->GetArray("Position",&array);
				if(size==3) mCameraInitPos.Set(array);
			}
			if(tree->Find("LookAtPoint")){
				REALTYPE *array;
				int size=tree->GetArray("LookAtPoint",&array);
				if(size==3) mCameraInitLookAt.Set(array);
			}
		}
	}

	if(!mConfigTree->Find("Robot")){

	}

	bInitGL = true;
}

void  WorldRendererWidget::DrawFloor(){
	glEnable(GL_POLYGON_OFFSET_FILL);
	if(mCamera.m_position.cz()>0){
		glPolygonOffset(0.0f, 100.0f);
	}else{
		glPolygonOffset(0.0f, -100.0f);
	}

	if(mFloorCallListId>0){
		glCallList(mFloorCallListId);
	}else{
		if(mFloorCallListId==0)
			mFloorCallListId = glGenLists(1);
		if(mFloorCallListId>0)
			glNewList(mFloorCallListId,GL_COMPILE);

		float col[4];
		col[0] = 0.2;
		col[1] = 0.2;
		col[2] = 0.2;
		col[3] = 1.0;
		glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,col);
		glBegin(GL_QUADS);
		float start  = -float(mFloorStepCount)*0.5*mFloorStepSize;
		float startx = start;
		for(int i=0;i<mFloorStepCount;i++){
			float starty = start;
			for(int j=0;j<mFloorStepCount;j++){
				int cId = (i+j) % 2;
				glColor4f(mFloorColor[cId][0],mFloorColor[cId][1],mFloorColor[cId][2],mFloorColor[cId][3]);
				glNormal3f(0,0,1);
				glVertex3f(startx,starty,mFloorHeight);
				glVertex3f(startx+mFloorStepSize,starty,mFloorHeight);
				glVertex3f(startx+mFloorStepSize,starty+mFloorStepSize,mFloorHeight);
				glVertex3f(startx,starty+mFloorStepSize,mFloorHeight);
				glNormal3f(0,0,-1);
				glVertex3f(startx,starty+mFloorStepSize,mFloorHeight);
				glVertex3f(startx+mFloorStepSize,starty+mFloorStepSize,mFloorHeight);
				glVertex3f(startx+mFloorStepSize,starty,mFloorHeight);
				glVertex3f(startx,starty,mFloorHeight);
				starty+=mFloorStepSize;
			}
			startx+=mFloorStepSize;
		}
		glEnd();
		if(mFloorCallListId>0)
			glEndList();
	}
	glDisable(GL_POLYGON_OFFSET_FILL);
}


void  WorldRendererWidget::DrawGrid(){
	if(mGridCallListId>0){
		glCallList(mGridCallListId);
	}else{
		if(mGridCallListId==0)
			mGridCallListId = glGenLists(1);
		if(mGridCallListId>0)
			glNewList(mGridCallListId,GL_COMPILE);

		glLineStipple(2, 0xAAAA);
		glEnable(GL_LINE_STIPPLE);
		glBegin(GL_LINES);
		glColor4f(mGridColor[0],mGridColor[1],mGridColor[2],mGridColor[3]);
		for(int i=-mGridStepCount;i<=mGridStepCount;i+=2){
			for(int j=-mGridStepCount;j<=mGridStepCount;j+=2){
				glVertex3f(mFloorStepSize*i,mFloorStepSize*j,0);
				glVertex3f(mFloorStepSize*i,mFloorStepSize*j,mFloorStepSize*mGridHeightStepCount);
			}
		}
		for(int i=-mGridStepCount;i<=mGridStepCount;i+=2){
			for(int j=0;j<=mGridHeightStepCount;j+=2){
				glVertex3f(mFloorStepSize*i,mFloorStepSize*-mGridStepCount,mFloorStepSize*j);
				glVertex3f(mFloorStepSize*i,mFloorStepSize*mGridStepCount,mFloorStepSize*j);
				glVertex3f(mFloorStepSize*-mGridStepCount,mFloorStepSize*i,mFloorStepSize*j);
				glVertex3f(mFloorStepSize*mGridStepCount,mFloorStepSize*i,mFloorStepSize*j);
			}
		}
		glEnd();
		glDisable(GL_LINE_STIPPLE);

		if(mFloorCallListId>0)
			glEndList();
	}
}

void  WorldRendererWidget::DrawCameraLookAt(){
	glColor4f(0,0.8,1,0.6);
	glPushMatrix();
	glTranslatef(mCamera.m_lookAtPoint[0],mCamera.m_lookAtPoint[1],mCamera.m_lookAtPoint[2]);
	mCameraObject.Render();
	mCameraObjectLine.Render();
	glRotatef(90,1,0,0);
	mCameraObjectLine.Render();
	glRotatef(90,0,1,0);
	mCameraObjectLine.Render();
	glPopMatrix();
}

