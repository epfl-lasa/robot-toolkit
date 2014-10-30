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

#include <QApplication>
#include <QObject>
#include <QTimer>
#include <QWidget>

#include <dlfcn.h>

#include "StdTools/XmlTree.h"
#include "StdTools/Timer.h"
#include "StdTools/Various.h"
#include "StdTools/LogStream.h"


#include "RobotGUI/MainWindow.h"

#include "UDPNetwork/UDPNetwork.h"
UDPNetwork net;

void ShowHelp();

#include <limits.h>

int main( int argc, char **argv )
{
    QApplication myApp( argc, argv );


    FileFinder::AddBasePath(".");
    FileFinder::AddBasePath("./data");

    gLOG.SetAutoPrint("Messages", true);

    XmlTree args;
    pXmlTree tree,tree2;
    //pXmlTreeList tlist,tlist2;
    //char modName[256];
    //Robot *cRobot;
    //Robot *csRobot;
    vector<string> nameAndPatches;

    bool bShowHelp      = false;
    //bool bError         = false;

    XmlTree argStruct("args","",11, new XmlTree("fullscreen","f","needArg=\"\""),
                                    new XmlTree("config",    "", "needArg=\"true\""),
                                    new XmlTree("debug",     "d","needArg=\"\""),
                                    new XmlTree("robot",     "r","needArg=\"true\""),
                                    new XmlTree("simrobot",  "s","needArg=\"true\""),
                                    new XmlTree("module",    "m","needArg=\"true\""),
                                    new XmlTree("control",   "c","needArg=\"true\""),
                                    new XmlTree("args",      "a","needArg=\"true\""),
                                    new XmlTree("world",     "w","needArg=\"true\""),
                                    new XmlTree("period",    "p","needArg=\"true\""),
                                    new XmlTree("help",      "h","needArg=\"\"")
                                    );
    bShowHelp = !args.ParseArguments(argc,argv, &argStruct);

    if(args.Find("help")){
        ShowHelp();
        return 0;
    }


    string configFile;
    XmlTree config("Config");
    if((tree = args.Find("config"))!=NULL){
        char txt[256];
        sprintf(txt,"config/%s.xml",tree->GetData().c_str());
        if(FileFinder::Find(txt)){
            if(!config.LoadFromFile(FileFinder::GetCStr())){
                fprintf(stderr,"Error while reading file: %s\n",FileFinder::GetCStr());
                fprintf(stderr,"Exiting\n");
                return -1;
            }else{
                configFile = FileFinder::GetString();
            }
        }else{
            fprintf(stderr,"Error: file %s for found\n",txt);
            fprintf(stderr,"Exiting\n");
            return -1;
        }
    }

    // Parsing arguments
    if(args.Find("fullscreen")){
        if(config.Find("Simulator")==NULL) config.Set("Simulator",string(""));
        config.Find("Simulator")->Set("Fullscreen",string(""));
    }
    if(args.Find("period")){
        if(config.Find("Simulator")==NULL) config.Set("Simulator",string(""));
        config.Find("Simulator")->Set("Period",args.Get("period",string("")));
    }
    if(args.Find("debug")){
        config.Set("Debug",string(""));
    }
    if(args.Find("robot")){
        while((tree=config.Find("Robot"))!=NULL){
            config.DelSubTree(tree);
        }
        config.Set("Robot",args.Get("robot",string("")));
    }
    if(args.Find("module")){
        if((tree=config.Find("Robot"))==NULL){
            cerr << "Error: not robot defined..."<<endl;
            bShowHelp = true;
        }else{
            while((tree2=config.Find("RobotModule"))!=NULL){
                tree->DelSubTree(tree2);
            }
            tree->Set("RobotModule",args.Get("module",string("")));
        }
    }
    if(args.Find("simrobot")){
        if((tree=config.Find("Robot"))==NULL){
            cerr << "Error: not robot defined..."<<endl;
            bShowHelp = true;
        }else{
            while((tree2=config.Find("SimulationRobot"))!=NULL){
                tree->DelSubTree(tree2);
            }
            tree->Set("SimulationRobot",args.Get("simrobot",string("")));
        }
    }
    if(args.Find("control")){
        if((tree=config.Find("Robot"))==NULL){
            cerr << "Error: not robot defined..."<<endl;
            bShowHelp = true;
        }else{
            tree->Set("ControlMode",args.Get("control",string("")));
        }
    }
    if(args.Find("args")){
        if((tree=config.Find("Robot"))==NULL){
            cerr << "Error: not robot defined..."<<endl;
            bShowHelp = true;
        }else{
            while((tree2=config.Find("Args"))!=NULL){
                tree->DelSubTree(tree2);
            }
            tree->Set("Args",args.Get("args",string("")));
        }
    }

    if(args.Find("world")){
        while((tree=config.Find("World"))!=NULL){
            config.DelSubTree(tree);
        }
        config.Set("World",args.Get("world",string("")));
    }




    if(bShowHelp){
        ShowHelp();
        return 0;
    }


    if(config.Find("Debug"))
        gLOG.SetDefaultAutoPrint(true);


    /*
    // User interface
    XmlTree gfxTree;
    pXmlTree winConfigTree = NULL;
    if(FileExists("./dta/Simulator/config.xml")){
        gfxTree.LoadFromFile("./data/Simulator/config.xml");
        winConfigTree = gfxTree.Find("  ");
        if(winConfigTree)
            config.AddSubTree(winConfigTree->Clone());
    }
    */

    SimulatorDynamicsInterface * mCoreInterface = new SimulatorDynamicsInterface();

//    Robot->SetUserInfo(&mCoreInterface);
//    mCoreInterface->mSimulatorDynamics->GetDynamicObject()

    MainWindow mainWindow(mCoreInterface,&config);

    mainWindow.SetCurrentConfigFile(configFile);

    /*
    WorldRendererWidget* worldWidget = mainWindow.GetWorldRendererWidget();

    //mPeriod = config.Get("Simulator.Period",0.001);
    worldWidget->SetProcessingPeriod(config.Get("Simulator.Period",0.001));
    Vector3 camPos(3,-3,1);
    Vector3 camLookAt(0,0,0.5);
    if(config.Find("Simulator.CameraPos")){
        REALTYPE *array;
        int size=config.GetArray("Simulator.CameraPos",&array);
        if(size==3) camPos.Set(array);
    }
    if(config.Find("Simulator.CameraLookAt")){
        REALTYPE *array;
        int size=config.GetArray("Simulator.CameraLookAt",&array);
        if(size==3) camLookAt.Set(array);
    }
    worldWidget->SetInitCameraParams(camPos,camLookAt);

    widgetPtr = worldWidget;
    */
    
    mainWindow.show();


    //myApp.setQuitOnLastWindowClosed (false);

    QObject::connect(&mainWindow, SIGNAL(isGonnaClose()), &myApp, SLOT(quit()));


    myApp.exec();

    delete mCoreInterface;
#if 0    
    if(myCoreInterface->Init() == WorldInterface::STATUS_OK){
        if(myCoreInterface->Start() == WorldInterface::STATUS_OK){


            if(config.Find("Simulator.Fullscreen"))
                mainWindow.showFullScreen();

            myApp.exec();

            cerr << "Exiting: Cleaning up"<<endl;

            myCoreInterface->Stop();
            myCoreInterface->Free();
        }else{
            cerr << "Error while starting interface"<<endl;
            myCoreInterface->Stop();
            myCoreInterface->Free();
        }
    }else{
        cerr << "Error while initializing interface"<<endl;
    }
#endif
    return 0;
}


void ShowHelp(){
    cout << "Usage:  RobotSimulator [options]"<<endl;
    cout << endl;
    cout << "Options list:"<<endl;
    cout << endl;
    cout << "  --config          : Config file (as in ./config/)"<<endl;
    cout << endl;
    cout << "  --robot      (-r) : Robot name (as in ./data/Robots/)"<<endl;
    cout << "                      If provided together with a config file, this field "<<endl;
    cout << "                      erase all robots description from the config file."<<endl;
    cout << "  --simrobot   (-s) : To-be-simulated robot name (as in ./data/Robots/)"<<endl;
    cout << "  --module     (-m) : Robot module (must be in ./module folder, whithout the .so extension)"<<endl;
    cout << "                      If provided together with a config file, this field "<<endl;
    cout << "                      force the use of a single module."<<endl;
    cout << "  --control    (-c) : Force to a given a control mode for all the robot joints."<<endl;
    cout << "                      Possible choices are: 'position','velocity','acceleration','torque'"<<endl;
    cout << "  --args       (-a) : Arguments to be passed to the robot structure."<<endl;
    cout << endl;
    cout << "  --world      (-w) : World name (as in ./data/Worlds/)"<<endl;
    cout << "                      If provided together with a config file, this field "<<endl;
    cout << "                      erase all world description from the config file."<<endl;
    cout << endl;
    cout << "  --period     (-p) : Simulation period in seconds"<<endl;
    cout << "  --fullscreen (-f) : Fullscreen mode"<<endl;
    cout << "  --debug      (-d) : Show debug messages"<<endl;
    cout << endl;
    cout << "  --help       (-h) : Show this help"<<endl;
    cout << endl;
    cout << "Example:"<<endl;
    cout << "  RobotSimulator -r WAM/Racket -m RobotPongModule"<<endl;
    cout << endl;
}

