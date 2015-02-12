#include "MainWindow.h"
#include "StdTools/Various.h"

#include <QVBoxLayout>
#include <QAction>
#include <QToolBar>
#include <QTimerEvent>
#include <QStatusBar>
#include <QFrame>
#include <QTimer>
#include <QString>
#include <QFileDialog>
#include <QStringList>



MainWindow::MainWindow(SimulatorDynamicsInterface* simInterface, pXmlTree config, QWidget *parent, Qt::WindowFlags flags)
: QMainWindow(parent,flags){

    mCoreInterface = simInterface;

    if(config)
        mCurrentConfig = config->Clone();
    else
        mCurrentConfig  = NULL;

    gLOG.SetAutoPrint("Messages", true);
    gLOG.SetOStream("Messages",*(mCoreInterface->GetConsole())->GetStream());


    setMinimumSize(800,640);
    
    setAnimated(false);

    setDockNestingEnabled(false);

    setCorner(Qt::TopLeftCorner, Qt::LeftDockWidgetArea);
    setCorner(Qt::BottomLeftCorner, Qt::LeftDockWidgetArea);
    setCorner(Qt::TopRightCorner, Qt::TopDockWidgetArea);
    setCorner(Qt::BottomRightCorner, Qt::BottomDockWidgetArea);

    statusBar()->hide();
    

    mWorldRendererWidget = new WorldRendererWidget(config->Find("Aspect"));
    mWorldRendererWidget->setMinimumSize(320,200);
    mWorldRendererWidget->setFocusPolicy(Qt::StrongFocus );
    setCentralWidget(mWorldRendererWidget);




    mConsoleWidgets.push_back(new ConsoleWidget());

    mConsoleTab = new QTabWidget();
    mConsoleTab->addTab(mConsoleWidgets[0],tr("Console"));
    

    QDockWidget::DockWidgetFeatures myFeats = QDockWidget::DockWidgetMovable;//QDockWidget::DockWidgetVerticalTitleBar;

    mConsoleDock = new QDockWidget(tr("Console"),this,Qt::Widget);
    mConsoleDock->setFeatures(myFeats);
    mConsoleDock->setTitleBarWidget(0);
    mConsoleDock->setAllowedAreas(Qt::BottomDockWidgetArea|Qt::TopDockWidgetArea|Qt::RightDockWidgetArea);
    addDockWidget(Qt::BottomDockWidgetArea, mConsoleDock);  
    mConsoleDock->setWidget(mConsoleTab);  

    QWidget* lTitleBar = mConsoleDock->titleBarWidget();
    QWidget* lEmptyWidget = new QWidget();
    mConsoleDock->setTitleBarWidget(lEmptyWidget);
    delete lTitleBar;



    mToolBar = addToolBar(tr("Window Toolbar"));
    //tb->setAllowedAreas ( Qt::ToolBarAreas areas )
    mToolBar->setFloatable(false);
    mToolBar->setMovable(false);


    QAction *newAct;
    newAct = new QAction(tr("&Load [F1]"), this);
    newAct->setShortcut(tr("F1"));
    connect(newAct, SIGNAL(triggered()), this, SLOT(LoadConfig()));
    mToolBar->addAction(newAct);
    this->addAction(newAct);

    newAct = new QAction(tr("&ReLoad File [F2]"), this);
    newAct->setShortcut(tr("F2"));
    connect(newAct, SIGNAL(triggered()), this, SLOT(ReLoadFile()));
    mToolBar->addAction(newAct);
    this->addAction(newAct);

    newAct = new QAction(tr("&Hide Console [F3]"), this);
    newAct->setShortcut(tr("F3"));
    connect(newAct, SIGNAL(triggered()), this, SLOT(ToggleConsole()));
    mToolBar->addAction(newAct);
    this->addAction(newAct);

    newAct = new QAction(tr("&Fullscreen [F4]"), this);
    newAct->setShortcut(tr("F4"));
    connect(newAct, SIGNAL(triggered()), this, SLOT(ToggleFullscreen()));
    mToolBar->addAction(newAct);
    this->addAction(newAct);

    for(int i=0;i<10;i++){
        newAct = new ConsoleAction(i, this);
        newAct->setShortcut(QKeySequence(Qt::ALT + (Qt::Key_0+i)));
        connect(newAct, SIGNAL(valueTriggered(int)), this, SLOT(SwitchToConsole(int)));
        this->addAction(newAct);
    }

    mWorldRendererWidget->SetSimulatorInterface(mCoreInterface);

    mFileDialog = new QFileDialog(this);
    mFileDialog->setFileMode(QFileDialog::AnyFile);
    mFileDialog->setNameFilter(tr("XML files (*.xml)"));
    mFileDialog->setViewMode(QFileDialog::Detail);
    mFileDialog->setDirectory("./config");




    mMsgConsoleTimer= new QTimer(this);
    connect(mMsgConsoleTimer, SIGNAL(timeout()), this, SLOT(CheckMsgConsoleUpdate()));
    mMsgConsoleTimer->start(100);

    SetupConfig();


}



MainWindow::~MainWindow(){
    mConsoleTab->clear();
    for(size_t i=0;i<mConsoleWidgets.size();i++){
        delete mConsoleWidgets[i];
    }
    mConsoleWidgets.clear();
    if(mCurrentConfig) delete mCurrentConfig; mCurrentConfig = NULL;
    delete mMsgConsoleTimer;
    delete mFileDialog;
    delete mToolBar;
}


WorldRendererWidget * MainWindow::GetWorldRendererWidget(){
    return mWorldRendererWidget;
}


void MainWindow::SetConsole(Console * console){
    mConsoleTab->clear();
    //mMsgTabIndex = 0;
    unsigned int cnt = 0;
    if(console!=NULL){
        char name[256];
        snprintf(name,256,"%s [Alt-%d]",console->GetName().c_str(),cnt +1);
        mConsoleTab->addTab(mConsoleWidgets[0],tr(name));
        mConsoleWidgets[0]->SetConsole(console);
        Console *subC;
        while((subC = console->GetSubConsole(cnt))!=NULL){
            if(mConsoleWidgets.size()<=cnt+1){
                mConsoleWidgets.push_back(new ConsoleWidget());
            }
            mConsoleWidgets[cnt+1]->SetConsole(subC);
            snprintf(name,256,"%s [Alt-%d]",subC->GetName().c_str(),cnt +2);
            mConsoleTab->addTab(mConsoleWidgets[cnt+1],tr(name));
            cnt++;
        }
        //mMsgTabIndex = i+1;
    }
    /*
    ConsoleWidget *cw = new ConsoleWidget();
    cw->setLineWrapMode(QTextEdit::NoWrap);
    cw->SetConsole(&mMsgConsole);
    mConsoleTab->addTab(cw,tr("Messages [Alt-0]"));
    mConsoleWidgets.push_back(cw);
    */
    for(int i=0;i<mConsoleTab->count();i++)
        mConsoleTab->setTabEnabled(i,true);
    mConsoleTab->setCurrentIndex(0);

}

void MainWindow::LoadConfig(){

    QStringList fileNames;
    if (mFileDialog->exec()){
        fileNames = mFileDialog->selectedFiles();

        if(fileNames.size()>0){
            mCurrentConfigFile = string(fileNames[0].toUtf8().data());
            ReLoadFile();
        }
    }
}
void MainWindow::ReLoadFile(){
    if(mCurrentConfig) delete mCurrentConfig;
    mCurrentConfig = new XmlTree();

    gLOG.SetCurrentEntry("Messages");
    gLOG.Append("Loading file <%s>",mCurrentConfigFile.c_str());

    if(FileExists(mCurrentConfigFile)){
        if(!mCurrentConfig->LoadFromFile(mCurrentConfigFile)){
            delete mCurrentConfig;
            mCurrentConfig = NULL;
            gLOG.SetCurrentEntry("Messages");
            gLOG.Append("Error while reading file <%s>",mCurrentConfigFile.c_str());
        }
    }else{
        gLOG.SetCurrentEntry("Messages");
        gLOG.Append("Error file <%s> not found",mCurrentConfigFile.c_str());
    }
    SetupConfig();
}


void MainWindow::ReLoadConfig(){
    gLOG.SetCurrentEntry("Messages");
    gLOG.Append("Reloading configuration");
    SetupConfig();
}
void MainWindow::SetupConfig(){

    Stop();
    //mCurrentConfig->Print();

    XmlTree *gfxTree = NULL;
    if(FileFinder::Find("Simulator/config.xml")){
        gfxTree = new XmlTree();
        if(gfxTree->LoadFromFile(FileFinder::GetCStr())){
            if(mCurrentConfig){
                gfxTree->Patch(mCurrentConfig->Find("Simulator"));
            }
        }else{
            delete gfxTree;
            gfxTree = NULL;
        }
    }else{
        if(mCurrentConfig->Find("Simulator")){
            gfxTree = mCurrentConfig->Find("Simulator")->Clone();
        }
    }

    if(mCurrentConfig){
        if(mCurrentConfig->Find("Simulator")){
            mCurrentConfig->DelSubTree(mCurrentConfig->Find("Simulator"));
            if(gfxTree){
                mCurrentConfig->AddSubTree(gfxTree->Clone());
            }
        }
    }

    if(!mCoreInterface->LoadConfig(mCurrentConfig)){
        gLOG.Append("Error while setting up world");
    }


    if(gfxTree){
        mWorldRendererWidget->LoadConfig(gfxTree->Find("Aspect"));
    }else{
        mWorldRendererWidget->LoadConfig(NULL);
    }
    //Init();
    if(gfxTree){
        delete gfxTree;
    }

    Start();
}
void MainWindow::ToggleConsole(){
    if(mConsoleDock->isHidden()){
        mConsoleDock->show();
        mToolBar->show();
    }else{
        mConsoleDock->hide();
        mToolBar->hide();
    }
}

void    MainWindow::Start(){
    SetConsole(mCoreInterface->GetConsole());
    if(mCoreInterface->Init() == WorldInterface::STATUS_OK){
        if(mCoreInterface->Start() == WorldInterface::STATUS_OK){
            mWorldRendererWidget->SetWorld();
            SetConsole(mCoreInterface->GetConsole());
        }else{
            gLOG.AppendToEntry("Messages","Error during the starting phase");
        }
    }else{
        gLOG.AppendToEntry("Messages","Error during the initialisation phase");
    }
}
void    MainWindow::Stop(){
    SetConsole(NULL);
    //SetConsole(mCoreInterface->GetConsole());
    mCoreInterface->Stop();
    mCoreInterface->Free();
    mWorldRendererWidget->SetWorld();
}

void MainWindow::CheckMsgConsoleUpdate(){
    mCoreInterface->GetConsole()->Update();
}
void MainWindow::ToggleFullscreen(){
    if(windowState() & Qt::WindowFullScreen)
        showNormal();
    else
        showFullScreen();
}
void    MainWindow::SetCurrentConfigFile(string file){
    mCurrentConfigFile = file;
}


void MainWindow::SwitchToConsole(int id){
    if(id<0) return;
    if(id-1<mConsoleTab->count()){
        mConsoleTab->setCurrentIndex(id-1);
    }
}

ConsoleAction::ConsoleAction(int id,QObject * parent)
        :QAction(parent){
    mID = id;
    connect(this, SIGNAL(triggered()), this, SLOT(triggerBridge()));
}

int ConsoleAction::GetId(){return mID;}

void ConsoleAction::triggerBridge(){
    emit valueTriggered(mID);
}




