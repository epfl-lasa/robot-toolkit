#ifndef __MAINWINDOW_H__
#define __MAINWINDOW_H__

#include <QMainWindow>
#include <QDockWidget>
#include <QTabWidget>
#include <QFileDialog>

#include <QTimer>
#include <QAction>
#include <vector>

#include "WorldRendererWidget.h"
#include "ConsoleWidget.h"
#include "StdTools/LogStream.h"

#include "SimulatorDynamics/SimulatorDynamicsInterface.h"



class MainWindow : public QMainWindow
{
    Q_OBJECT
  
protected:
    WorldRendererWidget     *mWorldRendererWidget;
    vector<ConsoleWidget*>   mConsoleWidgets;

    QDockWidget             *mConsoleDock;
    QTabWidget              *mConsoleTab;

    QToolBar                *mToolBar;

    QTimer                  *mMsgConsoleTimer;

    pXmlTree                 mCurrentConfig;
    string                   mCurrentConfigFile;

    QFileDialog             *mFileDialog;

    SimulatorDynamicsInterface*  mCoreInterface;

public:
    MainWindow(SimulatorDynamicsInterface* simInterface, pXmlTree config = NULL, QWidget * parent=0, Qt::WindowFlags flags = 0 );
    ~MainWindow();
  
    WorldRendererWidget * GetWorldRendererWidget();

    void SetConsole(Console * console);

    void    Start();
    void    Stop();

    void    SetCurrentConfigFile(string file);

    void    SetupConfig();



    virtual void 	closeEvent(QCloseEvent *event){emit isGonnaClose();}

public slots:

    void LoadConfig();
    void ReLoadFile();
    void ReLoadConfig();
    void ToggleConsole();
    void ToggleFullscreen();

    void CheckMsgConsoleUpdate();

    void SwitchToConsole(int id);



signals:
     void isGonnaClose();

};


class ConsoleAction : public QAction
{
    Q_OBJECT
protected:
    int mID;
public:
    ConsoleAction(int id, QObject * parent);

    int GetId();
 public slots:
     void triggerBridge();

 signals:
     void valueTriggered(int newValue);

};

#endif

