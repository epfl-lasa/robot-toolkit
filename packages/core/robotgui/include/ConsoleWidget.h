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

#ifndef __ConsoleWidget_H__
#define __ConsoleWidget_H__

#include <QObject>
#include <QTextEdit>
#include <QString>
#include <QStringList>

#include <QMouseEvent>
#include <QKeyEvent>


#include "StdTools/XmlTree.h"
#include "StdTools/Console.h"

#define MAX_NUM_LIGHTS 8



class ConsoleWidget : public QTextEdit , public ConsoleNotification
{
    Q_OBJECT
protected:

    int     mScreenWidth;
    int     mScreenHeight;

    bool    bTextChanged;
    bool    bCursorChanged;

    QStringList mContent;

    Console *mMainConsole;
    Console *mConsole;
    int mConsoleIndex;

public:
  ConsoleWidget(QWidget *parent=NULL);
  ~ConsoleWidget();

    void UpdateText();
    void    SetConsole(Console *console);

protected:
  virtual void keyPressEvent (QKeyEvent * event);
  virtual void keyReleaseEvent (QKeyEvent * event);
  virtual void showEvent ( QShowEvent * event );

  virtual void NeedUpdate();

  /*

  virtual void mouseMoveEvent(QMouseEvent * event);
  virtual void mousePressEvent(QMouseEvent * event);
  virtual void mouseReleaseEvent(QMouseEvent * event);
*/
};
#endif
