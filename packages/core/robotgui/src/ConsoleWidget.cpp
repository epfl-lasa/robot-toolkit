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

#include "ConsoleWidget.h"

#include <QTimer>


ConsoleWidget::ConsoleWidget(QWidget *parent)
:QTextEdit(parent) {

    setMinimumSize (300,100);
    setReadOnly(true);
    mScreenHeight   = 80;
    QFont font("Courier");
    font.setFixedPitch(true);
    font.setPointSize( 10);
    setCurrentFont(font);

    setWordWrapMode(QTextOption::WrapAnywhere);
    bTextChanged        = true;
    bCursorChanged      = true;

    mConsole = NULL;
    mMainConsole = NULL;

    mConsoleIndex = -1;

    UpdateText();
}

ConsoleWidget::~ConsoleWidget(){
}


void    ConsoleWidget::SetConsole(Console *console){
    mMainConsole = console;
    mConsole = console;
    if(console) console->SetNotifier(this);
    UpdateText();
}

void ConsoleWidget::UpdateText(){
    if(!mConsole){
        setPlainText("<No console>");
        QTextCursor tc = textCursor();
        tc.movePosition(QTextCursor::End);
        setTextCursor(tc);
        return;
    }



    if(bTextChanged){
        mContent.clear();

        int linePos = 0;
        int nbLinesAvailable = mScreenHeight-1;

        if(nbLinesAvailable>0){
            linePos     = (nbLinesAvailable-mConsole->GetLines().size());
            int startLine   = 0;
            if(linePos<0){
                startLine   = -linePos;
                linePos     = 0;
            }

            for(int i=startLine;i<(int)mConsole->GetLines().size();i++){
                mContent << tr(mConsole->GetLines()[i].c_str()); 
                linePos++;
            }
        }

        if(mConsole->IsActive()){
            string s;
            s = mConsole->GetName();
            s.append("> ");
            s.append(mConsole->GetCurrentCommand());
            mContent << tr(s.c_str());
        }
        setPlainText(mContent.join(tr("\n")));

    }

    if(bCursorChanged || bTextChanged){
        QTextCursor tc = textCursor();
        tc.movePosition(QTextCursor::End);
        if(mConsole->IsActive()){
            tc.movePosition(QTextCursor::Left,QTextCursor::MoveAnchor,mConsole->GetCurrentCommand().length()- mConsole->GetCursorPos());
            setReadOnly(false);
        }else{
            setReadOnly(true);
        }
        setTextCursor(tc);
    }


    bCursorChanged  = false;
    bTextChanged    = false;
}  

void ConsoleWidget::keyPressEvent (QKeyEvent * event){
    /*
    if(event->modifiers() == Qt::ControlModifier){
        QTextEdit::keyPressEvent(event);
        return;
    }
    */
    if(mConsole){
        
        bool bSpecialChar = true;
        if(event->text().length()>0){
            char key = event->text()[0].toAscii();
            if((key>=' ') && (key<'~')){
                mConsole->AddChar(key);
                bSpecialChar = false;
                bTextChanged = true;
            }
        }        

        if(bSpecialChar){
            switch(event->key()){
            case Qt::Key_Backspace:
                mConsole->EraseChar(true);
                bTextChanged = true;
                break;
            case Qt::Key_Delete:
                mConsole->EraseChar(false);
                bTextChanged = true;
                break;
            case Qt::Key_Enter:
            case Qt::Key_Return:
                mConsole->Accept();
                bTextChanged = true;
                break;
            case Qt::Key_Tab:
                mConsole->AutoCompletion();
                bTextChanged = true;
                break;
            case Qt::Key_Up:
                mConsole->HistoryPrev();
                bTextChanged = true;
                break;
            case Qt::Key_Down:
                mConsole->HistoryNext();
                bTextChanged = true;
                break;
            case Qt::Key_Left:
                if(event->modifiers() == Qt::ControlModifier){
                    mConsoleIndex--;

                    if( (mConsole = mMainConsole->GetSubConsole(mConsoleIndex)) == NULL){
                        mConsole = mMainConsole;
                        mConsoleIndex = -1;
                    }

                    bCursorChanged  = true;
                    bTextChanged = true;
                }else{
                    mConsole->MoveLeft();        
                    bCursorChanged  = true;
                }
                break;
            case Qt::Key_Right:
                if(event->modifiers() == Qt::ControlModifier){
                    mConsoleIndex++;
                    Console* oldC = mConsole;
                    if( (mConsole = mMainConsole->GetSubConsole(mConsoleIndex)) == NULL){
                        mConsole = oldC;
                        mConsoleIndex--;
                    }

                    bCursorChanged  = true;
                    bTextChanged = true;
                }else{
                    mConsole->MoveRight();
                    bCursorChanged  = true;
                }
                break;
            case Qt::Key_Clear:
                mConsole->ClearLine();
                bTextChanged = true;
                break;   
            default:
                break;   
    /*            char str[128];
                sprintf(str,"%d ",event->key());
                for(int i=0;i<strlen(str);i++)
                    AddChar(str[i]);
                break;*/
            }
        }
    }

    //

    if(mConsole){
        if((bTextChanged)||(bCursorChanged)){
            event->accept();
            NeedUpdate();
        }else{
            QTextEdit::keyPressEvent(event);
        }
    }else{
        QTextEdit::keyPressEvent(event);
    }
}
void ConsoleWidget::keyReleaseEvent (QKeyEvent * event){
    QTextEdit::keyReleaseEvent(event);
}
void ConsoleWidget::showEvent ( QShowEvent * event ){
    bTextChanged = true;
    UpdateText();
    QWidget::showEvent(event);
}
void ConsoleWidget::NeedUpdate(){
    bTextChanged = true;
    UpdateText();
}
