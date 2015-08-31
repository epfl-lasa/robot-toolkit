/****************************************************************************
** Meta object code from reading C++ file 'WorldRendererWidget.h'
**
** Created: Fri Nov 16 16:54:26 2012
**      by: The Qt Meta Object Compiler version 62 (Qt 4.7.4)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../include/WorldRendererWidget.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'WorldRendererWidget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 62
#error "This file was generated using the moc from 4.7.4. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_WorldRendererWidget[] = {

 // content:
       5,       // revision
       0,       // classname
       0,    0, // classinfo
       2,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      21,   20,   20,   20, 0x0a,
      30,   20,   20,   20, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_WorldRendererWidget[] = {
    "WorldRendererWidget\0\0OnIdle()\0OnPaint()\0"
};

const QMetaObject WorldRendererWidget::staticMetaObject = {
    { &QGLWidget::staticMetaObject, qt_meta_stringdata_WorldRendererWidget,
      qt_meta_data_WorldRendererWidget, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &WorldRendererWidget::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *WorldRendererWidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *WorldRendererWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_WorldRendererWidget))
        return static_cast<void*>(const_cast< WorldRendererWidget*>(this));
    return QGLWidget::qt_metacast(_clname);
}

int WorldRendererWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QGLWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: OnIdle(); break;
        case 1: OnPaint(); break;
        default: ;
        }
        _id -= 2;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
