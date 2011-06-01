/****************************************************************************
** Meta object code from reading C++ file 'openni_listener.h'
**
** Created: Wed Jun 1 18:41:57 2011
**      by: The Qt Meta Object Compiler version 62 (Qt 4.7.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../src/openni_listener.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'openni_listener.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 62
#error "This file was generated using the moc from 4.7.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_OpenNIListener[] = {

 // content:
       5,       // revision
       0,       // classname
       0,    0, // classinfo
       8,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       6,       // signalCount

 // signals: signature, parameters, type, tag, flags
      16,   15,   15,   15, 0x05,
      39,   15,   15,   15, 0x05,
      67,   15,   15,   15, 0x05,
      89,   15,   15,   15, 0x05,
     130,  122,   15,   15, 0x05,
     150,  122,   15,   15, 0x05,

 // slots: signature, parameters, type, tag, flags
     172,   15,   15,   15, 0x0a,
     186,   15,   15,   15, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_OpenNIListener[] = {
    "OpenNIListener\0\0newVisualImage(QImage)\0"
    "newFeatureFlowImage(QImage)\0"
    "newDepthImage(QImage)\0"
    "newTransformationMatrix(QString)\0"
    "message\0setGUIInfo(QString)\0"
    "setGUIStatus(QString)\0togglePause()\0"
    "getOneFrame()\0"
};

const QMetaObject OpenNIListener::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_OpenNIListener,
      qt_meta_data_OpenNIListener, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &OpenNIListener::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *OpenNIListener::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *OpenNIListener::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_OpenNIListener))
        return static_cast<void*>(const_cast< OpenNIListener*>(this));
    return QObject::qt_metacast(_clname);
}

int OpenNIListener::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: newVisualImage((*reinterpret_cast< QImage(*)>(_a[1]))); break;
        case 1: newFeatureFlowImage((*reinterpret_cast< QImage(*)>(_a[1]))); break;
        case 2: newDepthImage((*reinterpret_cast< QImage(*)>(_a[1]))); break;
        case 3: newTransformationMatrix((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 4: setGUIInfo((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 5: setGUIStatus((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 6: togglePause(); break;
        case 7: getOneFrame(); break;
        default: ;
        }
        _id -= 8;
    }
    return _id;
}

// SIGNAL 0
void OpenNIListener::newVisualImage(QImage _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void OpenNIListener::newFeatureFlowImage(QImage _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void OpenNIListener::newDepthImage(QImage _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void OpenNIListener::newTransformationMatrix(QString _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void OpenNIListener::setGUIInfo(QString _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}

// SIGNAL 5
void OpenNIListener::setGUIStatus(QString _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 5, _a);
}
QT_END_MOC_NAMESPACE
