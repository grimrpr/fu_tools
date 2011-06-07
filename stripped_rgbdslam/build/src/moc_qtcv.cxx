/****************************************************************************
** Meta object code from reading C++ file 'qtcv.h'
**
** Created: Tue Jun 7 15:45:51 2011
**      by: The Qt Meta Object Compiler version 62 (Qt 4.7.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../src/qtcv.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'qtcv.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 62
#error "This file was generated using the moc from 4.7.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_UserInterface[] = {

 // content:
       5,       // revision
       0,       // classname
       0,    0, // classinfo
      35,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       8,       // signalCount

 // signals: signature, parameters, type, tag, flags
      15,   14,   14,   14, 0x05,
      23,   14,   14,   14, 0x05,
      37,   14,   14,   14, 0x05,
      51,   14,   14,   14, 0x05,
      69,   14,   14,   14, 0x05,
      94,   85,   14,   14, 0x05,
     131,  117,   14,   14, 0x05,
     171,  161,   14,   14, 0x05,

 // slots: signature, parameters, type, tag, flags
     190,   14,   14,   14, 0x0a,
     213,   14,   14,   14, 0x0a,
     241,   14,   14,   14, 0x0a,
     263,   14,   14,   14, 0x0a,
     290,   14,   14,   14, 0x0a,
     318,  305,   14,   14, 0x0a,
     378,  367,   14,   14, 0x0a,
     420,  415,   14,   14, 0x0a,
     459,   14,   14,   14, 0x0a,
     476,   14,   14,   14, 0x08,
     487,   14,   14,   14, 0x08,
     497,   14,   14,   14, 0x08,
     506,   14,   14,   14, 0x08,
     516,   14,   14,   14, 0x08,
     533,   14,   14,   14, 0x08,
     548,   14,   14,   14, 0x08,
     560,   14,   14,   14, 0x08,
     568,   14,   14,   14, 0x08,
     575,   14,   14,   14, 0x08,
     602,   14,   14,   14, 0x08,
     619,   14,   14,   14, 0x08,
     637,   14,   14,   14, 0x08,
     656,   14,   14,   14, 0x08,
     673,   14,   14,   14, 0x08,
     700,  694,   14,   14, 0x08,
     719,  694,   14,   14, 0x08,
     737,   14,   14,   14, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_UserInterface[] = {
    "UserInterface\0\0reset()\0togglePause()\0"
    "getOneFrame()\0deleteLastFrame()\0"
    "sendAllClouds()\0filename\0"
    "saveAllClouds(QString)\0file_basename\0"
    "saveIndividualClouds(QString)\0max_depth\0"
    "setMaxDepth(float)\0setVisualImage(QImage)\0"
    "setFeatureFlowImage(QImage)\0"
    "setDepthImage(QImage)\0setTransformation(QString)\0"
    "sendFinished()\0pc,transform\0"
    "addPointCloud(const pointcloud_type*,QMatrix4x4)\0"
    "transforms\0updateTransforms(QList<QMatrix4x4>*)\0"
    "list\0setGraphEdges(QList<QPair<int,int> >*)\0"
    "deleteLastNode()\0resetCmd()\0sendAll()\0"
    "setMax()\0saveAll()\0saveIndividual()\0"
    "quickSaveAll()\0pause(bool)\0about()\0"
    "help()\0lastTransformationMatrix()\0"
    "setInfo(QString)\0setInfo2(QString)\0"
    "setStatus(QString)\0getOneFrameCmd()\0"
    "deleteLastFrameCmd()\0is_on\0"
    "set3DDisplay(bool)\0set2DStream(bool)\0"
    "toggleTriangulation()\0"
};

const QMetaObject UserInterface::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_UserInterface,
      qt_meta_data_UserInterface, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &UserInterface::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *UserInterface::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *UserInterface::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_UserInterface))
        return static_cast<void*>(const_cast< UserInterface*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int UserInterface::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: reset(); break;
        case 1: togglePause(); break;
        case 2: getOneFrame(); break;
        case 3: deleteLastFrame(); break;
        case 4: sendAllClouds(); break;
        case 5: saveAllClouds((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 6: saveIndividualClouds((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 7: setMaxDepth((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 8: setVisualImage((*reinterpret_cast< QImage(*)>(_a[1]))); break;
        case 9: setFeatureFlowImage((*reinterpret_cast< QImage(*)>(_a[1]))); break;
        case 10: setDepthImage((*reinterpret_cast< QImage(*)>(_a[1]))); break;
        case 11: setTransformation((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 12: sendFinished(); break;
        case 13: addPointCloud((*reinterpret_cast< const pointcloud_type*(*)>(_a[1])),(*reinterpret_cast< QMatrix4x4(*)>(_a[2]))); break;
        case 14: updateTransforms((*reinterpret_cast< QList<QMatrix4x4>*(*)>(_a[1]))); break;
        case 15: setGraphEdges((*reinterpret_cast< QList<QPair<int,int> >*(*)>(_a[1]))); break;
        case 16: deleteLastNode(); break;
        case 17: resetCmd(); break;
        case 18: sendAll(); break;
        case 19: setMax(); break;
        case 20: saveAll(); break;
        case 21: saveIndividual(); break;
        case 22: quickSaveAll(); break;
        case 23: pause((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 24: about(); break;
        case 25: help(); break;
        case 26: lastTransformationMatrix(); break;
        case 27: setInfo((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 28: setInfo2((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 29: setStatus((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 30: getOneFrameCmd(); break;
        case 31: deleteLastFrameCmd(); break;
        case 32: set3DDisplay((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 33: set2DStream((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 34: toggleTriangulation(); break;
        default: ;
        }
        _id -= 35;
    }
    return _id;
}

// SIGNAL 0
void UserInterface::reset()
{
    QMetaObject::activate(this, &staticMetaObject, 0, 0);
}

// SIGNAL 1
void UserInterface::togglePause()
{
    QMetaObject::activate(this, &staticMetaObject, 1, 0);
}

// SIGNAL 2
void UserInterface::getOneFrame()
{
    QMetaObject::activate(this, &staticMetaObject, 2, 0);
}

// SIGNAL 3
void UserInterface::deleteLastFrame()
{
    QMetaObject::activate(this, &staticMetaObject, 3, 0);
}

// SIGNAL 4
void UserInterface::sendAllClouds()
{
    QMetaObject::activate(this, &staticMetaObject, 4, 0);
}

// SIGNAL 5
void UserInterface::saveAllClouds(QString _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 5, _a);
}

// SIGNAL 6
void UserInterface::saveIndividualClouds(QString _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 6, _a);
}

// SIGNAL 7
void UserInterface::setMaxDepth(float _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 7, _a);
}
QT_END_MOC_NAMESPACE
