/****************************************************************************
** Meta object code from reading C++ file 'graph_manager.h'
**
** Created: Tue Jun 7 16:19:14 2011
**      by: The Qt Meta Object Compiler version 62 (Qt 4.7.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../src/graph_manager.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'graph_manager.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 62
#error "This file was generated using the moc from 4.7.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_GraphManager[] = {

 // content:
       5,       // revision
       0,       // classname
       0,    0, // classinfo
      15,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       9,       // signalCount

 // signals: signature, parameters, type, tag, flags
      14,   13,   13,   13, 0x05,
      47,   13,   13,   13, 0x05,
      70,   62,   13,   13, 0x05,
      90,   62,   13,   13, 0x05,
     130,  112,   13,   13, 0x05,
     195,  179,   13,   13, 0x05,
     232,   62,   13,   13, 0x05,
     263,  253,   13,   13, 0x05,
     302,   13,   13,   13, 0x05,

 // slots: signature, parameters, type, tag, flags
     319,   13,   13,   13, 0x0a,
     327,   13,   13,   13, 0x0a,
     357,  343,   13,   13, 0x0a,
     396,  387,   13,   13, 0x0a,
     419,   13,   13,   13, 0x0a,
     447,  437,   13,   13, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_GraphManager[] = {
    "GraphManager\0\0newTransformationMatrix(QString)\0"
    "sendFinished()\0message\0setGUIInfo(QString)\0"
    "setGUIStatus(QString)\0pc,transformation\0"
    "setPointCloud(const pointcloud_type*,QMatrix4x4)\0"
    "transformations\0updateTransforms(QList<QMatrix4x4>*)\0"
    "setGUIInfo2(QString)\0edge_list\0"
    "setGraphEdges(QList<QPair<int,int> >*)\0"
    "deleteLastNode()\0reset()\0sendAllClouds()\0"
    "file_basename\0saveIndividualClouds(QString)\0"
    "filename\0saveAllClouds(QString)\0"
    "deleteLastFrame()\0max_depth\0"
    "setMaxDepth(float)\0"
};

const QMetaObject GraphManager::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_GraphManager,
      qt_meta_data_GraphManager, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &GraphManager::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *GraphManager::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *GraphManager::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_GraphManager))
        return static_cast<void*>(const_cast< GraphManager*>(this));
    return QObject::qt_metacast(_clname);
}

int GraphManager::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: newTransformationMatrix((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 1: sendFinished(); break;
        case 2: setGUIInfo((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 3: setGUIStatus((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 4: setPointCloud((*reinterpret_cast< const pointcloud_type*(*)>(_a[1])),(*reinterpret_cast< QMatrix4x4(*)>(_a[2]))); break;
        case 5: updateTransforms((*reinterpret_cast< QList<QMatrix4x4>*(*)>(_a[1]))); break;
        case 6: setGUIInfo2((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 7: setGraphEdges((*reinterpret_cast< QList<QPair<int,int> >*(*)>(_a[1]))); break;
        case 8: deleteLastNode(); break;
        case 9: reset(); break;
        case 10: sendAllClouds(); break;
        case 11: saveIndividualClouds((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 12: saveAllClouds((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 13: deleteLastFrame(); break;
        case 14: setMaxDepth((*reinterpret_cast< float(*)>(_a[1]))); break;
        default: ;
        }
        _id -= 15;
    }
    return _id;
}

// SIGNAL 0
void GraphManager::newTransformationMatrix(QString _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void GraphManager::sendFinished()
{
    QMetaObject::activate(this, &staticMetaObject, 1, 0);
}

// SIGNAL 2
void GraphManager::setGUIInfo(QString _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void GraphManager::setGUIStatus(QString _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void GraphManager::setPointCloud(pointcloud_type const * _t1, QMatrix4x4 _t2)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}

// SIGNAL 5
void GraphManager::updateTransforms(QList<QMatrix4x4> * _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 5, _a);
}

// SIGNAL 6
void GraphManager::setGUIInfo2(QString _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 6, _a);
}

// SIGNAL 7
void GraphManager::setGraphEdges(QList<QPair<int,int> > * _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 7, _a);
}

// SIGNAL 8
void GraphManager::deleteLastNode()
{
    QMetaObject::activate(this, &staticMetaObject, 8, 0);
}
QT_END_MOC_NAMESPACE
