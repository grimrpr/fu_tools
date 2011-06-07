/****************************************************************************
** Meta object code from reading C++ file 'qtros.h'
**
** Created: Tue Jun 7 16:50:11 2011
**      by: The Qt Meta Object Compiler version 62 (Qt 4.7.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../src/qtros.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'qtros.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 62
#error "This file was generated using the moc from 4.7.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_QtROS[] = {

 // content:
       5,       // revision
       0,       // classname
       0,    0, // classinfo
       2,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: signature, parameters, type, tag, flags
       7,    6,    6,    6, 0x05,

 // slots: signature, parameters, type, tag, flags
      18,    6,    6,    6, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_QtROS[] = {
    "QtROS\0\0rosQuits()\0quitNow()\0"
};

const QMetaObject QtROS::staticMetaObject = {
    { &QThread::staticMetaObject, qt_meta_stringdata_QtROS,
      qt_meta_data_QtROS, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &QtROS::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *QtROS::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *QtROS::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_QtROS))
        return static_cast<void*>(const_cast< QtROS*>(this));
    return QThread::qt_metacast(_clname);
}

int QtROS::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QThread::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: rosQuits(); break;
        case 1: quitNow(); break;
        default: ;
        }
        _id -= 2;
    }
    return _id;
}

// SIGNAL 0
void QtROS::rosQuits()
{
    QMetaObject::activate(this, &staticMetaObject, 0, 0);
}
QT_END_MOC_NAMESPACE
