/****************************************************************************
** Meta object code from reading C++ file 'websocket.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.12.8)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../websocket.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'websocket.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.12.8. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_websocket_t {
    QByteArrayData data[16];
    char stringdata0[182];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_websocket_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_websocket_t qt_meta_stringdata_websocket = {
    {
QT_MOC_LITERAL(0, 0, 9), // "websocket"
QT_MOC_LITERAL(1, 10, 9), // "msgSignal"
QT_MOC_LITERAL(2, 20, 0), // ""
QT_MOC_LITERAL(3, 21, 3), // "msg"
QT_MOC_LITERAL(4, 25, 15), // "msgReciveSignal"
QT_MOC_LITERAL(5, 41, 7), // "message"
QT_MOC_LITERAL(6, 49, 13), // "msgSendSignal"
QT_MOC_LITERAL(7, 63, 4), // "open"
QT_MOC_LITERAL(8, 68, 9), // "onTimeout"
QT_MOC_LITERAL(9, 78, 15), // "onNewConnection"
QT_MOC_LITERAL(10, 94, 8), // "onClosed"
QT_MOC_LITERAL(11, 103, 21), // "onTextMessageReceived"
QT_MOC_LITERAL(12, 125, 23), // "onBinaryMessageReceived"
QT_MOC_LITERAL(13, 149, 12), // "MissionCheck"
QT_MOC_LITERAL(14, 162, 4), // "uuid"
QT_MOC_LITERAL(15, 167, 14) // "onDisconnected"

    },
    "websocket\0msgSignal\0\0msg\0msgReciveSignal\0"
    "message\0msgSendSignal\0open\0onTimeout\0"
    "onNewConnection\0onClosed\0onTextMessageReceived\0"
    "onBinaryMessageReceived\0MissionCheck\0"
    "uuid\0onDisconnected"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_websocket[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
      11,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       3,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   69,    2, 0x06 /* Public */,
       4,    1,   72,    2, 0x06 /* Public */,
       6,    1,   75,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       7,    0,   78,    2, 0x0a /* Public */,
       8,    0,   79,    2, 0x0a /* Public */,
       9,    0,   80,    2, 0x0a /* Public */,
      10,    0,   81,    2, 0x0a /* Public */,
      11,    1,   82,    2, 0x0a /* Public */,
      12,    1,   85,    2, 0x0a /* Public */,
      13,    1,   88,    2, 0x0a /* Public */,
      15,    0,   91,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void, QMetaType::Bool,    3,
    QMetaType::Void, QMetaType::QString,    5,
    QMetaType::Void, QMetaType::QString,    5,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,    5,
    QMetaType::Void, QMetaType::QByteArray,    5,
    QMetaType::Void, QMetaType::QString,   14,
    QMetaType::Void,

       0        // eod
};

void websocket::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<websocket *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->msgSignal((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 1: _t->msgReciveSignal((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 2: _t->msgSendSignal((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 3: _t->open(); break;
        case 4: _t->onTimeout(); break;
        case 5: _t->onNewConnection(); break;
        case 6: _t->onClosed(); break;
        case 7: _t->onTextMessageReceived((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 8: _t->onBinaryMessageReceived((*reinterpret_cast< QByteArray(*)>(_a[1]))); break;
        case 9: _t->MissionCheck((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 10: _t->onDisconnected(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (websocket::*)(bool );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&websocket::msgSignal)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (websocket::*)(QString );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&websocket::msgReciveSignal)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (websocket::*)(QString );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&websocket::msgSendSignal)) {
                *result = 2;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject websocket::staticMetaObject = { {
    &QObject::staticMetaObject,
    qt_meta_stringdata_websocket.data,
    qt_meta_data_websocket,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *websocket::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *websocket::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_websocket.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int websocket::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 11)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 11;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 11)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 11;
    }
    return _id;
}

// SIGNAL 0
void websocket::msgSignal(bool _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void websocket::msgReciveSignal(QString _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void websocket::msgSendSignal(QString _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
