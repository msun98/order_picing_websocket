QT       += core gui websockets

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11

# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

# Libraries setting (for x86_64)
#contains(QT_ARCH, x86_64){

    # OpenCV library all
    INCLUDEPATH += /usr/include/opencv4/
    LIBS += -L/usr/lib/x86_64-linux-gnu/
    LIBS += -lopencv_core \
        -lopencv_highgui \
        -lopencv_imgcodecs \
        -lopencv_imgproc \
        -lopencv_calib3d \
        -lopencv_features2d \
        -lopencv_flann \
        -lopencv_objdetect \
        -lopencv_photo \
        -lopencv_video \
        -lopencv_videoio \
        -lboost_system \
        -lopencv_ximgproc

SOURCES += \
    cv_to_qt.cpp \
    ipc.cpp \
    main.cpp \
    mainwindow.cpp \
    websocket.cpp

HEADERS += \
    cv_to_qt.h \
    global_defines.h \
    ipc.h \
    mainwindow.h \
    websocket.h

FORMS += \
    mainwindow.ui\
    mainwindow.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target



# TBB
LIBS += -L/usr/lib/x86_64-linux-gnu/
LIBS += -ltbb

DISTFILES += \
    resource/rainbow.png \
    resource/yujin.png

RESOURCES += \
    resource/resource.qrc
