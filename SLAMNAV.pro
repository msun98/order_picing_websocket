QT       += core gui network

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++14
CONFIG += optimize_full

# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    Logger.cpp \
    astar.cpp \
    autocontrol.cpp \
    cam.cpp \
    cv_to_qt.cpp \
    ipc.cpp \
    l2c.cpp \
    lidar_2d.cpp \
    main.cpp \
    mainwindow.cpp \
    mobile.cpp \
    mygraphicsview.cpp \
    p2o.cpp \
    sim.cpp \
    slam_2d.cpp \
    submap.cpp \
    icp/icp.cpp \
    icp/icpPointToPlane.cpp \
    icp/icpPointToPoint.cpp \
    icp/kdtree.cpp \
    icp/matrix.cpp \
    topomap.cpp \
    unimap.cpp

HEADERS += \
    Logger.h \
    astar.h \
    autocontrol.h \
    cam.h \
    cv_to_qt.h \
    global_defines.h \
    ipc.h \
    l2c.h \
    lidar_2d.h \
    mainwindow.h \
    mobile.h \
    mygraphicsview.h \
    p2o.h \
    sim.h \
    slam_2d.h \
    spline.h \
    submap.h \
    icp/icp.h \
    icp/icpPointToPlane.h \
    icp/icpPointToPoint.h \
    icp/kdtree.h \
    icp/matrix.h \
    topomap.h \
    unimap.h

FORMS += \
    mainwindow.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

# Libraries setting (for x86_64)
contains(QT_ARCH, x86_64){
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

    # Eigen and Sophus library
    INCLUDEPATH += /usr/include/eigen3/
    INCLUDEPATH += /usr/local/include/sophus/

    # TBB
    LIBS += -L/usr/lib/x86_64-linux-gnu/
    LIBS += -ltbb

    # OpenMP
    QMAKE_CXXFLAGS += -fopenmp
    QMAKE_CFLAGS += -fopenmp
    LIBS += -L/usr/lib/x86_64-linux-gnu/
    LIBS += -lpthread
    LIBS += -lgomp

    # rplidar sdk
#    INCLUDEPATH += /home/tw/rplidar_sdk/sdk/include/
#    LIBS += -L/home/tw/rplidar_sdk/output/Linux/Release/
    #INCLUDEPATH += /home/rainbow/rplidar_sdk/sdk/include/
    #LIBS += -L/home/rainbow/rplidar_sdk/output/Linux/Release/
    INCLUDEPATH += /home/rainbow/rplidar_sdk-master/sdk/include/
    LIBS += -L/home/rainbow/rplidar_sdk-master/output/Linux/Release/
    LIBS += -lsl_lidar_sdk

    # RealsenseSDK
    INCLUDEPATH += /usr/local/include/
    LIBS += -L/usr/local/lib/
    LIBS += -lrealsense2

    # LCM
    INCLUDEPATH += /usr/local/include/lcm/
    LIBS += -L/usr/local/lib/
    LIBS += -llcm
}

# Libraries setting (for aarch64)
contains(QT_ARCH, arm64) {
    # OpenCV library all
    INCLUDEPATH += /usr/include/opencv4/
    LIBS += -L/usr/lib/aarch64-linux-gnu/
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

    # Eigen and Sophus library
    INCLUDEPATH += /usr/include/eigen3/
    INCLUDEPATH += /usr/local/include/sophus/

    # TBB
    LIBS += -L/usr/lib/aarch64-linux-gnu/
    LIBS += -ltbb

    # OpenMP
    QMAKE_CXXFLAGS += -fopenmp
    QMAKE_CFLAGS += -fopenmp
    LIBS += -L/usr/lib/aarch64-linux-gnu/
    LIBS += -lpthread
    LIBS += -lgomp

    # rplidar sdk
    INCLUDEPATH += /home/odroid/rplidar_sdk/sdk/include/
    LIBS += -L/home/odroid/rplidar_sdk/output/Linux/Release/
    LIBS += -lsl_lidar_sdk

    # RealsenseSDK
    INCLUDEPATH += /usr/local/include/
    LIBS += -L/usr/local/lib/
    LIBS += -lrealsense2

    # LCM
    INCLUDEPATH += /usr/include/lcm/
    LIBS += -L/usr/lib/aarch64-linux-gnu/
    LIBS += -llcm
}
