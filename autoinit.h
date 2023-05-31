#ifndef AUTOINIT_H
#define AUTOINIT_H

#include "global_defines.h"

#include "lidar_2d.h"
#include "mobile.h"
#include "slam_2d.h"
#include "unimap.h"

#include <QObject>

class AUTOINIT : public QObject
{
    Q_OBJECT
public:
    explicit AUTOINIT(QObject *parent = nullptr);

    void init(LIDAR_2D *_lidar, MOBILE *_mobile, SLAM_2D *_slam, UNIMAP *_unimap);
    bool autoinit();

private:
    LIDAR_2D *lidar = NULL;
    MOBILE *mobile = NULL;
    SLAM_2D *slam = NULL;
    UNIMAP *unimap = NULL;

signals:


};

#endif // AUTOINIT_H
