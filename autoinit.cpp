#include "autoinit.h"

AUTOINIT::AUTOINIT(QObject *parent) : QObject(parent)
{

}

void AUTOINIT::init(LIDAR_2D *_lidar, MOBILE *_mobile, SLAM_2D *_slam, UNIMAP *_unimap)
{
    lidar = _lidar;
    mobile = _mobile;
    slam = _slam;
    unimap = _unimap;
}

bool AUTOINIT::autoinit()
{

}
