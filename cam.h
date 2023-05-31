#ifndef CAM_H
#define CAM_H

#include "global_defines.h"

#include <librealsense2/rs.hpp>
#include <librealsense2/rs_advanced_mode.h>

#include <QObject>

class CAM : public QObject
{
    Q_OBJECT
public:
    explicit CAM(QObject *parent = nullptr);
    ~CAM();

    // interface func
    void init();

    QString get_sn_l();
    QString get_sn_r();

    cv::Mat get_cur_img_l();
    cv::Mat get_cur_img_r();

    // util func
    Eigen::Matrix4d zyx_tranformation(double x, double y, double z, double rx, double ry, double rz);

    // loop
    std::atomic<bool> grabFlag;
    std::thread* grabThread = NULL;
    void grabLoop();

    // storage
    std::mutex mtx;
    QString sn_l;
    QString sn_r;
    cv::Mat cur_img_l;
    cv::Mat cur_img_r;
    double time_l = 0;
    double time_r = 0;
    std::vector<cv::Vec2d> cur_scan_l;
    std::vector<cv::Vec2d> cur_scan_r;
    std::vector<cv::Vec3d> cur_scan_3d_l;
    std::vector<cv::Vec3d> cur_scan_3d_r;

private:

signals:

};

#endif // CAM_H
