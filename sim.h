#ifndef SIM_H
#define SIM_H

#include "global_defines.h"

#include "mobile.h"
#include "lidar_2d.h"
#include "cam.h"

#include <QObject>
#include <QTimer>
#include <QFileInfo>

class SIM : public QObject
{
    Q_OBJECT
public:
    explicit SIM(QObject *parent = nullptr);
    ~SIM();
    void init(MOBILE *_mobile, LIDAR_2D *_lidar, CAM *_cam);
    void load_map(QString path);

    // comm loop
    std::atomic<bool> simFlag;
    std::thread* simThread = NULL;
    void simLoop();

private:
    double sgn(double val);
    double saturation(double val, double min, double max);
    cv::Vec2i xy_uv(cv::Vec2d P);
    cv::Vec2d uv_xy(cv::Vec2i uv);
    std::vector<cv::Vec2i> line_iterator(cv::Vec2i pt0, cv::Vec2i pt1);
    std::vector<cv::Vec2d> calc_lidar_scan(cv::Vec3d xi);

    std::vector<cv::Vec2d> lidar_pattern;

    std::atomic<bool> is_pause;
    QTimer sim_timer;
    double dt = 0.05;
    double sim_time = 0;
    double sim_grid_width = 0.03;
    int sim_cnt = 0;
    cv::Vec6d state;
    cv::Vec3d pre_mobile_pose;

    MOBILE *mobile;
    LIDAR_2D *lidar;
    CAM *cam;
    cv::Mat sim_map;

signals:

};

#endif // SIM_H
