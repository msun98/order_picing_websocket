#ifndef UNIMAP_H
#define UNIMAP_H

#include "global_defines.h"
#include "submap.h"

#include <QObject>
#include <QSettings>
#include <QDir>

class UNIMAP : public QObject
{
    Q_OBJECT
public:
    explicit UNIMAP(QObject *parent = nullptr);
    std::mutex mtx;

    void load_map(QString path);
    void save_map(QString path);

    void update_map(SUBMAP* submap);
    void update_map(cv::Mat &_map, SUBMAP* submap);
    void update_obs_map0(std::vector<cv::Vec3d> &pts);
    void update_obs_map(std::vector<cv::Vec2d> &pts);    
    void set_map(cv::Mat &_map);
    void clear_map();

    void draw_robot(cv::Mat &img, cv::Vec3d xi, cv::Scalar c, int tickness);
    void draw_lidar(cv::Mat &img, cv::Vec3d xi, std::vector<cv::Vec2d> pts, cv::Scalar c);
    void draw_path(cv::Mat &img, std::vector<PATH_POINT> path, cv::Scalar c);
    void draw_object(cv::Mat &img, std::vector<cv::Vec2d> obj, cv::Scalar c, int tickness);    
    void draw_point(cv::Mat &img, cv::Vec2d pt);
    void draw_trajectory(cv::Mat &img, std::vector<cv::Vec6d> traj, cv::Scalar c, int tickness);
    void draw_obs_map0(cv::Mat &img);
    void draw_obs_map(cv::Mat &img);
    void draw_other_robot(cv::Mat &img, ROBOT_SHARED &other);
    void draw_waypoints(cv::Mat &img, std::vector<cv::Vec3d> list);
    void edit_location(int type, int idx, cv::Vec3d new_loc);

    cv::Mat get_map_raw();
    cv::Mat get_map_cost();
    cv::Mat get_map_cost0();
    cv::Mat get_map_plot();
    cv::Mat get_map_obs();
    cv::Mat get_map_obs0();
    std::vector<cv::Vec2d> get_locations();
    std::vector<cv::Vec2d> get_map_pts();
    bool is_near_static_obs(cv::Vec2d pt);
    bool is_near_dynamic_obs(cv::Vec2d pt);

    cv::Vec2i xy_uv(cv::Vec2d P, cv::Vec3d xi);
    cv::Vec2i xy_uv(cv::Vec2d P);
    cv::Vec2d xy_uvd(cv::Vec2d P);    
    cv::Vec2d uv_xy(cv::Vec2i uv);
    cv::Vec2d uv_xy(cv::Vec2i uv, cv::Vec3d xi);
    std::vector<cv::Vec2i> filled_circle_iterator(cv::Vec2i pt, int r);

    // annotated locations
    std::atomic<bool> is_loaded;
    QString annotated_loc_path;

    std::vector<cv::Vec3d> serving_loc_list;
    std::vector<cv::Vec3d> patrol_loc_list;
    std::vector<cv::Vec3d> charging_loc_list;
    std::vector<cv::Vec3d> resting_loc_list;
    std::vector<std::vector<cv::Vec2d>> object_poly_list;

    std::vector<QString> serving_loc_name_list;
    std::vector<QString> patrol_loc_name_list;
    std::vector<QString> charging_loc_name_list;
    std::vector<QString> resting_loc_name_list;
    std::vector<QString> object_poly_name_list;

    // map info
    int map_w = robot_config.robot_map_size;
    int map_h = robot_config.robot_map_size;
    int map_ou = robot_config.robot_map_size/2;
    int map_ov = robot_config.robot_map_size/2;
    double map_grid_width = robot_config.robot_grid_size;

    // map image
    cv::Mat raw_map;
    cv::Mat travel_map;
    cv::Mat cost_map;
    cv::Mat cost_map0;
    cv::Mat obs_map;
    cv::Mat obs_map0;

    // map points
    std::vector<cv::Vec2d> map_pts;

private:

signals:

public slots:
};

#endif // UNIMAP_H
