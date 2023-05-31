#ifndef AUTOCONTROL_H
#define AUTOCONTROL_H

// defines
#include "global_defines.h"

#include <QObject>

#include "mobile.h"
#include "slam_2d.h"
#include "unimap.h"

#include "astar.h"
#include "spline.h"

class AUTOCONTROL : public QObject
{
    Q_OBJECT
public:
    explicit AUTOCONTROL(QObject *parent = nullptr);
    ~AUTOCONTROL();

    // interface funcs
    void init(MOBILE *_mobile, SLAM_2D *_slam, UNIMAP *_unimap);    
    std::vector<PATH_POINT> get_cur_path();
    std::vector<cv::Vec3d> get_goal_list();
    std::vector<ROBOT_SHARED> get_robots();

    void add_waypoint(cv::Vec3d wp);
    void ofsm_run(cv::Vec3d goal);
    void ifsm_run(cv::Vec3d goal, bool is_avoid, bool is_align);
    void ifsm_stop();
    void shared_path_callback(const lcm::ReceiveBuffer *rbuf, const std::string &chan, const shared_path *msg);


    // util funcs
    double sgn(double val);
    double saturation(double val, double min, double max);
    std::vector<cv::Vec2d> transform(std::vector<cv::Vec2d> pts, cv::Vec3d xi);
    double calc_point_to_line_segment_dist(cv::Vec2d P0, cv::Vec2d P1, cv::Vec2d P);
    std::vector<cv::Vec2i> line_iterator(cv::Vec2i pt0, cv::Vec2i pt1);
    std::vector<cv::Vec2i> filled_circle_iterator(cv::Vec2i pt, int r);

    // path util
    int find_nn_path_idx(cv::Vec2d pt, std::vector<PATH_POINT>& path);
    double find_nn_path_dist(cv::Vec2d pt, std::vector<PATH_POINT>& path);
    cv::Vec2d find_nn_path_point(cv::Vec2d pt, std::vector<PATH_POINT>& path);
    std::vector<cv::Vec2i> path_simplify(cv::Mat& cost, std::vector<cv::Vec2i>& path);
    void path_smoother(std::vector<cv::Vec2d> &path);

    // for path
    std::vector<PATH_POINT> calc_path(cv::Vec3d st_pose, cv::Vec3d ed_pose, std::vector<cv::Vec2d> cur_scan, bool avoid);
    std::vector<double> median_filter(std::vector<double>& src, int mask);
    std::vector<double> erode_filter(std::vector<double>& src, int mask);
    std::vector<double> dilate_filter(std::vector<double>& src, int mask);
    std::vector<double> average_filter(std::vector<double>& src, int mask);
    std::vector<double> smoothing_ref_v(std::vector<double>& src);

    // for control
    cv::Vec2d get_look_ahead_point(cv::Vec2d pt, double r, std::vector<PATH_POINT>& path);
    int get_look_ahead_idx(int cur_idx, double r, std::vector<PATH_POINT>& path);
    double calc_motion_time(double _s, double _v0, double _v1, double _acc);
    cv::Vec2d calc_pp(cv::Vec3d cur_pose, cv::Vec2d cur_vw, std::vector<PATH_POINT>& path, double dt);

    // for obstacle
    std::vector<cv::Vec6d> calc_trajectory(cv::Vec2d vw, double predict_time, double dt, cv::Vec3d cur_pose, cv::Vec2d cur_vw);    
    void marking_obstacle(int cur_idx, std::vector<PATH_POINT>& path, double deadzone);
    void clear_obstacle(std::vector<PATH_POINT>& path);

    // storage
    std::mutex mtx;    
    std::vector<cv::Vec3d> goal_list;
    std::vector<ROBOT_SHARED> robots;

    // path
    std::vector<PATH_POINT> cur_path;

    // flags
    std::atomic<bool> is_pause;
    std::atomic<int> ifsm_state;
    std::atomic<int> ofsm_state;

    // variable
    std::atomic<double> whole_gain;

private:    
    MOBILE *mobile = NULL;
    SLAM_2D *slam = NULL;
    UNIMAP *unimap = NULL;

    // inner state machine
    std::atomic<bool> ifsmFlag;
    std::thread* ifsmThread = NULL;
    void ifsmLoop(cv::Vec3d _goal_pose, bool is_avoid, bool is_align);

    // outer state machine
    std::atomic<bool> ofsmFlag;
    std::thread* ofsmThread = NULL;
    void ofsmLoop();

};

#endif // AUTOCONTROL_H
