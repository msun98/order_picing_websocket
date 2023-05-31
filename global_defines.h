#ifndef GLOBAL_DEFINES_H
#define GLOBAL_DEFINES_H

#define USE_SIM//simulation mode on
//#define USE_EX_TEMP

// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>

// sophus
#define SOPHUS_USE_BASIC_LOGGING
#include <sophus/geometry.hpp>
#include <sophus/se2.hpp>
#include <sophus/interpolate.hpp>

// stl
#include <vector>
#include <stack>
#include <mutex>
#include <thread>
#include <tbb/concurrent_queue.h>
#include <tbb/concurrent_vector.h>

// opencv
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/ximgproc.hpp>
#include "opencv2/flann/miniflann.hpp"
#include "cv_to_qt.h"

// lcm
#include <lcm-cpp.hpp>
#include "lcm_rb/lcm_rb.h"

// logger
#include "Logger.h"
extern Logger logger;

// qt
#include <QString>

// defines
#define N2S 1.0e-9
#define S2N 1.0e9
#define D2R (M_PI/180.0)
#define R2D (180.0/M_PI)
#define toWrap(rad) (std::atan2(std::sin(rad), std::cos(rad)))
#define deltaRad(ed,st) (std::atan2(std::sin(ed - st), std::cos(ed - st)))

// for algorithm
#define P_hit 0.55
#define P_miss 0.49
#define P_min 0.1
#define P_max 0.95
#define WALL_THRESHOLD 0.8
#define SUBMAP_MAX_COUNT 50

enum UI_MOTOR_STATE
{
    UI_MOTOR_NOT_READY = 0,
    UI_MOTOR_READY,
};

enum UI_LOC_STATE
{
    UI_LOC_NOT_READY = 0,
    UI_LOC_BUSY,
    UI_LOC_GOOD,
    UI_LOC_FAIL
};

enum UI_AUTO_STATE
{
    UI_AUTO_NOT_READY = 0,
    UI_AUTO_READY,
    UI_AUTO_MOVING,
    UI_AUTO_WAIT,
    UI_AUTO_PAUSE
};

enum UI_OBS_STATE
{
    UI_OBS_NONE = 0,
    UI_OBS_NEAR
};

enum COMMAND
{
    CMD_MOVE_LOCATION = 1,
    CMD_MOVE_TARGET,
    CMD_MOVE_JOYSTICK,
    CMD_MOVE_MANUAL,
    CMD_MOVE_STOP, //5
    CMD_PAUSE,
    CMD_RESUME,
    CMD_SET_VEL,
    CMD_RESTART,
    CMD_SET_INIT, //10
    CMD_LOC_RUN,
    CMD_LOC_STOP,
    CMD_LOC_AUTO,
    CMD_MAPPING_START,
    CMD_MAPPING_STOP, //15    
    CMD_REQ_CAMERA,
    CMD_MAP_SAVE,
    CMD_OBS_START,
    CMD_OBS_STOP,
    CMD_OBS_SAVE
};

enum INNER_FSM_STATE
{
    STATE_PATH_FINDING = 0,
    STATE_FIRST_ALIGN,
    STATE_PURE_PURSUIT,
    STATE_FINAL_ALIGN,
    STATE_GOAL_REACHED,
    STATE_OBSTACLE,
    STATE_WAIT,
    STATE_AVOID,
    STATE_PATH_OUT,
    STATE_PATH_FAILED,
    STATE_STOPPED,
    STATE_PAUSE
};

enum OUTER_FSM_STATE
{
    STATE_IDLE = 0,
    STATE_DRIVING,
    STATE_WAITING,
    STATE_FAILED
};

enum LOCATIONS
{
    LOCATION_SERVING = 0,
    LOCATION_PATROL,
    LOCATION_RESTING,
    LOCATION_CHARGING
};

// structures
struct ROBOT_CONFIG
{
    // read file
    QString map_name = "";
    QString map_path = "";

    QString robot_model = "C";
    QString robot_sn = "1";
    int robot_tn = 3;
    QString robot_type = "SERVING";

    int robot_id = 0;
    int robot_map_size = 1000;
    double robot_grid_size = 0.03;
    double robot_radius = 0.35;
    double robot_wheel_base = 0.3542;
    double robot_wheel_radius = 0.0635;

    double robot_look_ahead_dist = 0.5;
    double robot_limit_v = 0.6;
    double robot_limit_w = 90.0*D2R;
    double robot_limit_v_acc = 0.5;
    double robot_limit_w_acc = 360.0*D2R;
    double robot_limit_pivot = 30.0*D2R;
    double robot_limit_pivot_acc = 10.0*D2R;
    double robot_k_v = 1.0;
    double robot_k_w = 1.0;    
    double robot_vel_gain = 1.0;    
    double robot_manual_limit_v = 0.3;
    double robot_manual_limit_w = 30.0*D2R;

    bool robot_use_multi = false;
    double robot_icp_dist = 0.5;
    double robot_icp_near = 1.0;
    double robot_icp_error = 0.1;
    double robot_icp_ratio = 0.5;
    double robot_icp_repeat_time = 1.0;
    double robot_icp_repeat_dist = 0.15;
    double robot_icp_odometry_weight = 0.5;
    double robot_obs_magin = 0.1;
    double robot_obs_deadzone = 0.3;
    double robot_obs_wait_time = 5.0;
    double robot_goal_near_dist = 0.1;
    double robot_goal_near_th = 3.0*D2R;
    double robot_goal_dist = 0.01;
    double robot_goal_th = 1.0*D2R;
    double robot_goal_v = 0.05;
    double robot_st_v = 0.1;
    double robot_path_out_dist = 1.0;
    double robot_narrow_decel_ratio = 0.5;
    double robot_min_look_ahead_dist = 0.05;

    int robot_lidar_baudrate = 256000;
    double robot_lidar_offset_x = 0.0;
    double robot_lidar_offset_y = 0.0;
    double robot_lidar_max_range = 40.0;
    double robot_lidar_mask = 10.0; // deg

    bool robot_cmd_accept = true;
    bool robot_use_avoid = false;

    int motor_right_id = 0;
    int motor_left_id = 1;
    double motor_wheel_dir = -1.0;
    double motor_gear_ratio = 1.0;    
    double motor_k_p = 100.0;
    double motor_k_i = 0.0;
    double motor_k_d = 5000.0;
    double motor_limit_v = 2.0;
    double motor_limit_v_acc = 1.5;
    double motor_limit_w = 360.0*D2R;
    double motor_limit_w_acc = 360*D2R;

    double cam_exposure = 2000.0;
    QString cam_left_sn = "";
    QString cam_left_tf = "";
    QString cam_right_sn = "";
    QString cam_right_tf = "";
};
extern ROBOT_CONFIG robot_config;

struct MOBILE_POSE
{
    double t;
    cv::Vec3d pose; // global (x, y, th)
    cv::Vec3d vel; // global (x_dot, y_dot, th_dot)
    cv::Vec2d vw; // local (v, w)

    MOBILE_POSE()
    {
        t = 0;
        pose = cv::Vec3d(0,0,0);
        vel = cv::Vec3d(0,0,0);
        vw = cv::Vec2d(0,0);
    }
    MOBILE_POSE(const MOBILE_POSE& p)
    {
        t = p.t;
        pose = p.pose;
        vel = p.vel;
        vw = p.vw;
    }
};

struct MOBILE_STATUS
{
    bool is_ok = false;

    uint8_t connection_m0 = 0;
    uint8_t connection_m1 = 0;

    uint8_t status_m0 = 0;
    uint8_t status_m1 = 0;

    uint8_t temp_m0 = 0;
    uint8_t temp_m1 = 0;

    uint8_t temp_ex_m0 = 0;
    uint8_t temp_ex_m1 = 0;

    uint8_t cur_m0 = 0;
    uint8_t cur_m1 = 0;

    uint8_t charge_state = 0;
    uint8_t power_state = 0;
    uint8_t emo_state = 0;
    uint8_t remote_state = 0;

    float bat_in = 0;
    float bat_out = 0;
    float bat_current = 0;
    float power = 0;
    float total_power = 0;
};

struct LIDAR_FRM
{
    double t = 0;
    double t0 = 0;
    double t1 = 0;
    std::vector<double> ts;    
    std::vector<cv::Vec2d> pts;
    cv::Vec3d mobile_pose;
    cv::Vec3d pre_mobile_pose;

    LIDAR_FRM()
    {
        mobile_pose = cv::Vec3d(0,0,0);
        pre_mobile_pose = cv::Vec3d(0,0,0);
    }
    LIDAR_FRM(const LIDAR_FRM& p)
    {
        t = p.t;
        t0 = p.t0;
        t1 = p.t1;
        ts = p.ts;        
        pts = p.pts;
        mobile_pose = p.mobile_pose;
        pre_mobile_pose = p.pre_mobile_pose;
    }
};

struct LOC
{
    cv::Vec3d pose;
    cv::Vec3d mobile_pose;
    LOC()
    {
        pose = cv::Vec3d(0,0,0);
        mobile_pose = cv::Vec3d(0,0,0);
    }
    LOC(const LOC& p)
    {
        pose = p.pose;
        mobile_pose = p.mobile_pose;
    }
};

struct CORRESPONDENCE
{
    cv::Vec2d P0;
    cv::Vec2d P1;
    double sq_d = 0;

    CORRESPONDENCE()
    {
        P0 = cv::Vec2d(0,0);
        P1 = cv::Vec2d(0,0);
    }
    CORRESPONDENCE(const CORRESPONDENCE& p)
    {
        P0 = p.P0;
        P1 = p.P1;
        sq_d = p.sq_d;
    }
};

struct LC_INFO
{
    int id0 = 0;
    int id1 = 0;
    cv::Vec3d dxi;

    LC_INFO()
    {
        dxi = cv::Vec3d(0,0,0);
    }
    LC_INFO(const LC_INFO& p)
    {
        id0 = p.id0;
        id1 = p.id1;
        dxi = p.dxi;
    }
};

struct PATH_POINT
{
    cv::Vec2d pt;
    double dt = 0;
    double od = 0;
    double th = 0;
    double dth = 0;
    double v = 0;
    int obs = 0;
    double obs_v = 0;

    PATH_POINT()
    {
        pt = cv::Vec2d(0, 0);
        dt = 0;
        od = 0;
        th = 0;
        dth = 0;
        v = 0;
        obs = 0;
        obs_v = 0;
    }
    PATH_POINT(const PATH_POINT& p)
    {
        pt = p.pt;
        dt = p.dt;
        od = p.od;
        th = p.th;
        dth = p.dth;
        v = p.v;
        obs = p.obs;
        obs_v = p.obs_v;
    }
};

struct TIME_POSE
{
    double t = 0;
    cv::Vec3d pose;

    TIME_POSE()
    {
        pose = cv::Vec3d(0,0,0);
    }
    TIME_POSE(const TIME_POSE& p)
    {
        t = p.t;
        pose = p.pose;
    }
};

struct ROBOT_SHARED
{
    int id = -1;
    double t = 0;
    std::vector<cv::Vec2d> path;

    ROBOT_SHARED()
    {
        id = -1;
        t = 0;
    }
    ROBOT_SHARED(const ROBOT_SHARED& p)
    {
        id = p.id;        
        t = p.t;
        path = p.path;        
    }
};

extern double st_time_for_get_time;
double get_time();

#endif // GLOBAL_DEFINES_H



