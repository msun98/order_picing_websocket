#include "mainwindow.h"

#include <QApplication>

Q_DECLARE_METATYPE(cv::Mat)
Q_DECLARE_METATYPE(cv::Vec3d)
Q_DECLARE_METATYPE(std::vector<cv::Vec3d>)
Q_DECLARE_METATYPE(MOBILE_POSE)
Q_DECLARE_METATYPE(MOBILE_STATUS)

ROBOT_CONFIG robot_config;
Logger logger;

double st_time_for_get_time = get_time();
double get_time()
{
    std::chrono::time_point<std::chrono::system_clock> t = std::chrono::system_clock::now();
    auto timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(t.time_since_epoch()).count();
    return (timestamp*1.0e-9) - st_time_for_get_time;
}

int main(int argc, char *argv[])
{
    logger.write("[MAIN] program start", true);

    qRegisterMetaType<cv::Mat>();
    qRegisterMetaType<cv::Vec3d>();
    qRegisterMetaType<std::vector<cv::Vec3d>>();
    qRegisterMetaType<MOBILE_POSE>();
    qRegisterMetaType<MOBILE_STATUS>();

    // read config
    QString config_path = QDir::homePath()+"/robot_config.ini";
    QFileInfo config_info(config_path);
    if(config_info.exists() && config_info.isFile())
    {
        QSettings settings(config_path, QSettings::IniFormat);

        robot_config.map_name = settings.value("FLOOR/map_name").toString();
        robot_config.map_path = settings.value("FLOOR/map_path").toString();

        robot_config.robot_model = settings.value("ROBOT_HW/model").toString();
        robot_config.robot_sn = settings.value("ROBOT_HW/serial_num").toString();
        robot_config.robot_type = settings.value("ROBOT_HW/type").toString();
        robot_config.robot_radius = settings.value("ROBOT_HW/radius").toDouble();
        robot_config.robot_wheel_base = settings.value("ROBOT_HW/wheel_base").toDouble();
        robot_config.robot_wheel_radius = settings.value("ROBOT_HW/wheel_radius").toDouble();
        robot_config.robot_tn = settings.value("ROBOT_HW/tray_num").toInt();

        robot_config.robot_id = settings.value("ROBOT_SW/robot_id").toInt();
        robot_config.robot_manual_limit_v = settings.value("ROBOT_SW/limit_manual_v").toDouble();
        robot_config.robot_manual_limit_w = settings.value("ROBOT_SW/limit_manual_w").toDouble()*D2R;
        robot_config.robot_limit_v = settings.value("ROBOT_SW/limit_v").toDouble();
        robot_config.robot_limit_w = settings.value("ROBOT_SW/limit_w").toDouble()*D2R;
        robot_config.robot_limit_v_acc = settings.value("ROBOT_SW/limit_v_acc").toDouble();
        robot_config.robot_limit_w_acc = settings.value("ROBOT_SW/limit_w_acc").toDouble()*D2R;
        robot_config.robot_limit_pivot = settings.value("ROBOT_SW/limit_pivot").toDouble()*D2R;
        robot_config.robot_limit_pivot_acc = settings.value("ROBOT_SW/limit_pivot_acc").toDouble()*D2R;
        robot_config.robot_k_v = settings.value("ROBOT_SW/k_v").toDouble();
        robot_config.robot_k_w = settings.value("ROBOT_SW/k_w").toDouble();        
        robot_config.robot_look_ahead_dist = settings.value("ROBOT_SW/look_ahead_dist").toDouble();
        robot_config.robot_cmd_accept = settings.value("ROBOT_SW/use_uicmd").toBool();
        robot_config.robot_use_avoid = settings.value("ROBOT_SW/use_avoid").toBool();
        robot_config.robot_vel_gain = settings.value("ROBOT_SW/velocity").toDouble();
        robot_config.robot_map_size = settings.value("ROBOT_SW/map_size").toInt();
        robot_config.robot_grid_size = settings.value("ROBOT_SW/grid_size").toDouble();

        robot_config.robot_use_multi = settings.value("ROBOT_SW/use_multirobot").toBool();
        robot_config.robot_icp_dist = settings.value("ROBOT_SW/icp_dist").toDouble();
        robot_config.robot_icp_near = settings.value("ROBOT_SW/icp_near").toDouble();
        robot_config.robot_icp_error = settings.value("ROBOT_SW/icp_error").toDouble();
        robot_config.robot_icp_ratio = settings.value("ROBOT_SW/icp_ratio").toDouble();
        robot_config.robot_icp_repeat_time = settings.value("ROBOT_SW/icp_repeat_time").toDouble();
        robot_config.robot_icp_repeat_dist = settings.value("ROBOT_SW/icp_repeat_dist").toDouble();
        robot_config.robot_icp_odometry_weight = settings.value("ROBOT_SW/icp_odometry_weight").toDouble();
        robot_config.robot_obs_magin = settings.value("ROBOT_SW/obs_magin").toDouble();
        robot_config.robot_obs_deadzone = settings.value("ROBOT_SW/obs_deadzone").toDouble();
        robot_config.robot_obs_wait_time = settings.value("ROBOT_SW/obs_wait_time").toDouble();
        robot_config.robot_goal_near_dist = settings.value("ROBOT_SW/goal_near_dist").toDouble();
        robot_config.robot_goal_near_th = settings.value("ROBOT_SW/goal_near_th").toDouble()*D2R;
        robot_config.robot_goal_dist = settings.value("ROBOT_SW/goal_dist").toDouble();
        robot_config.robot_goal_th = settings.value("ROBOT_SW/goal_th").toDouble()*D2R;
        robot_config.robot_goal_v = settings.value("ROBOT_SW/goal_v").toDouble();
        robot_config.robot_st_v = settings.value("ROBOT_SW/st_v").toDouble();
        robot_config.robot_path_out_dist = settings.value("ROBOT_SW/path_out_dist").toDouble();
        robot_config.robot_narrow_decel_ratio = settings.value("ROBOT_SW/narrow_decel_ratio").toDouble();
        robot_config.robot_min_look_ahead_dist = settings.value("ROBOT_SW/min_look_ahead_dist").toDouble();


        robot_config.motor_gear_ratio = settings.value("MOTOR/gear_ratio").toDouble();
        robot_config.motor_wheel_dir = settings.value("MOTOR/wheel_dir").toDouble();
        robot_config.motor_left_id = settings.value("MOTOR/left_id").toInt();
        robot_config.motor_right_id = settings.value("MOTOR/right_id").toInt();
        robot_config.motor_k_p = settings.value("MOTOR/k_p").toDouble();
        robot_config.motor_k_i = settings.value("MOTOR/k_i").toDouble();
        robot_config.motor_k_d = settings.value("MOTOR/k_d").toDouble();
        robot_config.motor_limit_v = settings.value("MOTOR/limit_v").toDouble();
        robot_config.motor_limit_w = settings.value("MOTOR/limit_w").toDouble()*D2R;
        robot_config.motor_limit_v_acc = settings.value("MOTOR/limit_v_acc").toDouble();
        robot_config.motor_limit_w_acc = settings.value("MOTOR/limit_w_acc").toDouble()*D2R;

        robot_config.robot_lidar_baudrate = settings.value("SENSOR/baudrate").toInt();
        robot_config.robot_lidar_offset_x = settings.value("SENSOR/offset_x").toDouble();
        robot_config.robot_lidar_offset_y = settings.value("SENSOR/offset_y").toDouble();
        robot_config.robot_lidar_max_range = settings.value("SENSOR/max_range").toDouble();
        robot_config.robot_lidar_mask = settings.value("SENSOR/mask").toDouble();
        robot_config.cam_exposure = settings.value("SENSOR/cam_exposure").toDouble();
        robot_config.cam_left_sn = settings.value("SENSOR/left_camera").toString();
        robot_config.cam_left_tf = settings.value("SENSOR/left_camera_tf").toString();
        robot_config.cam_right_sn = settings.value("SENSOR/right_camera").toString();
        robot_config.cam_right_tf = settings.value("SENSOR/right_camera_tf").toString();

        logger.write("robot config loaded", true);
    }
    else
    {
        logger.write("robot config not found", true);
    }

    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    return a.exec();
}
