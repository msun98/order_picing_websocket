#ifndef MAINWINDOW_H
#define MAINWINDOW_H

// defines
#include "global_defines.h"

// Qt
#include <QMainWindow>
#include <QTimer>
#include <QFileDialog>

// my
#include "ipc.h"
#include "unimap.h"
#include "topomap.h"
#include "cam.h"
#include "lidar_2d.h"
#include "mobile.h"
#include "slam_2d.h"
#include "autocontrol.h"
#include "sim.h"
#include "ipc.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    // ipc
    IPC ipc;

    // camera
    CAM cam;

    // lidar
    LIDAR_2D lidar;

    // platforms
    MOBILE mobile;

    // alogorithms
    SLAM_2D slam;    

    // control
    AUTOCONTROL ctrl;

    // unified map
    UNIMAP unimap;

    // topology map
    TOPOMAP topomap;

    // simulation
    SIM sim;

    // lcm
    lcm::LCM lcm;
    std::atomic<bool> lcmFlag;
    std::thread* lcmThread = NULL;
    void lcmLoop();

    // simulation
    IPC lpc;

//    MOBILE *mobile = NULL;

    // background loop
    std::atomic<bool> backFlag;
    std::thread* backThread = NULL;
    void backLoop();

    // background loop
    std::atomic<bool> backFlag2;
    std::thread* backThread2 = NULL;
    void backLoop2();

    // lcm callback
    void command_callback(const lcm::ReceiveBuffer *rbuf, const std::string &chan, const command *msg);

    // send ui
    void publish_status();
    void publish_path();
    void publish_map();
    void publish_map_obs();
    void publish_cam();
    void publish_shared_path();

    // for ipc
    void publish_ipc_status();
    void publish_ipc_path();
    void publish_ipc_map();
    void publish_ipc_map_obs();
    void publish_ipc_cam();

    std::atomic<bool> ipcFlag;
    std::thread* ipcThread = NULL;
    void ipcLoop();

    void process_command(int cmd, uint8_t* params);

    // for websocket using ipc
    std::atomic<bool> webSocketFlag;
    std::thread* WebSocketThread = NULL;
    void webSocketLoop();

    // temporal
    std::vector<PATH_POINT> pre_path;
    int pre_emo = -1;    
    bool test_flag = false;
    std::thread *auto_init_thread = NULL;
    std::atomic<int> last_accepted_cmd;
    std::atomic<int> ui_motor_state;
    std::atomic<int> ui_loc_state;
    std::atomic<int> ui_auto_state;
    std::atomic<int> ui_obs_state;
    std::atomic<bool> pub_cam_flag;
    MOBILE_STATUS pre_status;

    std::mutex mtx;    
    tbb::concurrent_queue<cv::Mat> plot_que;

    // initialize
    void init();
    void set_ui_items_from_unimap();

    void loc_init(cv::Vec3d pose);
    void loc_auto_init();

    // websocket ipc command
    QString websocket_cmd;

private:
    Ui::MainWindow *ui;

    QTimer plot_timer;
    QTimer watchdog_timer;

    // record test
    QFile sim_rec_file;
    QFile real_rec_file;
    QTimer rec_timer;
    double rec_st_time = 0;
    double rec_predict_time = 3.0;

    // temporal
    int loc_fail_cnt = 0;
    int loop_count = 0;
    int random_count = 1;
    int pre_random_idx = 0;
    QTimer test_timer;

    uint32_t last_command_tick = 0;
    uint32_t last_status_tick = 0;

private slots:
    // mapping
    void bt_MapRun();
    void bt_MapStop();    
    void bt_MapSave();

    // robot move
    void bt_RobotMotorInit();    
    void bt_RobotMoveForward_pressed();
    void bt_RobotMoveBackward_pressed();
    void bt_RobotMoveLeftTurn_pressed();
    void bt_RobotMoveRightTurn_pressed();
    void bt_RobotMove_released();

    // localization
    void bt_LocalizationLoad();
    void bt_LocalizationInit();
    void bt_LocalizationInitAuto();    
    void bt_LocalizationRun();
    void bt_LocalizationStop();
    void AutoInitFinished();

    // manual
    void bt_ManualServingRun();
    void bt_ManualPatrolRun();
    void bt_ManualRestingRun();
    void bt_ManualChargingRun();

    void spb_AutoGain(double val);
    void ckb_SetEnable(int state);
    void bt_ManualServingSet();
    void bt_ManualPatrolSet();
    void bt_ManualRestingSet();
    void bt_ManualChargingSet();

    void bt_ManualOriginRun();
    void bt_ManualClickedRun();
    void bt_ManualClickedRun2();
    void bt_ManualStop();
    void bt_ManualPause();
    void bt_ManualResume();

    // temporal
    void bt_Exit();
    void bt_Test();
    void bt_Test2();
    void bt_Test3();
    void bt_Test4();
    void bt_TestEmo0();
    void bt_TestEmo1();
    void bt_TestCha0();
    void bt_TestCha1();

    void bt_ObsStart();
    void bt_ObsStop();
    void bt_ObsSave();

    void bt_TestLed();    
    void test_loop();
    void pose_clicked(double x, double y, double th);

    // plot loop
    void plot_loop();
    void watchdog_loop();

    // record
    void record_loop();

};
#endif // MAINWINDOW_H
