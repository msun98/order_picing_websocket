#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , plot_timer(this)
    , watchdog_timer(this)
    , lcm("udpm://239.255.76.67:7667?ttl=1")
    , ipc(this)
    , cam(this)
    , lidar(this)
    , mobile(this)
    , slam(this)
    , ctrl(this)
    , unimap(this)
    , topomap(this)
    , sim(this)
{
    ui->setupUi(this);

    // mapping
    connect(ui->bt_MapRun, SIGNAL(clicked()), this, SLOT(bt_MapRun()));
    connect(ui->bt_MapStop, SIGNAL(clicked()), this, SLOT(bt_MapStop()));
    connect(ui->bt_MapSave, SIGNAL(clicked()), this, SLOT(bt_MapSave()));

    // robot move
    connect(ui->bt_RobotMotorInit, SIGNAL(clicked()), this, SLOT(bt_RobotMotorInit()));
    connect(ui->bt_RobotMoveForward, SIGNAL(pressed()), this, SLOT(bt_RobotMoveForward_pressed()));
    connect(ui->bt_RobotMoveBackward, SIGNAL(pressed()), this, SLOT(bt_RobotMoveBackward_pressed()));
    connect(ui->bt_RobotMoveLeftTurn, SIGNAL(pressed()), this, SLOT(bt_RobotMoveLeftTurn_pressed()));
    connect(ui->bt_RobotMoveRightTurn, SIGNAL(pressed()), this, SLOT(bt_RobotMoveRightTurn_pressed()));

    connect(ui->bt_RobotMoveForward, SIGNAL(released()), this, SLOT(bt_RobotMove_released()));
    connect(ui->bt_RobotMoveBackward, SIGNAL(released()), this, SLOT(bt_RobotMove_released()));
    connect(ui->bt_RobotMoveLeftTurn, SIGNAL(released()), this, SLOT(bt_RobotMove_released()));
    connect(ui->bt_RobotMoveRightTurn, SIGNAL(released()), this, SLOT(bt_RobotMove_released()));

    // localization
    connect(ui->bt_LocalizationLoad, SIGNAL(clicked()), this, SLOT(bt_LocalizationLoad()));
    connect(ui->bt_LocalizationInit, SIGNAL(clicked()), this, SLOT(bt_LocalizationInit()));
    connect(ui->bt_LocalizationInitAuto, SIGNAL(clicked()), this, SLOT(bt_LocalizationInitAuto()));
    connect(ui->bt_LocalizationRun, SIGNAL(clicked()), this, SLOT(bt_LocalizationRun()));
    connect(ui->bt_LocalizationStop, SIGNAL(clicked()), this, SLOT(bt_LocalizationStop()));
    connect(&slam, SIGNAL(auto_init_finished()), this, SLOT(AutoInitFinished()));

    // manual
    connect(ui->bt_ManualServingRun, SIGNAL(clicked()), this, SLOT(bt_ManualServingRun()));
    connect(ui->bt_ManualPatrolRun, SIGNAL(clicked()), this, SLOT(bt_ManualPatrolRun()));
    connect(ui->bt_ManualRestingRun, SIGNAL(clicked()), this, SLOT(bt_ManualRestingRun()));
    connect(ui->bt_ManualChargingRun, SIGNAL(clicked()), this, SLOT(bt_ManualChargingRun()));

    connect(ui->spb_AutoGain, SIGNAL(valueChanged(double)), this, SLOT(spb_AutoGain(double)));
    connect(ui->ckb_SetEnable, SIGNAL(stateChanged(int)), this, SLOT(ckb_SetEnable(int)));
    connect(ui->bt_ManualServingSet, SIGNAL(clicked()), this, SLOT(bt_ManualServingSet()));
    connect(ui->bt_ManualPatrolSet, SIGNAL(clicked()), this, SLOT(bt_ManualPatrolSet()));
    connect(ui->bt_ManualRestingSet, SIGNAL(clicked()), this, SLOT(bt_ManualRestingSet()));
    connect(ui->bt_ManualChargingSet, SIGNAL(clicked()), this, SLOT(bt_ManualChargingSet()));

    connect(ui->bt_ManualOriginRun, SIGNAL(clicked()), this, SLOT(bt_ManualOriginRun()));
    connect(ui->bt_ManualClickedRun, SIGNAL(clicked()), this, SLOT(bt_ManualClickedRun()));
    connect(ui->bt_ManualClickedRun2, SIGNAL(clicked()), this, SLOT(bt_ManualClickedRun2()));
    connect(ui->bt_ManualStop, SIGNAL(clicked()), this, SLOT(bt_ManualStop()));
    connect(ui->bt_ManualPause, SIGNAL(clicked()), this, SLOT(bt_ManualPause()));
    connect(ui->bt_ManualResume, SIGNAL(clicked()), this, SLOT(bt_ManualResume()));

    // temporal
    connect(ui->bt_Test, SIGNAL(clicked()), this, SLOT(bt_Test()));
    connect(ui->bt_Test2, SIGNAL(clicked()), this, SLOT(bt_Test2()));
    connect(ui->bt_Test3, SIGNAL(clicked()), this, SLOT(bt_Test3()));
    connect(ui->bt_Test4, SIGNAL(clicked()), this, SLOT(bt_Test4()));
    connect(ui->bt_TestEmo0, SIGNAL(clicked()), this, SLOT(bt_TestEmo0()));
    connect(ui->bt_TestEmo1, SIGNAL(clicked()), this, SLOT(bt_TestEmo1()));
    connect(ui->bt_TestCha0, SIGNAL(clicked()), this, SLOT(bt_TestCha0()));
    connect(ui->bt_TestCha1, SIGNAL(clicked()), this, SLOT(bt_TestCha1()));

    connect(ui->bt_ObsStart, SIGNAL(clicked()), this, SLOT(bt_ObsStart()));
    connect(ui->bt_ObsStop, SIGNAL(clicked()), this, SLOT(bt_ObsStop()));
    connect(ui->bt_ObsSave, SIGNAL(clicked()), this, SLOT(bt_ObsSave()));

    connect(ui->bt_TestLed, SIGNAL(clicked()), this, SLOT(bt_TestLed()));
    connect(&test_timer, SIGNAL(timeout()), this, SLOT(test_loop()));
    connect(ui->bt_Exit, SIGNAL(clicked()), this, SLOT(bt_Exit()));

#ifdef USE_SIM
    connect(ui->gv_Screen1, SIGNAL(pose_clicked(double, double, double)), this, SLOT(pose_clicked(double, double, double)));
#endif

    // plot
    connect(&plot_timer, SIGNAL(timeout()), this, SLOT(plot_loop()));
    connect(&watchdog_timer, SIGNAL(timeout()), this, SLOT(watchdog_loop()));

    // record
    connect(&rec_timer, SIGNAL(timeout()), this, SLOT(record_loop()));

    // initialize
    init();

    logger.write("[MAIN] Initilized", true);
}

MainWindow::~MainWindow()
{
    if(lcmThread != NULL)
    {
        lcmFlag = false;
        lcmThread->join();
        lcmThread = NULL;
    }

    if(backThread != NULL)
    {
        backFlag = false;
        backThread->join();
        backThread = NULL;
    }

    if(ipcThread != NULL)
    {
        ipcFlag = false;
        ipcThread->join();
        ipcThread = NULL;
    }

    if(WebSocketThread != NULL)
    {
        webSocketFlag = false;
        WebSocketThread->join();
        WebSocketThread = NULL;
    }

    delete ui;
}

void MainWindow::bt_Exit()
{
    logger.write("[MAIN] program dead", true);
    QApplication::exit();
}

void MainWindow::init()
{
    // variable
    ui_motor_state = UI_MOTOR_NOT_READY;
    ui_loc_state = UI_LOC_NOT_READY;
    ui_auto_state = UI_AUTO_NOT_READY;
    ui_obs_state = UI_OBS_NONE;
    pub_cam_flag = false;

    // mobile init
    mobile.init();

    // camera init
    cam.init();

    // lidar init
    lidar.init(&mobile);

    // slam init
    slam.init(&cam, &lidar, &mobile, &unimap);

    // control init
    ctrl.init(&mobile, &slam, &unimap);

    // simulation init
#ifdef  USE_SIM
    sim.init(&mobile, &lidar, &cam);
    logger.write("[MAIN] sim mode", true);
#else
    // auto load map
    QString path = robot_config.map_path;
    if(!path.isEmpty())
    {
        QDir dir(path);
        if(dir.exists())
        {
            unimap.load_map(path);
            slam.set_map();
            ui->lb_MapName->setText(path);

            // set ui
            set_ui_items_from_unimap();
        }
    }

#endif

    // topomap
    //topomap.init(&unimap);

    // run watchdog
    watchdog_timer.start(100);

    // plot
    plot_timer.start(100);

    // background loop init
    if (backThread == NULL)
    {
        backFlag = true;
        backThread = new std::thread(&MainWindow::backLoop, this);
    }

    // lcm loop init
    if (lcmThread == NULL)
    {
        lcmFlag = true;
        //        printf("working lcm");
        lcmThread = new std::thread(&MainWindow::lcmLoop, this);
    }

    // ipc loop init
    if (ipcThread == NULL)
    {
        ipcFlag = true;
        ipcThread = new std::thread(&MainWindow::ipcLoop, this);
    }

    // ipc yujin loop init
    if (WebSocketThread == NULL)
    {
        webSocketFlag = true;
        WebSocketThread = new std::thread(&MainWindow::webSocketLoop, this);
    }
}

void MainWindow::set_ui_items_from_unimap()
{
    // set edited location to ui
    ui->cb_ManualServing->clear();
    ui->cb_ManualPatrol->clear();
    ui->cb_ManualCharging->clear();
    ui->cb_ManualResting->clear();

    for(size_t p = 0; p < unimap.serving_loc_name_list.size(); p++)
    {
        ui->cb_ManualServing->addItem(unimap.serving_loc_name_list[p]);
    }

    for(size_t p = 0; p < unimap.patrol_loc_name_list.size(); p++)
    {
        ui->cb_ManualPatrol->addItem(unimap.patrol_loc_name_list[p]);
    }

    for(size_t p = 0; p < unimap.charging_loc_name_list.size(); p++)
    {
        ui->cb_ManualCharging->addItem(unimap.charging_loc_name_list[p]);
    }

    for(size_t p = 0; p < unimap.resting_loc_name_list.size(); p++)
    {
        ui->cb_ManualResting->addItem(unimap.resting_loc_name_list[p]);
    }

    if(unimap.serving_loc_name_list.size() > 0)
    {
        ui->cb_ManualServing->setCurrentIndex(0);
    }

    if(unimap.patrol_loc_name_list.size() > 0)
    {
        ui->cb_ManualPatrol->setCurrentIndex(0);
    }

    if(unimap.charging_loc_name_list.size() > 0)
    {
        ui->cb_ManualCharging->setCurrentIndex(0);
    }

    if(unimap.resting_loc_name_list.size() > 0)
    {
        ui->cb_ManualResting->setCurrentIndex(0);
    }
}

// mapping
void MainWindow::bt_MapRun()
{
    slam.run();
}

void MainWindow::bt_MapStop()
{
    slam.stop();
}

void MainWindow::bt_MapSave()
{
    ui->ckb_PlotEnable->setChecked(false);

    QString path = QFileDialog::getExistingDirectory(this, "Select dir", QDir::homePath() + "/maps");
    if(!path.isNull())
    {
        unimap.save_map(path);
    }

    ui->ckb_PlotEnable->setChecked(true);
}

// robot move
void MainWindow::bt_RobotMotorInit()
{
    mobile.motor_init();
}

void MainWindow::bt_RobotMoveForward_pressed()
{
    double alpha = ui->spb_JogGain->value();
    double v = robot_config.robot_manual_limit_v * alpha;
    double w = 0;
    mobile.move(v, w);
    printf("[JOG] v:%f, w:%f(%f)\n", v, w*R2D, w);
}

void MainWindow::bt_RobotMoveBackward_pressed()
{    
    double alpha = ui->spb_JogGain->value();
    double v = -robot_config.robot_manual_limit_v * alpha;
    double w = 0;
    mobile.move(v, w);
    printf("[JOG] v:%f, w:%f(%f)\n", v, w*R2D, w);
}

void MainWindow::bt_RobotMoveLeftTurn_pressed()
{
    double alpha = ui->spb_JogGain->value();
    double v = 0;
    double w = robot_config.robot_manual_limit_w * alpha;
    mobile.move(v, w);
    printf("[JOG] v:%f, w:%f(%f)\n", v, w*R2D, w);
}

void MainWindow::bt_RobotMoveRightTurn_pressed()
{
    double alpha = ui->spb_JogGain->value();
    double v = 0;
    double w = -robot_config.robot_manual_limit_w * alpha;
    mobile.move(v, w);
    printf("[JOG] v:%f, w:%f(%f)\n", v, w*R2D, w);
}

void MainWindow::bt_RobotMove_released()
{
    mobile.move(0.0, 0.0);
}

// localization
void MainWindow::bt_LocalizationLoad()
{
    // dataset load
    QString path = QFileDialog::getExistingDirectory(this, "Select dir", QDir::homePath() + "/maps");
    if (!path.isNull())
    {
        // if thread already working
        slam.stop();
        slam.stop_loc();

#ifdef USE_SIM
        sim.load_map(path);
#endif

        // load map
        unimap.load_map(path);
        slam.set_map();

        // set annotated location to ui
        set_ui_items_from_unimap();
        ui->lb_MapName->setText(path);
    }
}

void MainWindow::bt_LocalizationInit()
{
    loc_init(ui->gv_Screen1->target_pose);
}

void MainWindow::loc_init(cv::Vec3d pose)
{
    logger.write("[LOC] try manual init", true);
    ui_loc_state = UI_LOC_BUSY;

    // initial location finding
    slam.set_initial_location(pose);
    slam.run_loc();
}

void MainWindow::bt_LocalizationInitAuto()
{
    loc_auto_init();
}

void MainWindow::loc_auto_init()
{
    logger.write("[LOC] try auto init", true);
    ui_loc_state = UI_LOC_BUSY;

    slam.set_cur_scans();

    // doing thread
    auto_init_thread = new std::thread(&SLAM_2D::set_auto_initial_location, &slam);
}

void MainWindow::AutoInitFinished()
{
    slam.run_loc();
}

void MainWindow::bt_LocalizationRun()
{
    slam.run_loc();

#ifdef USE_SIM
    slam.loc_inlier_error = 0;
    slam.loc_inlier_ratio = 1.0;
    ui_loc_state = UI_LOC_GOOD;
#endif
}

void MainWindow::bt_LocalizationStop()
{
    slam.stop_loc();
}

// manual
void MainWindow::bt_ManualServingRun()
{
    if(unimap.serving_loc_list.size() == 0)
    {
        return;
    }

    int idx = ui->cb_ManualServing->currentIndex();

    // robot state now moving
    ctrl.ofsm_run(unimap.serving_loc_list[idx]);
    printf("serving_%d, %f, %f, %f\n", idx, unimap.serving_loc_list[idx][0], unimap.serving_loc_list[idx][1], unimap.serving_loc_list[idx][2]*R2D);
}

void MainWindow::bt_ManualPatrolRun()
{
    if(unimap.patrol_loc_list.size() == 0)
    {
        return;
    }

    int idx = ui->cb_ManualPatrol->currentIndex();

    // robot state now moving
    ctrl.ofsm_run(unimap.patrol_loc_list[idx]);
    printf("patrol_%d, %f, %f, %f\n", idx, unimap.patrol_loc_list[idx][0], unimap.patrol_loc_list[idx][1], unimap.patrol_loc_list[idx][2]*R2D);
}

void MainWindow::bt_ManualRestingRun()
{
    if(unimap.resting_loc_list.size() == 0)
    {
        return;
    }

    int idx = ui->cb_ManualResting->currentIndex();

    // robot state now moving
    ctrl.ofsm_run(unimap.resting_loc_list[idx]);
    printf("resting_%d, %f, %f, %f\n", idx, unimap.resting_loc_list[idx][0], unimap.resting_loc_list[idx][1], unimap.resting_loc_list[idx][2]*R2D);
}

void MainWindow::bt_ManualChargingRun()
{
    if(unimap.charging_loc_list.size() == 0)
    {
        return;
    }

    int idx = ui->cb_ManualCharging->currentIndex();

    // robot state now moving
    ctrl.ofsm_run(unimap.charging_loc_list[idx]);
    printf("charging_%d, %f, %f, %f\n", idx, unimap.charging_loc_list[idx][0], unimap.charging_loc_list[idx][1], unimap.charging_loc_list[idx][2]*R2D);
}

void MainWindow::ckb_SetEnable(int state)
{
    if(state == Qt::Unchecked)
    {
        ui->bt_ManualServingSet->setEnabled(false);
        ui->bt_ManualPatrolSet->setEnabled(false);
        ui->bt_ManualRestingSet->setEnabled(false);
        ui->bt_ManualChargingSet->setEnabled(false);
    }
    else
    {
        ui->bt_ManualServingSet->setEnabled(true);
        ui->bt_ManualPatrolSet->setEnabled(true);
        ui->bt_ManualRestingSet->setEnabled(true);
        ui->bt_ManualChargingSet->setEnabled(true);
    }
}

void MainWindow::spb_AutoGain(double val)
{
    ctrl.whole_gain = val;
}

void MainWindow::bt_ManualServingSet()
{
    if(unimap.serving_loc_list.size() == 0)
    {
        return;
    }

    int idx = ui->cb_ManualServing->currentIndex();

    cv::Vec3d pose = slam.get_cur_pose();
    unimap.edit_location(LOCATION_SERVING, idx, pose);
}

void MainWindow::bt_ManualPatrolSet()
{
    if(unimap.patrol_loc_list.size() == 0)
    {
        return;
    }

    int idx = ui->cb_ManualPatrol->currentIndex();

    cv::Vec3d pose = slam.get_cur_pose();
    unimap.edit_location(LOCATION_PATROL, idx, pose);
}

void MainWindow::bt_ManualRestingSet()
{
    if(unimap.resting_loc_list.size() == 0)
    {
        return;
    }

    int idx = ui->cb_ManualResting->currentIndex();

    cv::Vec3d pose = slam.get_cur_pose();
    unimap.edit_location(LOCATION_RESTING, idx, pose);
}

void MainWindow::bt_ManualChargingSet()
{
    if(unimap.charging_loc_list.size() == 0)
    {
        return;
    }

    int idx = ui->cb_ManualCharging->currentIndex();

    cv::Vec3d pose = slam.get_cur_pose();
    unimap.edit_location(LOCATION_CHARGING, idx, pose);
}

void MainWindow::bt_ManualOriginRun()
{
    // robot state now moving
    ctrl.ofsm_run(cv::Vec3d(0,0,0));
}

void MainWindow::bt_ManualClickedRun()
{
    // robot state now moving
    ctrl.ofsm_run(ui->gv_Screen1->target_pose);
}

void MainWindow::bt_ManualClickedRun2()
{
    ctrl.add_waypoint(ui->gv_Screen1->target_pose);
}

void MainWindow::bt_ManualStop()
{
    ctrl.ifsm_stop();
    if(ctrl.ofsm_state == STATE_WAITING)
    {
        ctrl.ofsm_state = STATE_IDLE;
    }
}

void MainWindow::bt_ManualPause()
{
    ctrl.is_pause = true;
}

void MainWindow::bt_ManualResume()
{
    ctrl.is_pause = false;
}

// temporal
void MainWindow::bt_Test()
{
}

void MainWindow::bt_Test2()
{
    if(test_timer.isActive())
    {
        test_timer.stop();
        ui->bt_Test2->setText("Random");
    }
    else
    {
        test_timer.setInterval(1000);
        test_timer.start();
        ui->bt_Test2->setText("...");
    }
}

void MainWindow::bt_Test3()
{
    topomap.build();
}

void MainWindow::bt_Test4()
{

}

// obs map
void MainWindow::bt_ObsStart()
{
    unimap.mtx.lock();
    unimap.obs_map0.setTo(0);
    unimap.mtx.unlock();

    slam.is_obs = true;
}

void MainWindow::bt_ObsStop()
{
    slam.is_obs = false;
}

void MainWindow::bt_ObsSave()
{
    cv::Mat _obs_map0 = unimap.get_map_obs0();
    QString obs_map_path = robot_config.map_path + "/map_obs.png";
    cv::imwrite(obs_map_path.toStdString(), _obs_map0);
}


void MainWindow::bt_TestEmo0()
{
#ifdef USE_SIM
    MOBILE_STATUS status;
    status.is_ok = true;
    status.emo_state = 0;
    status.status_m0 = 0;
    status.status_m1 = 0;
    mobile.status_received(status);
#endif
}

void MainWindow::bt_TestEmo1()
{
#ifdef USE_SIM
    MOBILE_STATUS status;
    status.is_ok = true;
    status.emo_state = 1;
    status.status_m0 = 1;
    status.status_m1 = 1;
    mobile.status_received(status);
#endif
}

void MainWindow::bt_TestCha0()
{
#ifdef USE_SIM
    MOBILE_STATUS status;
    status.is_ok = true;
    status.emo_state = 1;
    status.status_m0 = 1;
    status.status_m1 = 1;
    status.charge_state = 0;
    mobile.status_received(status);
#endif
}

void MainWindow::bt_TestCha1()
{
#ifdef USE_SIM
    MOBILE_STATUS status;
    status.is_ok = true;
    status.emo_state = 1;
    status.status_m0 = 1;
    status.status_m1 = 1;
    status.charge_state = 1;
    mobile.status_received(status);
#endif
}

void MainWindow::pose_clicked(double x, double y, double th)
{

    /*
#ifdef USE_SIM
    shared_path msg;
    msg.id = 10;
    msg.po = 0;
    msg.v = 0;
    msg.pose[0] = x;
    msg.pose[1] = y;
    msg.pose[2] = th;
    msg.num = 1;
    msg.path.resize(1);
    msg.path[0].resize(2);
    msg.path[0][0] = x;
    msg.path[0][1] = y;

    lcm.publish("SHARED_PATH", &msg);
#endif
*/

    /*
#ifdef USE_SIM
    // make dynamic obstacle tree
    unimap.dynamic_obs_pts.clear();
    unimap.dynamic_obs_pts.push_back(cv::Vec2d(x,y));
    unimap.dynamic_obs_tree_src = cv::Mat(unimap.dynamic_obs_pts).reshape(1);
    unimap.dynamic_obs_tree_src.convertTo(unimap.dynamic_obs_tree_src, CV_32F);
    if(unimap.dynamic_obs_tree_idx_params != NULL)
    {
        delete unimap.dynamic_obs_tree_idx_params;
        unimap.dynamic_obs_tree_idx_params = NULL;
    }
    unimap.dynamic_obs_tree_idx_params = new cv::flann::KDTreeIndexParams(2);

    if(unimap.dynamic_obs_tree != NULL)
    {
        delete unimap.dynamic_obs_tree;
        unimap.dynamic_obs_tree = NULL;
    }
    unimap.dynamic_obs_tree = new cv::flann::Index(unimap.dynamic_obs_tree_src, *unimap.dynamic_obs_tree_idx_params);

#endif
*/

}

void MainWindow::bt_TestLed()
{
    int target = ui->spb_LedTarget->value();
    int mode = ui->spb_LedMode->value();
    mobile.led(target, mode);
}

void MainWindow::record_loop()
{
    if(real_rec_file.isOpen())
    {
        MOBILE_POSE mobile_pose = mobile.get_pose();

        double t = get_time() - rec_st_time;
        double x = mobile_pose.pose[0];
        double y = mobile_pose.pose[1];
        double th = mobile_pose.pose[2];
        double _v = std::sqrt(mobile_pose.vel[0]*mobile_pose.vel[0] + mobile_pose.vel[1]*mobile_pose.vel[1]);
        double _w = mobile_pose.vel[2]*R2D;

        QString str;
        str.sprintf("%f,%f,%f,%f,%f,%f\n", t, x, y, th*R2D, _v, _w*R2D);
        real_rec_file.write(str.toLocal8Bit(), str.length());

        if(t > rec_predict_time)
        {
            real_rec_file.close();
            rec_timer.stop();
            mobile.move(0, 0);

            printf("rec timer stopped\n");
            return;
        }
    }
}

void MainWindow::test_loop()
{
    if(unimap.serving_loc_list.size() == 0)
    {
        printf("no annotaton locations\n");
        return;
    }

    if(ui_auto_state == UI_AUTO_READY)
    {
        if(pre_random_idx == 0)
        {
            ctrl.ofsm_run(unimap.serving_loc_list[random_count]);
            printf("Run serving %d\n", random_count);

            random_count++;
            if(random_count >= (int)unimap.serving_loc_list.size())
            {
                random_count = 1;
            }

            pre_random_idx = 1;
        }
        else
        {
            ctrl.ofsm_run(unimap.serving_loc_list[0]);
            printf("Run resting\n", 0);

            pre_random_idx = 0;
        }
    }
}

// lcm
void MainWindow::lcmLoop()
{
    /*
    sudo ifconfig lo multicast
    sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev lo
    sudo sysctl -w net.core.rmem_default=2097152
    */

    // lcm init
    if(lcm.good())
    {
        lcm.subscribe(QString("COMMAND_" + robot_config.robot_model + robot_config.robot_sn).toStdString(), &MainWindow::command_callback, this);
        lcm.subscribe("SHARED_PATH", &AUTOCONTROL::shared_path_callback, &ctrl);
    }
    else
    {
        logger.write("[LCM] lcm init failed", true);
    }

    while(lcmFlag)
    {
        int res = lcm.handleTimeout(100);
        if(res < 0)
        {
            logger.write("[LCM] lcm some error", true);
        }
    }
}

// using lcm
void MainWindow::publish_status()
{
    cv::Vec3d cur_pose = slam.get_cur_pose();
    std::vector<cv::Vec2d> cur_scan = slam.get_cur_scan();
    if(cur_scan.size() == 0)
    {
        slam.set_cur_scans();
        cur_scan = slam.get_cur_scan();
    }

    std::vector<float> cur_scan_sampled(360, 0);
    if(cur_scan.size() > 0)
    {
        for(size_t p = 0; p < cur_scan.size(); p++)
        {
            double x = cur_scan[p][0];
            double y = cur_scan[p][1];
            double d = std::sqrt(x*x + y*y);
            double th = std::atan2(y, x);
            double deg = th*R2D;
            if(deg < 0)
            {
                deg += 360;
            }

            int idx = std::round(deg);
            if(cur_scan_sampled[idx] == 0)
            {
                cur_scan_sampled[idx] = d;
            }
            else
            {
                if(d < cur_scan_sampled[idx])
                {
                    cur_scan_sampled[idx] = d;
                }
            }
        }
    }

    MOBILE_STATUS status = mobile.get_status();

    robot_status msg;
    msg.connection_m0 = status.connection_m0;
    msg.connection_m1 = status.connection_m1;

    msg.status_m0 = status.status_m0;
    msg.status_m1 = status.status_m1;

    msg.temp_m0 = status.temp_m0;
    msg.temp_m1 = status.temp_m1;

    msg.cur_m0 = status.cur_m0;
    msg.cur_m1 = status.cur_m1;

    msg.status_charge = status.charge_state;
    msg.status_power = status.power_state;
    msg.status_emo = status.emo_state;
    msg.status_remote = status.remote_state;

    msg.bat_in = status.bat_in;
    msg.bat_out = status.bat_out;
    msg.bat_cur = status.bat_current;

    msg.power = status.power;
    msg.total_power = status.total_power;

    msg.ui_motor_state = ui_motor_state;
    msg.ui_loc_state = ui_loc_state;
    msg.ui_auto_state = ui_auto_state;
    msg.ui_obs_state = ui_obs_state;

    msg.robot_pose[0] = cur_pose[0];
    msg.robot_pose[1] = cur_pose[1];
    msg.robot_pose[2] = cur_pose[2];
    memcpy(msg.robot_scan, cur_scan_sampled.data(), sizeof(float)*360);

    QString name = "STATUS_DATA_" + robot_config.robot_model + robot_config.robot_sn;
    lcm.publish(name.toStdString(), &msg);
}

void MainWindow::publish_map()
{
    printf("pub map\n");
    cv::Mat _map = unimap.get_map_raw();

    cv::Mat map;
    cv::resize(_map, map, cv::Size(1000,1000), 0, 0, cv::INTER_NEAREST);

    map_data msg;
    msg.map_grid_w = robot_config.robot_grid_size;
    msg.map_w = map.cols;
    msg.map_h = map.rows;
    msg.map_origin[0] = map.cols/2;
    msg.map_origin[1] = map.rows/2;

    msg.len = msg.map_w*msg.map_h;
    msg.data.resize(msg.len, 0);
    memcpy(msg.data.data(), map.data, map.rows*map.cols);

    QString name = "MAP_DATA_" + robot_config.robot_model + robot_config.robot_sn;
    //    lcm.publish(name.toStdString(), &msg);
}

void MainWindow::publish_map_obs()
{
    printf("pub obs\n");
    cv::Mat _raw = unimap.get_map_raw();
    cv::Mat _obs = unimap.get_map_obs0();

    cv::Mat raw;
    cv::resize(_raw, raw, cv::Size(1000,1000), 0, 0, cv::INTER_NEAREST);

    cv::Mat obs;
    cv::resize(_obs, obs, cv::Size(1000,1000), 0, 0, cv::INTER_NEAREST);

    cv::Mat map;
    cv::addWeighted(raw, 0.5, obs, 0.5, 0, map);

    map_data msg;
    msg.map_grid_w = robot_config.robot_grid_size;
    msg.map_w = map.cols;
    msg.map_h = map.rows;
    msg.map_origin[0] = map.cols/2;
    msg.map_origin[1] = map.rows/2;

    msg.len = msg.map_w*msg.map_h;
    msg.data.resize(msg.len, 0);
    memcpy(msg.data.data(), map.data, map.rows*map.cols);

    QString name = "OBS_DATA_" + robot_config.robot_model + robot_config.robot_sn;
    //    lcm.publish(name.toStdString(), &msg);
}

void MainWindow::publish_path()
{
    std::vector<PATH_POINT> path = ctrl.get_cur_path();
    if(path.size() == 0)
    {
        return;
    }

    if(path.size() == pre_path.size() && path[0].pt == pre_path[0].pt && path.back().pt == pre_path.back().pt)
    {
        return;
    }
    pre_path = path;

    // path sampling, 0.5m
    std::vector<PATH_POINT> sampled_path;
    sampled_path.push_back(path[0]);
    for(size_t p = 1; p < path.size()-1; p+=50)
    {
        sampled_path.push_back(path[p]);
    }
    sampled_path.push_back(path.back());

    robot_path msg;
    msg.num = sampled_path.size();
    msg.path.resize(msg.num);
    for(int p = 0; p < msg.num; p++)
    {
        msg.path[p].resize(2);
        msg.path[p][0] = sampled_path[p].pt[0];
        msg.path[p][1] = sampled_path[p].pt[1];
    }

    QString name = "ROBOT_PATH_" + robot_config.robot_model + robot_config.robot_sn;
    lcm.publish(name.toStdString(), &msg);
}

void MainWindow::publish_shared_path()
{
    int id = robot_config.robot_id;
    cv::Vec3d cur_pose = slam.get_cur_pose();
    std::vector<PATH_POINT> path = ctrl.get_cur_path();

    // path sampling, 0.1m
    std::vector<PATH_POINT> sampled_path;

    PATH_POINT cur;
    cur.pt[0] = cur_pose[0];
    cur.pt[1] = cur_pose[1];
    sampled_path.push_back(cur);

    if(path.size() > 0)
    {
        int cur_idx = ctrl.find_nn_path_idx(cv::Vec2d(cur_pose[0], cur_pose[1]), path);
        for(size_t p = cur_idx + 1; p < path.size(); p+=10)
        {
            sampled_path.push_back(path[p]);
        }
    }

    shared_path msg;
    msg.id = id;
    msg.num = sampled_path.size();
    msg.path.resize(msg.num);
    for(int p = 0; p < msg.num; p++)
    {
        msg.path[p].resize(2);
        msg.path[p][0] = sampled_path[p].pt[0]; // x
        msg.path[p][1] = sampled_path[p].pt[1]; // y
    }

    lcm.publish("SHARED_PATH", &msg);
}

void MainWindow::publish_cam()
{
    auto cam_l = cam.get_cur_img_l();
    auto cam_r = cam.get_cur_img_r();

    camera_data msg;
    msg.width = cam_l.cols;
    msg.height = cam_l.rows;

    msg.num = 2;
    msg.image_len = cam_l.cols*cam_l.rows;
    msg.image.resize(2);

    msg.image[0].resize(cam_l.cols*cam_l.rows, 0);
    msg.image[1].resize(cam_r.cols*cam_r.rows, 0);
    memcpy(msg.image[0].data(), cam_l.data, cam_l.rows*cam_l.cols);
    memcpy(msg.image[1].data(), cam_r.data, cam_r.rows*cam_r.cols);

    msg.serial.resize(2, std::string(""));
    msg.serial[0] = cam.get_sn_l().toStdString();
    msg.serial[1] = cam.get_sn_r().toStdString();

    QString name = "CAMERA_DATA_" + robot_config.robot_model + robot_config.robot_sn;
    lcm.publish(name.toStdString(), &msg);
}

void MainWindow::command_callback(const lcm::ReceiveBuffer *rbuf, const std::string &chan, const command *msg)
{
    process_command(msg->cmd, (uint8_t*)msg->params);
}

void MainWindow::process_command(int cmd, uint8_t* params)
{
    if(robot_config.robot_cmd_accept == false)
    {
        printf("cmd:%d, ignored\n",cmd);
        return;
    }
    else
    {
        printf("cmd:%d, accepted\n",cmd);
    }

    if(cmd == CMD_MOVE_LOCATION)
    {
        if(unimap.is_loaded == false)
        {
            printf("[CMD] map not loaded\n");
            return;
        }

        QByteArray buf = (char*)params;

        QString str(buf);
        std::cout << str.toStdString() << std::endl;

        // parsing and doing
        QStringList str_list = str.split("_");
        QString type = str_list[0];
        int idx = str_list[1].toInt();

        if(type == "Serving")
        {
            // robot state now moving
            if(idx >= (int)unimap.serving_loc_list.size())
            {
                printf("[CMD] has no location\n");
                return;
            }

            ctrl.ofsm_run(unimap.serving_loc_list[idx]);

            QString str;
            str.sprintf("[CMD] Serving_%d, %f, %f, %f", idx, unimap.serving_loc_list[idx][0], unimap.serving_loc_list[idx][1], unimap.serving_loc_list[idx][2]*R2D);
            logger.write(str, true);
        }
        else if(type == "Patrol")
        {
            // robot state now moving
            if(idx >= (int)unimap.patrol_loc_list.size())
            {
                printf("[CMD] has no location\n");
                return;
            }

            ctrl.ofsm_run(unimap.patrol_loc_list[idx]);

            QString str;
            str.sprintf("[CMD] patrol_%d, %f, %f, %f\n", idx, unimap.patrol_loc_list[idx][0], unimap.patrol_loc_list[idx][1], unimap.patrol_loc_list[idx][2]*R2D);
            logger.write(str, true);
        }
        else if(type == "Charging")
        {
            // robot state now moving
            if(idx >= (int)unimap.charging_loc_list.size())
            {
                printf("[CMD] has no location\n");
                return;
            }

            ctrl.ofsm_run(unimap.charging_loc_list[idx]);

            QString str;
            str.sprintf("[CMD] charging_%d, %f, %f, %f\n", idx, unimap.charging_loc_list[idx][0], unimap.charging_loc_list[idx][1], unimap.charging_loc_list[idx][2]*R2D);
            logger.write(str, true);
        }
        else if(type == "Resting")
        {
            // robot state now moving
            if(idx >= (int)unimap.resting_loc_list.size())
            {
                printf("[CMD] has no location\n");
                return;
            }

            ctrl.ofsm_run(unimap.resting_loc_list[idx]);

            QString str;
            str.sprintf("[CMD] resting_%d, %f, %f, %f\n", idx, unimap.resting_loc_list[idx][0], unimap.resting_loc_list[idx][1], unimap.resting_loc_list[idx][2]*R2D);
            logger.write(str, true);
        }

        last_accepted_cmd = cmd;
    }
    else if(cmd == CMD_MOVE_TARGET)//
    {
        if(unimap.is_loaded == false)
        {
            return;
        }

        float x; memcpy(&x, &params[0], 4);
        float y; memcpy(&y, &params[4], 4);
        float th; memcpy(&th, &params[8], 4);

        ctrl.ofsm_run(cv::Vec3d(x, y, th));
        // 이 함수를 통해 move 명령 파씽하여 원격 테스트 가능.

        QString str;

        str.sprintf("[CMD] move target, %f, %f, %f\n", x, y, th*R2D);
        logger.write(str, true);

        last_accepted_cmd = cmd;
    }
    else if(cmd == CMD_MOVE_JOYSTICK)
    {
        float v_ratio; memcpy(&v_ratio, &params[0], 4);
        float w_ratio; memcpy(&w_ratio, &params[4], 4);
        mobile.move(v_ratio*robot_config.robot_manual_limit_v, w_ratio*robot_config.robot_manual_limit_w);
        printf("[CMD] JOG, v:%f,w:%f\n", v_ratio, w_ratio);

        last_accepted_cmd = cmd;
    }
    else if(cmd == CMD_MOVE_STOP)
    {

        bt_ManualStop();

        logger.write("[CMD] stop", true);
        last_accepted_cmd = cmd;
    }
    else if(cmd == CMD_PAUSE)
    {
        ctrl.is_pause = true;

        logger.write("[CMD] pause", true);
        last_accepted_cmd = cmd;
    }
    else if(cmd == CMD_RESUME)
    {
        ctrl.is_pause = false;

        logger.write("[CMD] resume", true);
        last_accepted_cmd = cmd;
    }
    else if(cmd == CMD_SET_VEL)
    {
        float gain; memcpy(&gain, &params[0], 4);
        ctrl.whole_gain = gain;

        logger.write("[CMD] set vel", true);
        last_accepted_cmd = cmd;
    }
    else if(cmd == CMD_RESTART)
    {
        logger.write("[CMD] restart", true);
        bt_Exit();
    }
    else if(cmd == CMD_SET_INIT)
    {
        float x; memcpy(&x, &params[0], 4);
        float y; memcpy(&y, &params[4], 4);
        float th; memcpy(&th, &params[8], 4);

        loc_init(cv::Vec3d(x,y,th));

        logger.write("[CMD] set init", true);
        last_accepted_cmd = cmd;
    }
    else if(cmd == CMD_LOC_RUN)
    {
        slam.run_loc();

        logger.write("[CMD] loc run", true);
        last_accepted_cmd = cmd;
    }
    else if(cmd == CMD_LOC_STOP)
    {
        slam.stop_loc();

        logger.write("[CMD] loc stop", true);
        last_accepted_cmd = cmd;
    }
    else if(cmd == CMD_LOC_AUTO)
    {
        loc_auto_init();

        logger.write("[CMD] auto init", true);
        last_accepted_cmd = cmd;
    }
    else if(cmd == CMD_MAPPING_START)
    {
        slam.run();

        logger.write("[CMD] start mapping", true);
        last_accepted_cmd = cmd;
    }
    else if(cmd == CMD_MAPPING_STOP)
    {
        slam.stop();

        logger.write("[CMD] stop mapping", true);
        last_accepted_cmd = cmd;
    }
    else if(cmd == CMD_REQ_CAMERA)
    {
        pub_cam_flag = true;

        logger.write("[CMD] req cam", true);
        last_accepted_cmd = cmd;
    }
    else if(cmd == CMD_MAP_SAVE)
    {
        QByteArray buf = (char*)params;
        QString str(buf);
        QString path = QDir::homePath() + "/maps/" + str;

        QDir dir;
        dir.mkpath(path);

        unimap.save_map(path);

        logger.write("[CMD] map save", true);
        last_accepted_cmd = cmd;
    }
    else if(cmd == CMD_OBS_START)
    {
        bt_ObsStart();

        logger.write("[CMD] obs start", true);
        last_accepted_cmd = cmd;
    }
    else if(cmd == CMD_OBS_STOP)
    {
        bt_ObsStop();

        logger.write("[CMD] obs stop", true);
        last_accepted_cmd = cmd;
    }
    else if(cmd == CMD_OBS_SAVE)
    {
        bt_ObsSave();

        logger.write("[CMD] obs save", true);
        last_accepted_cmd = cmd;
    }
}

// using shared memory
void MainWindow::ipcLoop()
{
//    qDebug()<<"ipc working";
    IPC::CMD pre_cmd;
    while(ipcFlag)
    {
        IPC::CMD cur_cmd = ipc.get_cmd();
        if(cur_cmd.tick != pre_cmd.tick)
        {
            // new command received
            process_command(cur_cmd.cmd, (uint8_t*)cur_cmd.params);
        }
        pre_cmd = cur_cmd;

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

// using shared memory for websocket
void MainWindow::webSocketLoop()
{
//    IPC::CMD pre_cmd;
    //    IPC::POSE pre_pose;
    while(webSocketFlag)
    {
//                qDebug()<<"ipc working";
//        IPC::CMD cur_cmd = ipc.get_cmd();
        IPC::STATUS status = ipc.get_status();
        IPC::POSE websocket_get_pose = ipc.get_move_where();
        IPC::SUCCESS_CHECK success_check;
        IPC::ROBOT_COMMAND robot_command = ipc.get_mobile_status();
        //        IPC::POSE get_pose = ipc.get_move_where();//어디로 가야하는지 유진로봇으로부터 전달받은 데이터.

//        if((cur_cmd.tick != pre_cmd.tick) && pre_cmd.tick != 0)
//        {
//            // new command received
//            process_command(cur_cmd.cmd, (uint8_t*)cur_cmd.params);
//        }

        bool success_flag = false;
        int old_ifsm_state;

        if(robot_command.tick != last_command_tick) //틱으로 로봇 상태 갱신되는지 확인.
        {
            if (robot_command.robot_status==0)
            {
                //                websocket_cmd = "CMD_MOVE_MOVING";
                //                qDebug()<<"cmd : "<<websocket_cmd;

                float x = websocket_get_pose.x;
                float y = websocket_get_pose.y;
                float th = websocket_get_pose.theta;

                ctrl.ofsm_run(cv::Vec3d(x,y,th));

                //                success_flag = true;
                old_ifsm_state = ctrl.ifsm_state ;
            }

            else if (robot_command.robot_status==1)
            {
                websocket_cmd = "CMD_PAUSE";
                qDebug()<<"cmd : "<<websocket_cmd;
                ctrl.ifsm_state = STATE_WAITING;
            }
            else if(robot_command.robot_status==2)
            {
                websocket_cmd = "CMD_RESUME";
                qDebug()<<"cmd : "<<websocket_cmd;
            }
            else if(robot_command.robot_status==3)
            {
                websocket_cmd = "CMD_MOVE_STOP";
                qDebug()<<"cmd : "<<websocket_cmd;
                mobile.move(0, 0);
                ctrl.ifsm_state = STATE_STOPPED;
                //                pre_u = cv::Vec2d(0, 0);
                //                ifsm_state = STATE_STOPPED;
            }
        }

        //        qDebug()<<"ctrl.ifsm_state"<<ctrl.ifsm_state;

        if (old_ifsm_state != ctrl.ifsm_state)
        {
            //유진에서 명령 내린 직후와는 다른 state여야 함.
            if(ctrl.ifsm_state == STATE_GOAL_REACHED) //틱으로 로봇 상태 갱신되는지 확인.
            {
                success_check.check = 0;
                ipc.set_mobile_success_check(success_check);
                std::cout<<"successed!!!!"<<std::endl;
                success_flag = false;
            }
            else if(ctrl.ifsm_state == STATE_PATH_FAILED)
            {
                success_check.check = 1;
                ipc.set_mobile_success_check(success_check);
                std::cout<<"fail!!!!"<<std::endl;
                success_flag = false;
            }
            old_ifsm_state = ctrl.ifsm_state;//이후 스테이트를 계속 저장하고 if 문 계속 돌리기
        }

        last_command_tick = robot_command.tick;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

void MainWindow::publish_ipc_status()
{
    cv::Vec3d cur_pose = slam.get_cur_pose();
    std::vector<cv::Vec2d> cur_scan = slam.get_cur_scan();
    if(cur_scan.size() == 0)
    {
        slam.set_cur_scans();
        cur_scan = slam.get_cur_scan();
    }

    std::vector<float> cur_scan_sampled(360, 0);
    if(cur_scan.size() > 0)
    {
        for(size_t p = 0; p < cur_scan.size(); p++)
        {
            double x = cur_scan[p][0];
            double y = cur_scan[p][1];
            double d = std::sqrt(x*x + y*y);
            double th = std::atan2(y, x);
            double deg = th*R2D;
            if(deg < 0)
            {
                deg += 360;
            }

            int idx = std::round(deg);
            if(cur_scan_sampled[idx] == 0)
            {
                cur_scan_sampled[idx] = d;
            }
            else
            {
                if(d < cur_scan_sampled[idx])
                {
                    cur_scan_sampled[idx] = d;
                }
            }
        }
    }

    MOBILE_STATUS status = mobile.get_status();

    IPC::STATUS msg;
    msg.connection_m0 = status.connection_m0;
    msg.connection_m1 = status.connection_m1;

    msg.status_m0 = status.status_m0;
    msg.status_m1 = status.status_m1;

    msg.temp_m0 = status.temp_m0;
    msg.temp_m1 = status.temp_m1;

    msg.cur_m0 = status.cur_m0;
    msg.cur_m1 = status.cur_m1;

    msg.status_charge = status.charge_state;
    msg.status_power = status.power_state;
    msg.status_emo = status.emo_state;
    msg.status_remote = status.remote_state;

    msg.bat_in = status.bat_in;
    msg.bat_out = status.bat_out;
    msg.bat_cur = status.bat_current;

    msg.power = status.power;
    msg.total_power = status.total_power;

    msg.ui_motor_state = ui_motor_state;
    msg.ui_loc_state = ui_loc_state;
    msg.ui_auto_state = ui_auto_state;
    msg.ui_obs_state = ui_obs_state;

    msg.robot_pose[0] = cur_pose[0];
    msg.robot_pose[1] = cur_pose[1];
    msg.robot_pose[2] = cur_pose[2];

    memcpy(msg.robot_scan, cur_scan_sampled.data(), sizeof(float)*360);

    ipc.set_status(msg);
}

void MainWindow::publish_ipc_path()
{
    std::vector<PATH_POINT> path = ctrl.get_cur_path();
    if(path.size() == 0)
    {
        return;
    }

    if(path.size() == pre_path.size() && path[0].pt == pre_path[0].pt && path.back().pt == pre_path.back().pt)
    {
        return;
    }
    pre_path = path;

    // path sampling, 0.5m
    std::vector<PATH_POINT> sampled_path;
    sampled_path.push_back(path[0]);
    for(size_t p = 1; p < path.size()-1; p+=50)
    {
        sampled_path.push_back(path[p]);
    }
    sampled_path.push_back(path.back());

    IPC::PATH msg;
    msg.num = sampled_path.size();
    for(int p = 0; p < msg.num; p++)
    {
        msg.x[p] = sampled_path[p].pt[0];
        msg.y[p] = sampled_path[p].pt[1];
    }

    ipc.set_path(msg);
}

void MainWindow::publish_ipc_map()
{
    cv::Mat _map = unimap.get_map_raw();

    cv::Mat map;
    cv::resize(_map, map, cv::Size(1000,1000), 0, 0, cv::INTER_NEAREST);

    IPC::MAP msg;
    memcpy(msg.buf, map.data, map.rows*map.cols);

    ipc.set_map(msg);
}

void MainWindow::publish_ipc_map_obs()
{
    cv::Mat _raw = unimap.get_map_raw();
    cv::Mat _obs = unimap.get_map_obs0();

    cv::Mat raw;
    cv::resize(_raw, raw, cv::Size(1000,1000), 0, 0, cv::INTER_NEAREST);

    cv::Mat obs;
    cv::resize(_obs, obs, cv::Size(1000,1000), 0, 0, cv::INTER_NEAREST);

    cv::Mat map;
    cv::addWeighted(raw, 0.5, obs, 0.5, 0, map);

    IPC::MAP msg;
    memcpy(msg.buf, map.data, map.rows*map.cols);

    ipc.set_obs(msg);
}

void MainWindow::publish_ipc_cam()
{
    cv::Mat cam0 = cam.get_cur_img_l();
    cv::Mat cam1 = cam.get_cur_img_r();

    QString sn0 = cam.get_sn_l();
    QString sn1 = cam.get_sn_r();

    IPC::IMG msg0;
    memcpy(msg0.serial, sn0.toUtf8().data(), 255);
    memcpy(msg0.buf, cam0.data, 480*270);
    ipc.set_cam0(msg0);

    IPC::IMG msg1;
    memcpy(msg1.serial, sn1.toUtf8().data(), 255);
    memcpy(msg1.buf, cam1.data, 480*270);
    ipc.set_cam1(msg1);
}

void MainWindow::plot_loop()
{
    // cmd info
    if(last_accepted_cmd == CMD_MOVE_LOCATION)
    {
        ui->lb_CmdInfo->setText("CMD_MOVE_LOCATION");
    }
    else if(last_accepted_cmd == CMD_MOVE_TARGET)
    {
        ui->lb_CmdInfo->setText("CMD_MOVE_TARGET");
    }
    else if(last_accepted_cmd == CMD_MOVE_JOYSTICK)
    {
        ui->lb_CmdInfo->setText("CMD_MOVE_JOYSTICK");
    }
    else if(last_accepted_cmd == CMD_MOVE_MANUAL)
    {
        ui->lb_CmdInfo->setText("CMD_MOVE_MANUAL");
    }
    else if(last_accepted_cmd == CMD_MOVE_STOP)
    {
        ui->lb_CmdInfo->setText("CMD_MOVE_STOP");
    }
    else if(last_accepted_cmd == CMD_PAUSE)
    {
        ui->lb_CmdInfo->setText("CMD_PAUSE");
    }
    else if(last_accepted_cmd == CMD_RESUME)
    {
        ui->lb_CmdInfo->setText("CMD_RESUME");
    }
    else if(last_accepted_cmd == CMD_SET_VEL)
    {
        ui->lb_CmdInfo->setText("CMD_SET_VEL");
    }
    else if(last_accepted_cmd == CMD_RESTART)
    {
        ui->lb_CmdInfo->setText("CMD_RESTART");
    }
    else if(last_accepted_cmd == CMD_SET_INIT)
    {
        ui->lb_CmdInfo->setText("CMD_SET_INIT");
    }
    else if(last_accepted_cmd == CMD_LOC_RUN)
    {
        ui->lb_CmdInfo->setText("CMD_LOC_RUN");
    }
    else if(last_accepted_cmd == CMD_LOC_STOP)
    {
        ui->lb_CmdInfo->setText("CMD_LOC_STOP");
    }
    else if(last_accepted_cmd == CMD_LOC_AUTO)
    {
        ui->lb_CmdInfo->setText("CMD_LOC_AUTO");
    }
    else if(last_accepted_cmd == CMD_MAPPING_START)
    {
        ui->lb_CmdInfo->setText("CMD_MAPPING_START");
    }
    else if(last_accepted_cmd == CMD_MAPPING_STOP)
    {
        ui->lb_CmdInfo->setText("CMD_MAPPING_STOP");
    }
    else if(last_accepted_cmd == CMD_REQ_CAMERA)
    {
        ui->lb_CmdInfo->setText("CMD_REQ_CAMERA");
    }
    else if(last_accepted_cmd == CMD_MAP_SAVE)
    {
        ui->lb_CmdInfo->setText("CMD_MAP_SAVE");
    }
    else if(last_accepted_cmd == CMD_OBS_START)
    {
        ui->lb_CmdInfo->setText("CMD_OBS_START");
    }
    else if(last_accepted_cmd == CMD_OBS_STOP)
    {
        ui->lb_CmdInfo->setText("CMD_OBS_STOP");
    }
    else if(last_accepted_cmd == CMD_OBS_SAVE)
    {
        ui->lb_CmdInfo->setText("CMD_OBS_SAVE");
    }

    // fsm state outer
    if(ctrl.ofsm_state == STATE_IDLE)
    {
        ui->lb_FsmStateOuter->setText("STATE_IDLE");
    }
    else if(ctrl.ofsm_state == STATE_DRIVING)
    {
        ui->lb_FsmStateOuter->setText("STATE_DRIVING");
    }
    else if(ctrl.ofsm_state == STATE_WAITING)
    {
        ui->lb_FsmStateOuter->setText("STATE_WAITING");
    }
    else if(ctrl.ofsm_state == STATE_FAILED)
    {
        ui->lb_FsmStateOuter->setText("STATE_FAILED");
    }

    // fsm state inner
    if(ctrl.ifsm_state == STATE_PATH_FINDING)
    {
        ui->lb_FsmStateInner->setText("STATE_PATH_FINDING");
    }
    else if(ctrl.ifsm_state == STATE_FIRST_ALIGN)
    {
        ui->lb_FsmStateInner->setText("STATE_FIRST_ALIGN");
    }
    else if(ctrl.ifsm_state == STATE_PURE_PURSUIT)
    {
        ui->lb_FsmStateInner->setText("STATE_PURE_PURSUIT");
    }
    else if(ctrl.ifsm_state == STATE_FINAL_ALIGN)
    {
        ui->lb_FsmStateInner->setText("STATE_FINAL_ALIGN");
    }
    else if(ctrl.ifsm_state == STATE_GOAL_REACHED)
    {
        ui->lb_FsmStateInner->setText("STATE_GOAL_REACHED");
    }
    else if(ctrl.ifsm_state == STATE_OBSTACLE)
    {
        ui->lb_FsmStateInner->setText("STATE_OBSTACLE");
    }
    else if(ctrl.ifsm_state == STATE_WAIT)
    {
        ui->lb_FsmStateInner->setText("STATE_WAIT");
    }
    else if(ctrl.ifsm_state == STATE_AVOID)
    {
        ui->lb_FsmStateInner->setText("STATE_AVOID");
    }
    else if(ctrl.ifsm_state == STATE_PATH_OUT)
    {
        ui->lb_FsmStateInner->setText("STATE_PATH_OUT");
    }
    else if(ctrl.ifsm_state == STATE_PATH_FAILED)
    {
        ui->lb_FsmStateInner->setText("STATE_PATH_FAILED");
    }
    else if(ctrl.ifsm_state == STATE_STOPPED)
    {
        ui->lb_FsmStateInner->setText("STATE_STOPPED");
    }
    else if(ctrl.ifsm_state == STATE_PAUSE)
    {
        ui->lb_FsmStateInner->setText("STATE_PAUSE");
    }

    // plot state
    if(ui_motor_state == UI_MOTOR_NOT_READY)
    {
        ui->lb_MotorState->setText("UI_MOTOR_NOT_READY");
    }
    else if(ui_motor_state == UI_MOTOR_READY)
    {
        ui->lb_MotorState->setText("UI_MOTOR_READY");
    }

    if(ui_loc_state == UI_LOC_NOT_READY)
    {
        ui->lb_LocState->setText("UI_LOC_NOT_READY");
    }
    else if(ui_loc_state == UI_LOC_BUSY)
    {
        ui->lb_LocState->setText("UI_LOC_BUSY");
    }
    else if(ui_loc_state == UI_LOC_GOOD)
    {
        ui->lb_LocState->setText("UI_LOC_GOOD");
    }
    else if(ui_loc_state == UI_LOC_FAIL)
    {
        ui->lb_LocState->setText("UI_LOC_FAIL");
    }

    if(ui_auto_state == UI_AUTO_NOT_READY)
    {
        ui->lb_AutoState->setText("UI_AUTO_NOT_READY");
    }
    else if(ui_auto_state == UI_AUTO_READY)
    {
        ui->lb_AutoState->setText("UI_AUTO_READY");
    }
    else if(ui_auto_state == UI_AUTO_MOVING)
    {
        ui->lb_AutoState->setText("UI_AUTO_MOVING");
    }
    else if(ui_auto_state == UI_AUTO_WAIT)
    {
        ui->lb_AutoState->setText("UI_AUTO_WAIT");
    }
    else if(ui_auto_state == UI_AUTO_PAUSE)
    {
        ui->lb_AutoState->setText("UI_AUTO_PAUSE");
    }

    if(ui_obs_state == UI_OBS_NONE)
    {
        ui->lb_ObsState->setText("UI_OBS_NONE");
    }
    else if(ui_obs_state == UI_OBS_NEAR)
    {
        ui->lb_ObsState->setText("UI_OBS_NEAR");
    }



    // plot que info
    QString que_info_str;
    que_info_str.sprintf("raw_q:%d, scan_q:%d, l2c_q:%d", (int)lidar.raw_que.unsafe_size(), (int)lidar.scan_que.unsafe_size(), (int)mobile.l2c.msg_que.unsafe_size());
    ui->lb_QueInfo->setText(que_info_str);

    // plot localization info
    QString loc_info_str;
    loc_info_str.sprintf("Inlier:%.3f, Err:%.3f", (double)slam.loc_inlier_ratio, (double)slam.loc_inlier_error);
    ui->lb_LocInfo->setText(loc_info_str);

    // plot mobile pose
    MOBILE_POSE mobile_pose = mobile.get_pose();
    QString mobile_pose_str;
    mobile_pose_str.sprintf("t: %f\npos: %.2f, %.2f, %.2f\nvel: %.2f, %.2f, %.2f\nvw: %.2f, %.2f", mobile_pose.t,
                            mobile_pose.pose[0], mobile_pose.pose[1], mobile_pose.pose[2]*R2D,
            mobile_pose.vel[0], mobile_pose.vel[1], mobile_pose.vel[2]*R2D,
            mobile_pose.vw[0], mobile_pose.vw[1]*R2D);

    IPC::MOBILE_POSE ipc_mobile_pose;
    ipc_mobile_pose.pose = cv::Vec3d(mobile_pose.pose[0], mobile_pose.pose[1], mobile_pose.pose[2]*R2D);
    ipc_mobile_pose.vel = cv::Vec3d(mobile_pose.vel[0], mobile_pose.vel[1], mobile_pose.vel[2]*R2D);
    ipc_mobile_pose.vw = cv::Vec2d(mobile_pose.vw[0], mobile_pose.vw[1]*R2D);

    ipc.set_mobile_pos(ipc_mobile_pose); //shared memory save

    //    IPC::POSE pose;
    //    IPC::MOBILE_POSE pose = ipc.get_move_where() ;

    //    QString posedddddd;
    //    int x_pose =pose.x;
    //    qDebug()<<"robot move : "<<pose.x<<pose.y<<pose.theta;

    ui->te_Pose->setText(mobile_pose_str);

    // plot mobile status
    MOBILE_STATUS mobile_status = mobile.get_status();
    QString mobile_status_str;
    mobile_status_str.sprintf("connection(m0, m1): %d, %d\nstatus(m0, m1): %d, %d\ntemperature(m0, m1): %d, %d, cur(m0, m1):%.2f, %.2f\ncharge, power, emo, remote state: %d, %d, %d, %d\nBAT(in, out, cur):%.3f, %.3f, %.3f\npower: %.3f\ntotal power: %.3f",
                              mobile_status.connection_m0, mobile_status.connection_m1, mobile_status.status_m0, mobile_status.status_m1, mobile_status.temp_m0, mobile_status.temp_m1,
                              (double)mobile_status.cur_m0/10.0, (double)mobile_status.cur_m1/10.0,
                              mobile_status.charge_state, mobile_status.power_state, mobile_status.emo_state, mobile_status.remote_state,
                              mobile_status.bat_in, mobile_status.bat_out, mobile_status.bat_current,
                              mobile_status.power, mobile_status.total_power);
    ui->lb_RobotStatus->setText(mobile_status_str);

    // plot pose
    cv::Vec3d cur_pose = slam.get_cur_pose();
    QString pose_str;
    pose_str.sprintf("LOC: %.2f, %.2f, %.1f", cur_pose[0], cur_pose[1], cur_pose[2]*R2D);
    ui->lb_Pose->setText(pose_str);

    // image plot
    cv::Mat plot_img;
    if(plot_que.try_pop(plot_img))
    {
        ui->gv_Screen1->map.setPixmap(QPixmap::fromImage(mat_to_qimage_cpy(plot_img)));
        plot_que.clear();
    }

    loop_count++;
}

void MainWindow::watchdog_loop()
{
    // check status
    MOBILE_STATUS status = mobile.get_status();
    if(status.is_ok)
    {
        // motor
        if(status.status_m0 == 1 && status.status_m1 == 1)
        {
            ui_motor_state = UI_MOTOR_READY;
        }
        else
        {
            ui_motor_state = UI_MOTOR_NOT_READY;
        }

        // emo release
        if(pre_status.emo_state == 1 && status.emo_state == 0)
        {
            logger.write("[WATCHDOG] EMO pushed", true);
        }
        else if(pre_status.emo_state == 0 && status.emo_state == 1)
        {
            logger.write("[WATCHDOG] EMO released", true);

            mobile.motor_init();
            ctrl.ofsm_state = STATE_IDLE;
        }

        pre_status = status;
    }

    // check localization
    if(slam.loc_inlier_ratio >= robot_config.robot_icp_ratio && slam.loc_inlier_error <= robot_config.robot_icp_error)
    {
        ui_loc_state = UI_LOC_GOOD;
        loc_fail_cnt = 0;
    }
    else
    {
        loc_fail_cnt++;
        if(loc_fail_cnt >= 30 && ui_loc_state != UI_LOC_NOT_READY)
        {
            bt_ManualStop();
            ui_loc_state = UI_LOC_FAIL;

            QString str;
            str.sprintf("[WATCHDOG] LOC failed, ir:%f, ie:%f", (double)slam.loc_inlier_ratio, (double)slam.loc_inlier_error);
            logger.write(str, true);

            loc_fail_cnt = 0;
        }
    }

    // check obstacle
    cv::Vec3d cur_pose = slam.get_cur_pose();
    if(unimap.is_near_static_obs(cv::Vec2d(cur_pose[0], cur_pose[1])))
    {
        ui_obs_state = UI_OBS_NEAR;
    }
    else
    {
        ui_obs_state = UI_OBS_NONE;
    }

    // check running
    bool is_charging = (status.is_ok == true && status.charge_state == 1);
    if(ui_motor_state == UI_MOTOR_READY && ui_loc_state == UI_LOC_GOOD && is_charging == false)
    {
        if(ctrl.ofsm_state == STATE_IDLE)
        {
            ui_auto_state = UI_AUTO_READY;
        }
        else if(ctrl.ofsm_state == STATE_FAILED)
        {
            ui_auto_state = UI_AUTO_NOT_READY;
        }
        else if(ctrl.ofsm_state == STATE_DRIVING)
        {
            ui_auto_state = UI_AUTO_MOVING;
        }
        else if(ctrl.ofsm_state == STATE_WAITING)
        {
            ui_auto_state = UI_AUTO_WAIT;
        }

        if(ctrl.is_pause == true)
        {
            ui_auto_state = UI_AUTO_PAUSE;
        }
    }
    else
    {
        ui_auto_state = UI_AUTO_NOT_READY;
    }
}

void MainWindow::backLoop()
{
    double dt = 0.1;
    double pre_loop_time = get_time();
    double last_pub_time = get_time();

    while(backFlag)
    {
        double cur_time = get_time();

        // publish status for ui
        publish_ipc_status();
        if(cur_time - last_pub_time >= 1.0)
        {
            // publish path for ui
            publish_ipc_path();

            // publish map for ui
            if(slam.is_slam)
            {
                publish_ipc_map();
            }

            if(slam.is_obs)
            {
                // publish map obs for ui
                publish_ipc_map_obs();
            }

            last_pub_time = cur_time;
        }

        // publish cam
        if(pub_cam_flag == true)
        {
            publish_ipc_cam();
            pub_cam_flag = false;
        }

        // publish shared path for multi-robot
        if(robot_config.robot_use_multi)
        {
            publish_shared_path();
        }

        // make plot image
        if(ui->ckb_PlotEnable->isChecked())
        {
            // plot map info
            cv::Mat plot_img = unimap.get_map_plot();
            if(!plot_img.empty())
            {
                cv::Vec3d cur_pose(0,0,0);
                cv::Vec2d cur_vw(0,0);
                std::vector<cv::Vec2d> cur_scan;

                if(slam.is_slam)
                {
                    // has location info
                    cur_pose = slam.get_cur_pose();
                    cur_scan = slam.get_cur_scan_global();
                    cur_vw = mobile.get_pose().vw;

#ifdef USE_SIM
                    cur_pose = mobile.get_pose().pose;
#endif
                }
                else if(slam.is_loc)
                {
                    // has location info
                    cur_pose = slam.get_cur_pose();
                    cur_scan = slam.get_cur_scan_global();
                    cur_vw = mobile.get_pose().vw;
                }
                else
                {
                    // no location info
                    cur_scan = lidar.get_cur_scan();
                    cur_vw = mobile.get_pose().vw;
                }

                // map data loaded
                if(unimap.is_loaded)
                {
                    // draw locations
                    for(size_t p = 0; p < unimap.serving_loc_list.size(); p++)
                    {
                        unimap.draw_robot(plot_img, unimap.serving_loc_list[p], cv::Scalar(0,128,0), 1);
                    }

                    for(size_t p = 0; p < unimap.patrol_loc_list.size(); p++)
                    {
                        unimap.draw_robot(plot_img, unimap.patrol_loc_list[p], cv::Scalar(128,128,0), 1);
                    }

                    for(size_t p = 0; p < unimap.charging_loc_list.size(); p++)
                    {
                        unimap.draw_robot(plot_img, unimap.charging_loc_list[p], cv::Scalar(0,255,255), 1);
                    }

                    for(size_t p = 0; p < unimap.resting_loc_list.size(); p++)
                    {
                        unimap.draw_robot(plot_img, unimap.resting_loc_list[p], cv::Scalar(255,255,255), 1);
                    }
                }

                if(robot_config.robot_use_multi)
                {
                    // draw other robots
                    std::vector<ROBOT_SHARED> _robots = ctrl.get_robots();
                    for(size_t p = 0; p < _robots.size(); p++)
                    {
                        if(_robots[p].id == -1 || _robots[p].id == robot_config.robot_id)
                        {
                            continue;
                        }

                        unimap.draw_other_robot(plot_img, _robots[p]);
                    }
                }

                // draw current sensing data
                unimap.draw_path(plot_img, ctrl.get_cur_path(), cv::Scalar(0,255,0));
                unimap.draw_robot(plot_img, ui->gv_Screen1->target_pose, cv::Scalar(255, 255, 0), 1);
                unimap.draw_robot(plot_img, cur_pose, cv::Scalar(0,255,255), 1);
                unimap.draw_lidar(plot_img, cv::Vec3d(0,0,0), cur_scan, cv::Scalar(0,0,255));
                unimap.draw_obs_map(plot_img);

                // obstacle map plot
                if(slam.is_obs)
                {
                    unimap.draw_obs_map0(plot_img);
                }

                // draw local trajectory
                std::vector<cv::Vec6d> traj_drv = ctrl.calc_trajectory(cur_vw, 1.5, 0.2, cur_pose, cur_vw);
                unimap.draw_trajectory(plot_img, traj_drv, cv::Scalar(255,255,255), 1);

                // plot image
                cv::flip(plot_img, plot_img, 0);
                cv::rotate(plot_img, plot_img, cv::ROTATE_90_COUNTERCLOCKWISE); // image north is +x axis
                plot_que.push(plot_img);
            }
        }

        // for real time loop
        double cur_loop_time = get_time();
        double delta_loop_time = cur_loop_time - pre_loop_time;
        if(delta_loop_time < dt)
        {
            int sleep_ms = (dt-delta_loop_time)*1000;
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
        }
        else
        {
            printf("[PLOT] loop time drift, dt:%f\n", delta_loop_time);
        }
        pre_loop_time = get_time();

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}
