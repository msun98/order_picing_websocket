#include "mobile.h"

MOBILE::MOBILE(QObject *parent)
    : QObject(parent)
    , l2c(this)
{
    connect(&l2c, SIGNAL(connected()), this, SLOT(motor_init()));
    connect(&l2c, SIGNAL(pose_received(MOBILE_POSE)), this, SLOT(pose_received(MOBILE_POSE)));
    connect(&l2c, SIGNAL(status_received(MOBILE_STATUS)), this, SLOT(status_received(MOBILE_STATUS)));

    last_v = 0;
    last_w = 0;
}

MOBILE::~MOBILE()
{
    if(poolThread != NULL)
    {
        poolFlag = false;
        poolThread->join();
        poolThread = NULL;
    }
}

void MOBILE::init()
{
    l2c.init();

    if (poolThread == NULL)
    {
        poolFlag = true;
        poolThread = new std::thread(&MOBILE::poolLoop, this);
    }
}

MOBILE_POSE MOBILE::get_pose()
{
    mtx.lock();
    MOBILE_POSE _pose = pose;
    mtx.unlock();

    return _pose;
}

std::vector<MOBILE_POSE> MOBILE::get_poses()
{
    mtx.lock();
    std::vector<MOBILE_POSE> _poses = poses;
    mtx.unlock();

    return _poses;
}

MOBILE_STATUS MOBILE::get_status()
{
    mtx.lock();
    MOBILE_STATUS _status = status;
    mtx.unlock();

    return _status;
}

void MOBILE::motor_init()
{
    l2c.SendMotorID(robot_config.motor_right_id, robot_config.motor_left_id);
    printf("[MOTORINIT] id_l:%d, id_r:%d\n", robot_config.motor_right_id, robot_config.motor_left_id);

    l2c.SendMotorSpec(robot_config.motor_wheel_dir, robot_config.motor_gear_ratio);
    printf("[MOTORINIT] wheel_dir:%f, gear_ratio:%f\n", robot_config.motor_wheel_dir, robot_config.motor_gear_ratio);

    l2c.SendMotorWheelSpec(robot_config.robot_wheel_base, robot_config.robot_wheel_radius);
    printf("[MOTORINIT] wheel_base:%f, wheel_radius:%f\n", robot_config.robot_wheel_base, robot_config.robot_wheel_radius);

    l2c.SendMotorLimitVel(robot_config.motor_limit_v, robot_config.motor_limit_w);
    printf("[MOTORINIT] limit_v:%f, limit_w:%f(%f)\n", robot_config.motor_limit_v, robot_config.motor_limit_w*R2D, robot_config.motor_limit_w);

    l2c.SendMotorLimitAcc(robot_config.motor_limit_v_acc, robot_config.motor_limit_w_acc);
    printf("[MOTORINIT] limit_v_acc:%f, limit_w_acc:%f(%f)\n", robot_config.motor_limit_v_acc, robot_config.motor_limit_w_acc*R2D, robot_config.motor_limit_w_acc);

    l2c.SendMotorInit(robot_config.motor_k_p, robot_config.motor_k_i, robot_config.motor_k_d);
    printf("[MOTORINIT] kp:%f, ki:%f, kd:%f\n", robot_config.motor_k_p, robot_config.motor_k_i, robot_config.motor_k_d);

    logger.write("[MOBILE] send motor init", true);
}

void MOBILE::motor_gain(int Kp, int Ki, int Kd)
{
    l2c.SendMotorGain(Kp, Ki, Kd);
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
}

void MOBILE::motor_cur_gain(int Kp, int Ki, int Kd)
{
    l2c.SendMotorCurGain(Kp, Ki, Kd);
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
}

void MOBILE::motor_limit_update(float v_limit, float v_acc_limit, float w_limit, float w_acc_limit)
{
    l2c.SendMotorLimitUpdate(v_limit, v_acc_limit, w_limit, w_acc_limit);
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
}

void MOBILE::motor_mT_update(float m, float T)
{
    l2c.SendMotormTUpdate(m, T);
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
}

void MOBILE::move(double v, double w)
{
    // storing last command
    last_v = v;
    last_w = w;

    // check motor init
    MOBILE_STATUS _status = get_status();
    if(_status.is_ok != true || _status.status_m0 != 1 || _status.status_m1 != 1 || _status.emo_state != 1 || _status.charge_state != 0)
    {
        printf("check mobile status\n");
        return;
    }

    if(!isfinite(v))
    {
        printf("nan occured v, set zero\n");
        v = 0;
    }

    if(!isfinite(w))
    {
        printf("nan occured w, set zero\n");
        w = 0;
    }

    if(v == 0 && w == 0)
    {
        l2c.msg_que.clear();
    }
    l2c.SendLinearAngularVel(v, w);
}

void MOBILE::led(int target, int mode)
{
    l2c.SendLed(target, mode);
}

void MOBILE::poolLoop()
{
    while(poolFlag)
    {
        MOBILE_POSE _pose;
        while(l2c.pose_que.try_pop(_pose))
        {
            mtx.lock();
            pose = _pose;
            poses.push_back(_pose);
            if(poses.size() > 100)
            {
                poses.erase(poses.begin());
            }
            mtx.unlock();
        }

        MOBILE_STATUS _status;
        if(l2c.status_que.try_pop(_status))
        {
            mtx.lock();
            status = _status;
            mtx.unlock();
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

}

void MOBILE::pose_received(MOBILE_POSE mobile_pose)
{
    mtx.lock();
    pose = mobile_pose;

    poses.push_back(mobile_pose);
    if(poses.size() > 100)
    {
        poses.erase(poses.begin());
    }
    mtx.unlock();
}

void MOBILE::status_received(MOBILE_STATUS mobile_status)
{
    mtx.lock();
    status = mobile_status;
    mtx.unlock();
}
