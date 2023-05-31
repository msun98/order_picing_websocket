#ifndef L2C_H
#define L2C_H

#include "global_defines.h"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <netdb.h>

#include <QObject>

class L2C : public QObject
{
    Q_OBJECT
public:
    explicit L2C(QObject *parent = nullptr);
    ~L2C();

    // initialize
    void init();

    // comm loop
    std::atomic<bool> commFlag;
    std::thread* commThread = NULL;
    void commLoop();

    std::atomic<bool> commFlag2;
    std::thread* commThread2 = NULL;
    void commLoop2();

    // message que
    tbb::concurrent_queue<std::vector<uchar>> msg_que;
    tbb::concurrent_queue<MOBILE_POSE> pose_que;
    tbb::concurrent_queue<MOBILE_STATUS> status_que;

    // timeoffset
    double offset_t = 0;

    // connection info
    int fd = 0;
    std::atomic<bool> is_connected;
    std::atomic<bool> is_resync;

    // send funcs
    void SendMotorInit(float kp, float ki, float kd);
    void SendMotorGain(float kp, float ki, float kd);
    void SendMotorCurGain(float kp, float ki, float kd);
    void SendMotorLimitUpdate(float v_limit, float v_acc_limit, float w_limit, float w_acc_limit);
    void SendMotormTUpdate(float m, float T);
    void SendMotorID(float id_r, float id_l);
    void SendMotorLimitVel(float limit_v, float limit_w);
    void SendMotorLimitAcc(float limit_v_acc, float limit_w_acc);
    void SendMotorWheelSpec(float wheel_base, float wheel_radius);
    void SendMotorSpec(float wheel_dir, float gear_ratio);
    void SendLinearAngularVel(float v, float w);
    void SendLed(int target, int mode);

signals:
    void connected();
    void pose_received(MOBILE_POSE mobile_pose);
    void status_received(MOBILE_STATUS mobile_status);
};

#endif // L2C_H
