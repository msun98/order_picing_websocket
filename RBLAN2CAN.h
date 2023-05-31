#ifndef RBLAN2CAN_H
#define RBLAN2CAN_H

#include "global_defines.h"

#include <QObject>
#include <QtNetwork>
#include <iostream>

#define LAN2CAN_IP  "192.168.2.100"

class RBLAN2CAN : public QObject
{
    Q_OBJECT

public:
    RBLAN2CAN();
    ~RBLAN2CAN();

    QTcpSocket socket;

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

    tbb::concurrent_queue<std::vector<uchar>> msg_que;
    QTimer msg_timer;
    double offset_t = 0;

public slots:
    void onSockConnected();
    void onSockDisconnected();
    void onSockReadyRead();
    void msgLoop();

private:
    std::atomic<bool> sockConnectionStatus;

signals:
    void l2c_connected();
    void pose_received(MOBILE_POSE mobile_pose);
    void status_received(MOBILE_STATUS mobile_status);

};

#endif // RBLAN2CAN_H
