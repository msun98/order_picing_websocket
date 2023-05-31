#include "RBLAN2CAN.h"

RBLAN2CAN::RBLAN2CAN()
{
    sockConnectionStatus = false;

    connect(&socket, SIGNAL(connected()), this, SLOT(onSockConnected()));
    connect(&socket, SIGNAL(disconnected()), this, SLOT(onSockDisconnected()));
    connect(&socket, SIGNAL(readyRead()), this, SLOT(onSockReadyRead()));
    connect(&msg_timer, SIGNAL(timeout()), this, SLOT(msgLoop()));

    socket.connectToHost(LAN2CAN_IP, 1977);    
}

RBLAN2CAN::~RBLAN2CAN()
{
    msg_que.clear();
    SendLinearAngularVel(0,0);
}

void RBLAN2CAN::onSockConnected()
{
    socket.setSocketOption(QAbstractSocket::LowDelayOption, 1);

    sockConnectionStatus = true;
    msg_timer.setInterval(1);
    msg_timer.setTimerType(Qt::PreciseTimer);
    msg_timer.start();

    emit l2c_connected();
    std::cout << "LAN2CAN Connected" << std::endl;
}

void RBLAN2CAN::onSockDisconnected()
{
    sockConnectionStatus = false;
    msg_timer.stop();
    std::cout << "LAN2CAN Disonnected" << std::endl;
}

void RBLAN2CAN::onSockReadyRead()
{
    QByteArray buf = socket.readAll();

    const int packet_size = 73;
    bool is_fine = false;
    if(buf.length() == packet_size)
    {
        is_fine = true;
    }

    int cnt = 0;
    while(buf.length() >= packet_size)
    {
        uchar *_buf = (uchar*)buf.data();
        if(_buf[0] == 0x24 && _buf[5] == 0xA2 && _buf[packet_size-1] == 0x25)
        {
            int index=6;
            int dlc=1;
            int dlc_f=4;

            uint32_t tick;
            memcpy(&tick, &_buf[index], dlc_f);     index=index+dlc_f;

            double mobile_t = tick*0.002;
            if(cnt < 10 && is_fine)
            {
                double pc_t = get_time();
                offset_t = pc_t - mobile_t;
                cnt++;
            }

            uint8_t connection_status_m0, connection_status_m1;
            connection_status_m0 = _buf[index];     index=index+dlc;
            connection_status_m1 = _buf[index];     index=index+dlc;

            float x_dot, y_dot, th_dot;
            memcpy(&x_dot, &_buf[index], dlc_f);     index=index+dlc_f;
            memcpy(&y_dot, &_buf[index], dlc_f);     index=index+dlc_f;
            memcpy(&th_dot, &_buf[index], dlc_f);    index=index+dlc_f;

            float x, y, th;
            memcpy(&x, &_buf[index], dlc_f);     index=index+dlc_f;
            memcpy(&y, &_buf[index], dlc_f);     index=index+dlc_f;
            memcpy(&th, &_buf[index], dlc_f);    index=index+dlc_f;

            float local_v, local_w;
            memcpy(&local_v, &_buf[index], dlc_f);     index=index+dlc_f;
            memcpy(&local_w, &_buf[index], dlc_f);     index=index+dlc_f;

            uint8_t stat_m0, stat_m1;
            stat_m0 = _buf[index];     index=index+dlc;
            stat_m1 = _buf[index];     index=index+dlc;

            uint8_t temp_m0, temp_m1;
            temp_m0 = _buf[index];     index=index+dlc;
            temp_m1 = _buf[index];     index=index+dlc;

            uint8_t charge_state, power_state, emo_state, remote_state;
            charge_state = _buf[index];     index=index+dlc;
            power_state = _buf[index];      index=index+dlc;
            emo_state = _buf[index];        index=index+dlc;
            remote_state = _buf[index];     index=index+dlc;

            float bat_in, bat_out, bat_cur, power, total_used_power;
            memcpy(&bat_in, &_buf[index], dlc_f);               index=index+dlc_f;
            memcpy(&bat_out, &_buf[index], dlc_f);              index=index+dlc_f;
            memcpy(&bat_cur, &_buf[index], dlc_f);              index=index+dlc_f;
            memcpy(&power, &_buf[index], dlc_f);                index=index+dlc_f;
            memcpy(&total_used_power, &_buf[index], dlc_f);     index=index+dlc_f;

            // received mobile pose update
            MOBILE_POSE mobile_pose;
            mobile_pose.t = mobile_t + offset_t;
            mobile_pose.pose = cv::Vec3d(x, y, toWrap(th));
            mobile_pose.vel = cv::Vec3d(x_dot, y_dot, th_dot);
            mobile_pose.vw = cv::Vec2d(local_v, local_w);            
            emit pose_received(mobile_pose);

            // received mobile status update
            MOBILE_STATUS mobile_status;
            mobile_status.is_ok = true;
            mobile_status.connection_m0 = connection_status_m0;
            mobile_status.connection_m1 = connection_status_m1;
            mobile_status.status_m0 = stat_m0;
            mobile_status.status_m1 = stat_m1;
            mobile_status.temp_m0 = temp_m0;
            mobile_status.temp_m1 = temp_m1;
            mobile_status.charge_state = charge_state;
            mobile_status.power_state = power_state;
            mobile_status.emo_state = emo_state;
            mobile_status.remote_state = remote_state;
            mobile_status.bat_in = bat_in;
            mobile_status.bat_out = bat_out;
            mobile_status.bat_current = bat_cur;
            mobile_status.power = power;
            mobile_status.total_power = total_used_power;
            emit status_received(mobile_status);
        }

        buf.remove(0, packet_size);
    }
}

void RBLAN2CAN::SendMotorInit(float kp, float ki, float kd)
{
    std::vector<uchar> send_byte(25, 0);
    send_byte[0] = 0x24;

    uint16_t size = 6+8+8;
    memcpy(&send_byte[1], &size, 2); // size
    send_byte[3] = 0x00;
    send_byte[4] = 0x00;

    send_byte[5] = 0xA0;
    send_byte[6] = 0x00;
    send_byte[7] = 100; // cmd motor init

    memcpy(&send_byte[8], &kp, 4);
    memcpy(&send_byte[12], &ki, 4);
    memcpy(&send_byte[16], &kd, 4);
    send_byte[24] = 0x25;

    msg_que.push(send_byte);
}

void RBLAN2CAN::SendMotorGain(float kp, float ki, float kd)
{
    std::vector<uchar> send_byte(25, 0);
    send_byte[0] = 0x24;

    uint16_t size = 6+8+8;
    memcpy(&send_byte[1], &size, 2); // size
    send_byte[3] = 0x00;
    send_byte[4] = 0x00;

    send_byte[5] = 0xA0;
    send_byte[6] = 0x00;
    send_byte[7] = 106; // cmd motor init

    memcpy(&send_byte[8], &kp, 4);
    memcpy(&send_byte[12], &ki, 4);
    memcpy(&send_byte[16], &kd, 4);
    send_byte[24] = 0x25;

    msg_que.push(send_byte);
}

void RBLAN2CAN::SendMotorCurGain(float kp, float ki, float kd)
{
    std::vector<uchar> send_byte(25, 0);
    send_byte[0] = 0x24;

    uint16_t size = 6+8+8;
    memcpy(&send_byte[1], &size, 2); // size
    send_byte[3] = 0x00;
    send_byte[4] = 0x00;

    send_byte[5] = 0xA0;
    send_byte[6] = 0x00;
    send_byte[7] = 107; // cmd motor init

    memcpy(&send_byte[8], &kp, 4);
    memcpy(&send_byte[12], &ki, 4);
    memcpy(&send_byte[16], &kd, 4);
    send_byte[24] = 0x25;

    msg_que.push(send_byte);
}

void RBLAN2CAN::SendMotorLimitUpdate(float v_limit, float v_acc_limit, float w_limit, float w_acc_limit)
{
    std::vector<uchar> send_byte(25, 0);
    send_byte[0] = 0x24;

    uint16_t size = 6+8+8;
    memcpy(&send_byte[1], &size, 2); // size
    send_byte[3] = 0x00;
    send_byte[4] = 0x00;

    send_byte[5] = 0xA0;
    send_byte[6] = 0x00;
    send_byte[7] = 108; // cmd error clear

    memcpy(&send_byte[8], &v_limit, 4); // m/s
    memcpy(&send_byte[12], &v_acc_limit, 4); // m/s^2
    memcpy(&send_byte[16], &w_limit, 4); // rad/s
    memcpy(&send_byte[20], &w_acc_limit, 4); // rad/s^2
    send_byte[24] = 0x25;

    msg_que.push(send_byte);
}
void RBLAN2CAN::SendMotormTUpdate(float m, float T)
{
    std::vector<uchar> send_byte(25, 0);
    send_byte[0] = 0x24;

    uint16_t size = 6+8+8;
    memcpy(&send_byte[1], &size, 2); // size
    send_byte[3] = 0x00;
    send_byte[4] = 0x00;

    send_byte[5] = 0xA0;
    send_byte[6] = 0x00;
    send_byte[7] = 109; // cmd error clear

    memcpy(&send_byte[8], &m, 4);
    memcpy(&send_byte[12], &T, 4);
    send_byte[24] = 0x25;

    msg_que.push(send_byte);
}

void RBLAN2CAN::SendMotorID(float id_r, float id_l)
{
    std::vector<uchar> send_byte(25, 0);
    send_byte[0] = 0x24;

    uint16_t size = 6+8+8;
    memcpy(&send_byte[1], &size, 2); // size
    send_byte[3] = 0x00;
    send_byte[4] = 0x00;

    send_byte[5] = 0xA0;
    send_byte[6] = 0x00;
    send_byte[7] = 101; // cmd motor init

    memcpy(&send_byte[8], &id_r, 4);
    memcpy(&send_byte[12], &id_l, 4);
    send_byte[24] = 0x25;

    msg_que.push(send_byte);
}

void RBLAN2CAN::SendMotorLimitVel(float limit_v, float limit_w)
{
    std::vector<uchar> send_byte(25, 0);
    send_byte[0] = 0x24;

    uint16_t size = 6+8+8;
    memcpy(&send_byte[1], &size, 2); // size
    send_byte[3] = 0x00;
    send_byte[4] = 0x00;

    send_byte[5] = 0xA0;
    send_byte[6] = 0x00;
    send_byte[7] = 102; // cmd motor init

    memcpy(&send_byte[8], &limit_v, 4);
    memcpy(&send_byte[12], &limit_w, 4);
    send_byte[24] = 0x25;

    msg_que.push(send_byte);
}

void RBLAN2CAN::SendMotorLimitAcc(float limit_v_acc, float limit_w_acc)
{
    std::vector<uchar> send_byte(25, 0);
    send_byte[0] = 0x24;

    uint16_t size = 6+8+8;
    memcpy(&send_byte[1], &size, 2); // size
    send_byte[3] = 0x00;
    send_byte[4] = 0x00;

    send_byte[5] = 0xA0;
    send_byte[6] = 0x00;
    send_byte[7] = 103; // cmd motor init

    memcpy(&send_byte[8], &limit_v_acc, 4);
    memcpy(&send_byte[12], &limit_w_acc, 4);
    send_byte[24] = 0x25;

    msg_que.push(send_byte);
}

void RBLAN2CAN::SendMotorWheelSpec(float wheel_base, float wheel_radius)
{
    std::vector<uchar> send_byte(25, 0);
    send_byte[0] = 0x24;

    uint16_t size = 6+8+8;
    memcpy(&send_byte[1], &size, 2); // size
    send_byte[3] = 0x00;
    send_byte[4] = 0x00;

    send_byte[5] = 0xA0;
    send_byte[6] = 0x00;
    send_byte[7] = 104; // cmd motor init

    memcpy(&send_byte[8], &wheel_base, 4);
    memcpy(&send_byte[12], &wheel_radius, 4);
    send_byte[24] = 0x25;

    msg_que.push(send_byte);
}

void RBLAN2CAN::SendMotorSpec(float wheel_dir, float gear_ratio)
{
    std::vector<uchar> send_byte(25, 0);
    send_byte[0] = 0x24;

    uint16_t size = 6+8+8;
    memcpy(&send_byte[1], &size, 2); // size
    send_byte[3] = 0x00;
    send_byte[4] = 0x00;

    send_byte[5] = 0xA0;
    send_byte[6] = 0x00;
    send_byte[7] = 105; // cmd motor init

    memcpy(&send_byte[8], &wheel_dir, 4);
    memcpy(&send_byte[12], &gear_ratio, 4);
    send_byte[24] = 0x25;

    msg_que.push(send_byte);
}

void RBLAN2CAN::SendLinearAngularVel(float v, float w)
{
    std::vector<uchar> send_byte(25, 0);
    send_byte[0] = 0x24;

    uint16_t size = 6+8+8;
    memcpy(&send_byte[1], &size, 2); // size
    send_byte[3] = 0x00;
    send_byte[4] = 0x00;

    send_byte[5] = 0xA0;
    send_byte[6] = 0x00;
    send_byte[7] = 10; // cmd move

    memcpy(&send_byte[8], &v, 4); // param1 linear vel
    memcpy(&send_byte[12], &w, 4); // param2 angular vel
    send_byte[24] = 0x25;

    msg_que.push(send_byte);
}

void RBLAN2CAN::SendLed(int target, int mode)
{
    std::vector<uchar> send_byte(25, 0);
    send_byte[0] = 0x24;

    uint16_t size = 6+8+8;
    memcpy(&send_byte[1], &size, 2); // size
    send_byte[3] = 0x00;
    send_byte[4] = 0x00;

    send_byte[5] = 0xB0;
    send_byte[6] = target; // 0~1
    send_byte[7] = mode; // cmd move

    send_byte[24] = 0x25;

    msg_que.push(send_byte);
}

void RBLAN2CAN::msgLoop()
{
    std::vector<uchar> msg;    
    if(msg_que.try_pop(msg))
    {
        if(sockConnectionStatus == true)
        {
            socket.write((char*)msg.data(), msg.size());            
        }
    }
}
