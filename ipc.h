#ifndef IPC_H
#define IPC_H

#include <QObject>
#include <QSharedMemory>
#include <QDebug>
class IPC : public QObject
{
    Q_OBJECT

public:
    struct CMD
    {
        uint32_t tick = 0;
        int32_t cmd = 0;
        uint8_t params[255] = {0,};

        CMD()
        {
        }
        CMD(const CMD& p)
        {
            tick = p.tick;
            cmd = p.cmd;
            memcpy(params, p.params, 255);
        }
    };

    struct STATUS
            // 구조체 변수 정의함과 동시에 생성자에 의해서 특정값으로 초기화됨
    {
        uint32_t   tick = 0;
        int8_t     connection_m0 = 0;
        int8_t     connection_m1 = 0;
        int8_t     status_m0 = 0;
        int8_t     status_m1 = 0;
        int8_t     temp_m0 = 0;
        int8_t     temp_m1 = 0;
        int8_t     temp_ex_m0 = 0;
        int8_t     temp_ex_m1 = 0;
        int8_t     cur_m0 = 0;
        int8_t     cur_m1 = 0;
        int8_t     status_charge = 0;
        int8_t     status_power = 0;
        int8_t     status_emo = 0;
        int8_t     status_remote = 0;
        float      bat_in = 0;
        float      bat_out = 0;
        float      bat_cur = 0;
        float      power = 0;
        float      total_power = 0;
        int8_t     ui_motor_state = 0;
        int8_t     ui_loc_state = 0;
        int8_t     ui_auto_state = 0;
        int8_t     ui_obs_state = 0;
        float      robot_pose[3] = {0,};
        float      robot_scan[360] = {0,};

        STATUS()
        {
        }
        STATUS(const STATUS& p)
        {
            tick = p.tick;
            connection_m0 = p.connection_m0;
            connection_m1 = p.connection_m1;
            status_m0 = p.status_m0;
            status_m1 = p.status_m1;
            temp_m0 = p.temp_m0;
            temp_m1 = p.temp_m1;
            temp_ex_m0 = p.temp_ex_m0;
            temp_ex_m1 = p.temp_ex_m1;
            cur_m0 = p.cur_m0;
            cur_m1 = p.cur_m1;
            status_charge = p.status_charge;
            status_power = p.status_power;
            status_emo = p.status_emo;
            status_remote = p.status_remote;
            bat_in = p.bat_in;
            bat_out = p.bat_out;
            bat_cur = p.bat_cur;
            power = p.power;
            total_power = p.total_power;
            ui_motor_state = p.ui_motor_state;
            ui_loc_state = p.ui_loc_state;
            ui_auto_state = p.ui_auto_state;
            ui_obs_state = p.ui_obs_state;
            memcpy(robot_pose, p.robot_pose, sizeof(float)*3);
            memcpy(robot_scan, p.robot_scan, sizeof(float)*360);
        }
    };

    struct PATH
    {
        uint32_t tick = 0;
        int32_t num = 0;
        float x[512] = {0,};//모든 배열의 요소를 0으로 초기화(only 0만 가느)
        float y[512] = {0,};

        PATH()
        {
        }
        PATH(const PATH& p)
        {
            tick = p.tick;
            num = p.num;
            memcpy(x, p.x, sizeof(float)*512);
            memcpy(y, p.y, sizeof(float)*512);
        }
    };

    struct MAP
    {
//        https://m.blog.naver.com/PostView.naver?isHttpsRedirect=true&blogId=kyed203&logNo=220056487775
        uint32_t tick = 0;
        uint32_t width = 1000;
        uint32_t height = 1000;
        uint8_t buf[1000*1000] = {0,};

        MAP()
        {
        }
        MAP(const MAP& p)
        {
            tick = p.tick;
            width = p.width;
            height = p.height;
            memcpy(buf, p.buf, 1000*1000);
        }
    };

    struct IMG
    {
        uint32_t tick = 0;        
        uint8_t serial[255] = {0,};
        uint32_t width = 480;
        uint32_t height = 270;
        uint8_t buf[480*270] = {0,};

        IMG()
        {
        }
        IMG(const IMG& p)
        {
            tick = p.tick;
            width = p.width;
            height = p.height;
            memcpy(serial, p.serial, 255);
            memcpy(buf, p.buf, 480*270);
        }
    };

//for yujin websocket
    struct POSE
    {
        uint32_t tick = 0;
        float x = 0;
        float y = 0;
        float theta = 0;

        POSE()
        {
        }
        POSE(const POSE& p)
        {
            tick = p.tick;
            x = p.x;
            y = p.y;
            theta = p.theta;
        }
    };


public:
    explicit IPC(QObject *parent = nullptr);
    ~IPC();

    std::atomic<uint32_t> tick;

    QSharedMemory shm_cmd;
    QSharedMemory shm_status;
    QSharedMemory shm_path;
    QSharedMemory shm_map;
    QSharedMemory shm_obs;
    QSharedMemory shm_cam0;
    QSharedMemory shm_cam1;
    QSharedMemory shm_move;

    CMD get_cmd();
    STATUS get_status();
    PATH get_path();
    MAP get_map();
    MAP get_obs();
    IMG get_cam0();
    IMG get_cam1();
    POSE get_move_where(); //어디로 갈지 유진 로봇으로 부터 받는 코드.


    void set_cmd(IPC::CMD val);
    void set_status(IPC::STATUS val);
    void set_path(IPC::PATH val);
    void set_map(IPC::MAP val);
    void set_obs(IPC::MAP val);
    void set_cam0(IPC::IMG val);
    void set_cam1(IPC::IMG val);
    void set_move_where(IPC::POSE val);

signals:

};

#endif // IPC_H
