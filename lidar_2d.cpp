#include "lidar_2d.h"

LIDAR_2D::LIDAR_2D(QObject *parent) : QObject(parent)
{
}

LIDAR_2D::~LIDAR_2D()
{
    if(grabThread != NULL)
    {
        grabFlag = false;
        grabThread->join();
        grabThread = NULL;
    }

    if(aThread != NULL)
    {
        aFlag = false;
        aThread->join();
        aThread = NULL;
    }
}

void LIDAR_2D::init(MOBILE *_mobile)
{
    mobile = _mobile;

    // start grab loop
    if (grabThread == NULL)
    {
        grabFlag = true;
        grabThread = new std::thread(&LIDAR_2D::grabLoop, this);
    }

    if (aThread == NULL)
    {
        aFlag = true;
        aThread = new std::thread(&LIDAR_2D::aLoop, this);
    }
}

void LIDAR_2D::grabLoop()
{
    //sudo adduser $USER dialout
    //return;

    sl::ILidarDriver* drv = *sl::createLidarDriver();
    if(!drv)
    {
        printf("lidar driver init failed\n");
        logger.write("[LIDAR] driver init failed", true);
        return;
    }

    sl::IChannel* channel = (*sl::createSerialPortChannel("/dev/ttyRP0", 256000));
    if(!channel->open())
    {
        logger.write("[LIDAR] port open failed", true);
        return;
    }
    else
    {
        // close for next operation
        channel->close();
    }

    if(drv->connect(channel) != SL_RESULT_OK)
    {
        printf("lidar connection failed\n");
        logger.write("[LIDAR] connection failed", true);
        return;
    }

    std::vector<sl::LidarScanMode> modes;
    drv->getAllSupportedScanModes(modes);

    sl::LidarScanMode mode;
    float min = modes[0].us_per_sample;
    for(size_t p = 0; p < modes.size(); p++)
    {
        printf("%s[%d] us_per_sample:%f\n", modes[p].scan_mode, modes[p].id, modes[p].us_per_sample);
        if(modes[p].us_per_sample < min)
        {
            min = modes[p].us_per_sample;
            mode = modes[p];
        }
    }

    if(drv->setMotorSpeed(DEFAULT_MOTOR_SPEED) != SL_RESULT_OK)
    {
        printf("lidar set motor speed failed\n");        
    }

    if(drv->startScanExpress(0, mode.id, 0, &mode) != SL_RESULT_OK)
    {
        logger.write("[LIDAR] start scan failed", true);
        return;
    }
    printf("lidar scan start, %s\n", mode.scan_mode);
    logger.write("lidar scan start", true);

    MOBILE_POSE pre_mobile_pose = mobile->get_pose();
    while(grabFlag)
    {
        sl_lidar_response_measurement_node_hq_t nodes[8192];
        size_t count = _countof(nodes);
        if(drv->grabScanDataHq(nodes, count) == SL_RESULT_OK)
        {
            MOBILE_POSE mobile_pose = mobile->get_pose();
            drv->ascendScanData(nodes, count);

            // waiting new pose
            int wait_cnt = 0;
            double lidar_t = get_time();
            double mobile_t = mobile_pose.t;
            while(lidar_t > mobile_t)
            {
                mobile_pose = mobile->get_pose();
                mobile_t = mobile_pose.t;

                wait_cnt++;
                if(wait_cnt > 500)
                {
                    logger.write("[LIDAR] mobile pose time drift", true);
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }

            // find best mobile pose
            std::vector<MOBILE_POSE> mobile_poses = mobile->get_poses();
            if(mobile_poses.size() >= 10)
            {
                MOBILE_POSE mobile_pose0;
                for(size_t p = 0; p < mobile_poses.size(); p++)
                {
                    if(mobile_poses[p].t < lidar_t)
                    {
                        mobile_pose0 = mobile_poses[p];
                    }
                }

                double t = lidar_t;
                double t0 = mobile_pose0.t;
                double t1 = mobile_poses.back().t;
                if(t1 <= t0)
                {
                    t1 = t0+0.01;
                }

                double a = (t-t1)/(t0-t1);
                double b = (t-t0)/(t1-t0);

                cv::Vec3d intp_pose = intpXi(a, b, mobile_pose0.pose, mobile_poses.back().pose);
                mobile_pose.t = lidar_t;
                mobile_pose.pose = intp_pose;
            }

            const double t0 = 0;
            const double t1 = 360;

            std::vector<double> ts;
            std::vector<cv::Vec2d> pts;
            for(size_t p = 0; p < count; p++)
            {
                if(nodes[p].dist_mm_q2 == 0)
                {
                    continue;
                }

                double deg =(nodes[p].angle_z_q14 * 90.0)/16384.0;
                double t = deg;
                double th = deg*D2R;
                double dist = (nodes[p].dist_mm_q2/4.0)/1000.0;
                if(dist > robot_config.robot_lidar_max_range)
                {
                    continue;
                }

                /*
                const double dist_th = 0.4;
                if(dist < dist_th && deg > 40.0 && deg < 50.0)
                {
                    continue;
                }

                if(dist < dist_th && deg > 130.0 && deg < 140.0)
                {
                    continue;
                }

                if(dist < dist_th && deg > 220.0 && deg < 230.0)
                {
                    continue;
                }

                if(dist < dist_th && deg > 310.0 && deg < 320.0)
                {
                    continue;
                }
                */

                // coordinates flip
                double x = std::cos(th)*dist + robot_config.robot_lidar_offset_x;                
                double y = std::sin(th)*dist + robot_config.robot_lidar_offset_y;                
                if(!isfinite(x) || !isfinite(y))
                {
                    continue;
                }

                double d = std::sqrt(x*x+y*y);
                if(d <= robot_config.robot_radius + 0.03)
                {
                    continue;
                }

                //double y = -1 * std::sin(th)*dist + robot_config.robot_lidar_offset_y;

                ts.push_back(t);
                pts.push_back(cv::Vec2d(x,y));
            }

            LIDAR_FRM frm;
            frm.t = lidar_t;
            frm.t0 = t0;
            frm.t1 = t1;
            frm.ts = ts;
            frm.pts = pts;
            frm.mobile_pose = mobile_pose.pose;
            frm.pre_mobile_pose = pre_mobile_pose.pose;
            raw_que.push(frm);

            pre_mobile_pose = mobile_pose;
            if(raw_que.unsafe_size() > 50)
            {
                LIDAR_FRM temp;
                raw_que.try_pop(temp);
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    drv->stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    drv->setMotorSpeed(0);

    delete drv;
    drv = NULL;
}


void LIDAR_2D::aLoop()
{
    while(aFlag)
    {
        LIDAR_FRM frm;
        if(raw_que.try_pop(frm))
        {
            const double t0 = frm.t0;
            const double t1 = frm.t1;

            cv::Vec3d pre_mobile_pose = frm.pre_mobile_pose;
            cv::Vec3d mobile_pose = frm.mobile_pose;

            LIDAR_FRM _frm;
            _frm.t = frm.t;
            _frm.t0 = frm.t0;
            _frm.t1 = frm.t1;
            _frm.ts = frm.ts;
            _frm.mobile_pose = frm.mobile_pose;
            _frm.pre_mobile_pose = frm.pre_mobile_pose;

            for(size_t p = 0; p < frm.pts.size(); p++)
            {
                // motion distortion compensation using odometry
                double t = frm.ts[p];

                cv::Vec2d P;
                P[0] = frm.pts[p][0];
                P[1] = frm.pts[p][1];

                double a = (t-t1)/(t0-t1);
                double b = (t-t0)/(t1-t0);

                cv::Vec3d _xi = intpXi(a, b, pre_mobile_pose, mobile_pose);
                cv::Vec3d xi = divXi(_xi, mobile_pose);

                cv::Matx22d R;
                R(0, 0) = std::cos(xi[2]);
                R(0, 1) = -std::sin(xi[2]);
                R(1, 0) = std::sin(xi[2]);
                R(1, 1) = std::cos(xi[2]);

                cv::Vec2d T;
                T[0] = xi[0];
                T[1] = xi[1];

                cv::Vec2d _P = R * P + T;

                // storing
                _frm.ts.push_back(t);
                _frm.pts.push_back(_P);
            }

            scan_que.push(_frm);

            mtx.lock();
            cur_scan = _frm.pts;
            mtx.unlock();

            // for memory
            if(scan_que.unsafe_size() > 50)
            {
                LIDAR_FRM temp;
                scan_que.try_pop(temp);
            }

            continue;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

std::vector<cv::Vec2d> LIDAR_2D::get_cur_scan()
{
    mtx.lock();
    std::vector<cv::Vec2d> _cur_scan = cur_scan;
    mtx.unlock();

    return _cur_scan;
}

cv::Vec3d LIDAR_2D::intpXi(double a, double b, cv::Vec3d xi0, cv::Vec3d xi1)
{
    cv::Vec3d res;
    res[0] = a*xi0[0] + b*xi1[0];
    res[1] = a*xi0[1] + b*xi1[1];

    cv::Vec2d r_vec0;
    r_vec0[0] = std::cos(xi0[2]);
    r_vec0[1] = std::sin(xi0[2]);

    cv::Vec2d r_vec1;
    r_vec1[0] = std::cos(xi1[2]);
    r_vec1[1] = std::sin(xi1[2]);

    cv::Vec2d r_vec = a*r_vec0 + b*r_vec1;
    res[2] = std::atan2(r_vec[1], r_vec[0]);
    return res;
}

cv::Vec3d LIDAR_2D::divXi(cv::Vec3d xi0, cv::Vec3d xi1)
{
    cv::Matx22d R1_inv;
    R1_inv(0, 0) = std::cos(xi1[2]);
    R1_inv(0, 1) = std::sin(xi1[2]);
    R1_inv(1, 0) = -std::sin(xi1[2]);
    R1_inv(1, 1) = std::cos(xi1[2]);

    cv::Vec2d t0;
    t0[0] = xi0[0];
    t0[1] = xi0[1];

    cv::Vec2d t1;
    t1[0] = xi1[0];
    t1[1] = xi1[1];

    cv::Vec2d _t = R1_inv * (t0-t1);

    cv::Vec3d xi;
    xi[0] = _t[0];
    xi[1] = _t[1];
    xi[2] = toWrap(xi0[2] - xi1[2]);
    return xi;
}

cv::Vec3d LIDAR_2D::mulXi(cv::Vec3d xi0, cv::Vec3d xi1)
{
    cv::Matx22d R0;
    R0(0, 0) = std::cos(xi0[2]);
    R0(0, 1) = -std::sin(xi0[2]);
    R0(1, 0) = std::sin(xi0[2]);
    R0(1, 1) = std::cos(xi0[2]);

    cv::Vec2d t1;
    t1[0] = xi1[0];
    t1[1] = xi1[1];

    cv::Vec2d _t1 = R0 * t1;

    cv::Vec3d xi;
    xi[0] = xi0[0] + _t1[0];
    xi[1] = xi0[1] + _t1[1];
    xi[2] = toWrap(xi0[2] + xi1[2]);
    return xi;
}
