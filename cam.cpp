#include "cam.h"

CAM::CAM(QObject *parent) : QObject(parent)
{
}

CAM::~CAM()
{
    if(grabThread != NULL)
    {
        grabFlag = false;
        grabThread->join();
        grabThread = NULL;
    }
}

void CAM::init()
{
    // start grab loop
    if (grabThread == NULL)
    {
        grabFlag = true;
        grabThread = new std::thread(&CAM::grabLoop, this);
    }
}

void CAM::grabLoop()
{
    try
    {
        rs2::context ctx;
        rs2::device_list devices = ctx.query_devices();
        if(devices.size() != 2)
        {
            logger.write("[CAM] need two cam, failed", true);
            return;
        }

        printf("cam found %d\n", devices.size());
        for(size_t p = 0; p < devices.size(); p++)
        {
            std::cout << p << ":" << devices[p].get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << std::endl;
        }

        bool is_sn_ok = false;
        if((robot_config.cam_left_sn == devices[0].get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) || robot_config.cam_left_sn == devices[1].get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)) &&
           (robot_config.cam_right_sn == devices[0].get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) || robot_config.cam_right_sn == devices[1].get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)))
        {
            is_sn_ok = true;
        }

        const int w = 480;
        const int h = 270;
        const int fps = 6;

        QStringList l_tf = robot_config.cam_left_tf.split(",");
        if(l_tf.size() != 6)
        {
            printf("cam left tf invalid\n");
            return;
        }

        QStringList r_tf = robot_config.cam_right_tf.split(",");
        if(r_tf.size() != 6)
        {
            printf("cam right tf invalid\n");
            return;
        }

        // transformations
        Eigen::Matrix4d T_l = zyx_tranformation(l_tf[0].toDouble(), l_tf[1].toDouble(), l_tf[2].toDouble(), l_tf[3].toDouble(), l_tf[4].toDouble(), l_tf[5].toDouble());
        Eigen::Matrix4d T_r = zyx_tranformation(r_tf[0].toDouble(), r_tf[1].toDouble(), r_tf[2].toDouble(), r_tf[3].toDouble(), r_tf[4].toDouble(), r_tf[5].toDouble());

        // init left cam
        rs2::pipeline pipe_l = rs2::pipeline(ctx);
        rs2::config cfg_l;
        if(is_sn_ok)
        {
            cfg_l.enable_device(robot_config.cam_left_sn.toStdString());
            sn_l = robot_config.cam_left_sn;
        }
        else
        {
            cfg_l.enable_device(devices[0].get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
            sn_l = devices[0].get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        }
        cfg_l.disable_all_streams();
        cfg_l.enable_stream(RS2_STREAM_DEPTH, w, h, RS2_FORMAT_Z16, fps);
        cfg_l.enable_stream(RS2_STREAM_INFRARED, 1, w, h, RS2_FORMAT_Y8, fps);

        auto sensor_l = cfg_l.resolve(pipe_l).get_device().first<rs2::depth_sensor>();
        if(sensor_l.supports(RS2_OPTION_EMITTER_ENABLED))
        {
            auto range = sensor_l.get_option_range(RS2_OPTION_LASER_POWER);
            sensor_l.set_option(RS2_OPTION_EMITTER_ENABLED, 1.0f); // enable emitter
            sensor_l.set_option(RS2_OPTION_LASER_POWER, range.max); // power max
        }

        if (sensor_l.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE))
        {
            sensor_l.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0.0f);
            sensor_l.set_option(RS2_OPTION_EXPOSURE, robot_config.cam_exposure);
        }

        sensor_l.set_option(rs2_option::RS2_OPTION_VISUAL_PRESET, rs2_rs400_visual_preset::RS2_RS400_VISUAL_PRESET_HIGH_ACCURACY);

        // init right cam
        rs2::pipeline pipe_r = rs2::pipeline(ctx);
        rs2::config cfg_r;
        if(is_sn_ok)
        {
            cfg_r.enable_device(robot_config.cam_right_sn.toStdString());
            sn_r = robot_config.cam_right_sn;
        }
        else
        {
            cfg_r.enable_device(devices[1].get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
            sn_r = devices[1].get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        }
        cfg_r.disable_all_streams();
        cfg_r.enable_stream(RS2_STREAM_DEPTH, w, h, RS2_FORMAT_Z16, fps);
        cfg_r.enable_stream(RS2_STREAM_INFRARED, 1, w, h, RS2_FORMAT_Y8, fps);

        auto sensor_r = cfg_r.resolve(pipe_r).get_device().first<rs2::depth_sensor>();
        if(sensor_r.supports(RS2_OPTION_EMITTER_ENABLED))
        {
            auto range = sensor_r.get_option_range(RS2_OPTION_LASER_POWER);
            sensor_r.set_option(RS2_OPTION_EMITTER_ENABLED, 1.0f); // enable emitter
            sensor_r.set_option(RS2_OPTION_LASER_POWER, range.max); // power max
        }

        if (sensor_r.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE))
        {
            sensor_r.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0.0f);
            sensor_r.set_option(RS2_OPTION_EXPOSURE, robot_config.cam_exposure);
        }

        sensor_r.set_option(rs2_option::RS2_OPTION_VISUAL_PRESET, rs2_rs400_visual_preset::RS2_RS400_VISUAL_PRESET_HIGH_ACCURACY);

        // callbacks
        const double fov_clip = 0.6;
        auto callback_l = [&](const rs2::frame& frame)
        {
            if (rs2::frameset fs = frame.as<rs2::frameset>())
            {
                double time = get_time();
                auto depth = fs.get_depth_frame();
                auto infra = fs.get_infrared_frame(1);

                // filtering
                rs2::decimation_filter dec_filter;
                dec_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2);
                depth = dec_filter.process(depth);

                rs2::disparity_transform depth_to_disparity(true);
                depth = depth_to_disparity.process(depth);

                rs2::spatial_filter spat_filter;
                spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.55f);
                depth = spat_filter.process(depth);

                rs2::temporal_filter temp_filter;
                temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.4f);
                temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 20.0f);
                temp_filter.set_option(RS2_OPTION_HOLES_FILL, 3);
                depth = temp_filter.process(depth);

                rs2::disparity_transform disparity_to_depth(false);
                depth = disparity_to_depth.process(depth);

                // parsing point cloud
                rs2::pointcloud pc;
                pc.map_to(depth);

                rs2::points points;
                points = pc.calculate(depth);

                if(points.size() < 10)
                {
                    printf("left cam no points\n");
                    return;
                }

                std::vector<cv::Vec3d> scan;
                auto vertices = points.get_vertices();
                auto tex_coords = points.get_texture_coordinates();
                int _w = depth.get_width(); int _h = depth.get_height();
                //cv::Mat img = cv::Mat(cv::Size(_w,_h), CV_8UC1, cv::Scalar::all(0));
                for(size_t p = 0; p < points.size(); p++)
                {
                    // fov clipping
                    int u = tex_coords[p].u*_w;
                    int v = tex_coords[p].v*_h;

                    if(u < 0 || u >= _w || v < 0 || v >= _h)
                    {
                        continue;
                    }

                    /*double val = 255*(1- vertices[p].z / 3.0);
                    if(vertices[p].z < 0.1)
                    {
                        val = 0;
                    }
                    img.ptr<uchar>(v)[u] = int(val);*/

                    if(v < _h*fov_clip)
                    {
                        continue;
                    }

                    Eigen::Vector3d P(vertices[p].x, vertices[p].y, vertices[p].z);
                    if(P[2] < 0.1 || P[2] > 2.0)
                    {
                        continue;
                    }

                    Eigen::Vector3d _P = T_l.block(0,0,3,3)*P+T_l.block(0,3,3,1);
                    if(_P[2] > 0.1 && _P[2] < 2.0)
                    {
                        scan.push_back(cv::Vec3d(_P[0], _P[1], _P[2]));
                    }
                }
                //cv::flip(img, img, 0);

                // sampling
                int sample_w = (1.5 / robot_config.robot_grid_size)*2;
                int sample_c = sample_w/2;

                cv::Mat sampling_mask(sample_w, sample_w, CV_32S, cv::Scalar(0));
                for(size_t p = 0; p < scan.size(); p++)
                {
                    int u = scan[p][0]/robot_config.robot_grid_size + sample_c;
                    int v = scan[p][1]/robot_config.robot_grid_size + sample_c;
                    if(u < 0 || u >= sample_w || v < 0 || v >= sample_w)
                    {
                        continue;
                    }
                    sampling_mask.ptr<int>(v)[u] = p;
                }

                std::vector<cv::Vec2d> sampled_scan;
                std::vector<cv::Vec3d> sampled_scan_3d;
                for(int i = 0; i < sample_w; i++)
                {
                    for(int j = 0; j < sample_w; j++)
                    {
                        int idx = sampling_mask.ptr<int>(i)[j];
                        if(idx != 0)
                        {
                            sampled_scan.push_back(cv::Vec2d(scan[idx][0], scan[idx][1]));
                            sampled_scan_3d.push_back(scan[idx]);
                        }
                    }
                }

                int ir_w = infra.get_width();
                int ir_h = infra.get_height();
                cv::Mat img_ir(cv::Size(ir_w, ir_h), CV_8U, (void*)infra.get_data(), cv::Mat::AUTO_STEP);
                cv::flip(img_ir, img_ir, 0);

                // update
                mtx.lock();
                time_l = time;
                cur_scan_l = sampled_scan;
                cur_scan_3d_l = sampled_scan_3d;
                cur_img_l = img_ir.clone();
                mtx.unlock();
            }
        };

        auto callback_r = [&](const rs2::frame& frame)
        {
            if (rs2::frameset fs = frame.as<rs2::frameset>())
            {
                double time = get_time();
                auto depth = fs.get_depth_frame();
                auto infra = fs.get_infrared_frame(1);

                // filtering
                rs2::decimation_filter dec_filter;
                dec_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2);
                depth = dec_filter.process(depth);

                rs2::disparity_transform depth_to_disparity(true);
                depth = depth_to_disparity.process(depth);

                rs2::spatial_filter spat_filter;
                spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.55f);
                depth = spat_filter.process(depth);

                rs2::temporal_filter temp_filter;
                temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.4f);
                temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 20.0f);
                temp_filter.set_option(RS2_OPTION_HOLES_FILL, 3);
                depth = temp_filter.process(depth);

                rs2::disparity_transform disparity_to_depth(false);
                depth = disparity_to_depth.process(depth);

                rs2::pointcloud pc;
                pc.map_to(depth);

                rs2::points points;
                points = pc.calculate(depth);

                if(points.size() < 10)
                {
                    printf("right cam no points\n");
                    return;
                }

                std::vector<cv::Vec3d> scan;
                auto vertices = points.get_vertices();
                auto tex_coords = points.get_texture_coordinates();
                int _w = depth.get_width(); int _h = depth.get_height();
                //cv::Mat img = cv::Mat(cv::Size(_w,_h), CV_8UC1, cv::Scalar::all(0));
                for(size_t p = 0; p < points.size(); p++)
                {
                    // fov clipping
                    int u = tex_coords[p].u*_w;
                    int v = tex_coords[p].v*_h;

                    if(u < 0 || u >= _w || v < 0 || v >= _h)
                    {
                        continue;
                    }

                    /*double val = 255*(1- vertices[p].z / 3.0);
                    if(vertices[p].z < 0.1)
                    {
                        val = 0;
                    }
                    img.ptr<uchar>(v)[u] = int(val);*/

                    if(v < _h*fov_clip)
                    {
                        continue;
                    }

                    Eigen::Vector3d P(vertices[p].x, vertices[p].y, vertices[p].z);
                    if(P[2] < 0.1 || P[2] > 2.0)
                    {
                        continue;
                    }

                    Eigen::Vector3d _P = T_r.block(0,0,3,3)*P+T_r.block(0,3,3,1);
                    if(_P[2] > 0.1 && _P[2] < 2.0)
                    {
                        scan.push_back(cv::Vec3d(_P[0], _P[1], _P[2]));
                    }
                }
                //cv::flip(img, img, 0);

                // sampling
                int sample_w = (1.5 / robot_config.robot_grid_size)*2;
                int sample_c = sample_w/2;

                cv::Mat sampling_mask(sample_w, sample_w, CV_32S, cv::Scalar(0));
                for(size_t p = 0; p < scan.size(); p++)
                {
                    int u = scan[p][0]/robot_config.robot_grid_size + sample_c;
                    int v = scan[p][1]/robot_config.robot_grid_size + sample_c;
                    if(u < 0 || u >= sample_w || v < 0 || v >= sample_w)
                    {
                        continue;
                    }
                    sampling_mask.ptr<int>(v)[u] = p;
                }

                std::vector<cv::Vec2d> sampled_scan;
                std::vector<cv::Vec3d> sampled_scan_3d;
                for(int i = 0; i < sample_w; i++)
                {
                    for(int j = 0; j < sample_w; j++)
                    {
                        int idx = sampling_mask.ptr<int>(i)[j];
                        if(idx != 0)
                        {
                            sampled_scan.push_back(cv::Vec2d(scan[idx][0], scan[idx][1]));
                            sampled_scan_3d.push_back(scan[idx]);
                        }
                    }
                }

                int ir_w = infra.get_width();
                int ir_h = infra.get_height();
                cv::Mat img_ir(cv::Size(ir_w, ir_h), CV_8U, (void*)infra.get_data(), cv::Mat::AUTO_STEP);
                cv::flip(img_ir, img_ir, 0);

                // update
                mtx.lock();
                time_r = time;
                cur_scan_r = sampled_scan;
                cur_scan_3d_r = sampled_scan_3d;
                cur_img_r = img_ir.clone();
                mtx.unlock();
            }
        };

        // start pipeline
        pipe_l.start(cfg_l, callback_l);
        pipe_r.start(cfg_r, callback_r);

        while(grabFlag)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        // stop pipeline
        pipe_l.stop();
        pipe_r.stop();
    }
    catch(rs2::error &e)
    {
        printf("\033[31m[CAM] realsense some problem\n\033[0m");
        std::cout << e.what() << std::endl;
    }
}

QString CAM::get_sn_l()
{
    mtx.lock();
    QString _sn_l = sn_l;
    mtx.unlock();

    return _sn_l;
}

QString CAM::get_sn_r()
{
    mtx.lock();
    QString _sn_r = sn_r;
    mtx.unlock();

    return _sn_r;
}

cv::Mat CAM::get_cur_img_l()
{
    mtx.lock();
    cv::Mat _cur_img_l = cur_img_l.clone();
    mtx.unlock();

    return _cur_img_l;
}

cv::Mat CAM::get_cur_img_r()
{
    mtx.lock();
    cv::Mat _cur_img_r = cur_img_r.clone();
    mtx.unlock();

    return _cur_img_r;
}

Eigen::Matrix4d CAM::zyx_tranformation(double x, double y, double z, double rx, double ry, double rz)
{
    Eigen::Matrix4d res;
    res.setIdentity();

    Eigen::AngleAxisd Rz(rz*D2R, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd Ry(ry*D2R, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd Rx(rx*D2R, Eigen::Vector3d::UnitX());

    Eigen::Quaternion<double> q = Rz * Ry * Rx;

    res.block(0,0,3,3) = q.matrix();
    res(0,3) = x;
    res(1,3) = y;
    res(2,3) = z;

    return res;
}

