#include "unimap.h"

UNIMAP::UNIMAP(QObject *parent) : QObject(parent)
{
    is_loaded = false;
    raw_map = cv::Mat(map_h, map_w, CV_8U, cv::Scalar::all(0));    
    travel_map = cv::Mat(map_h, map_w, CV_8U, cv::Scalar::all(0));
    cost_map = cv::Mat(map_h, map_w, CV_8U, cv::Scalar::all(0));
    cost_map0 = cv::Mat(map_h, map_w, CV_8U, cv::Scalar::all(0));
    obs_map = cv::Mat(map_h, map_w, CV_8U, cv::Scalar::all(0));
    obs_map0 = cv::Mat(map_h, map_w, CV_8U, cv::Scalar::all(0));
}

void UNIMAP::update_map(SUBMAP* submap)
{
    mtx.lock();
    cv::Mat _map = raw_map.clone();
    mtx.unlock();

    cv::Matx22d R;
    R(0, 0) = std::cos(submap->xi[2]);
    R(0, 1) = -std::sin(submap->xi[2]);
    R(1, 0) = std::sin(submap->xi[2]);
    R(1, 1) = std::cos(submap->xi[2]);

    cv::Vec2d submap_center = R * cv::Vec2d(submap->ou, submap->ov);
    cv::Vec2d global_center = xy_uvd(cv::Vec2d(submap->xi[0], submap->xi[1]));

    cv::Matx23d T;
    T(0, 0) = std::cos(submap->xi[2]);
    T(0, 1) = -std::sin(submap->xi[2]);
    T(0, 2) = std::round(global_center[0] - submap_center[0]);
    T(1, 0) = std::sin(submap->xi[2]);
    T(1, 1) = std::cos(submap->xi[2]);
    T(1, 2) = std::round(global_center[1] - submap_center[1]);

    cv::Mat src = submap->get_map_gray();
    cv::Mat dst;
    cv::warpAffine(src, dst, T, raw_map.size(), cv::InterpolationFlags::INTER_NEAREST);

    int n = map_w*map_h;
    #pragma omp parallel for num_threads(2)
    for(int p = 0; p < n; p++)
    {
        int i = p/map_w;
        int j = p%map_w;

        uchar val = dst.ptr<uchar>(i)[j];
        if(val == 0)
        {
            continue;
        }

        uchar pre_val = _map.ptr<uchar>(i)[j];
        if (pre_val == 0)
        {
            _map.ptr<uchar>(i)[j] = val;
            continue;
        }

        uchar pre_diff = std::abs(pre_val - 128);
        uchar cur_diff = std::abs(val - 128);
        if (cur_diff > pre_diff)
        {
            _map.ptr<uchar>(i)[j] = val;
        }
    }

    mtx.lock();
    raw_map = _map.clone();
    mtx.unlock();
}

void UNIMAP::update_map(cv::Mat &_map, SUBMAP* submap)
{
    cv::Matx22d R;
    R(0, 0) = std::cos(submap->xi[2]);
    R(0, 1) = -std::sin(submap->xi[2]);
    R(1, 0) = std::sin(submap->xi[2]);
    R(1, 1) = std::cos(submap->xi[2]);

    cv::Vec2d submap_center = R * cv::Vec2d(submap->ou, submap->ov);
    cv::Vec2d global_center = xy_uvd(cv::Vec2d(submap->xi[0], submap->xi[1]));

    cv::Matx23d T;
    T(0, 0) = std::cos(submap->xi[2]);
    T(0, 1) = -std::sin(submap->xi[2]);
    T(0, 2) = std::round(global_center[0] - submap_center[0]);
    T(1, 0) = std::sin(submap->xi[2]);
    T(1, 1) = std::cos(submap->xi[2]);
    T(1, 2) = std::round(global_center[1] - submap_center[1]);

    cv::Mat src = submap->get_map_gray();
    cv::Mat dst;
    cv::warpAffine(src, dst, T, raw_map.size(), cv::InterpolationFlags::INTER_NEAREST);

    int n = map_w*map_h;
    #pragma omp parallel for num_threads(2)
    for(int p = 0; p < n; p++)
    {
        int i = p/map_w;
        int j = p%map_w;

        uchar val = dst.ptr<uchar>(i)[j];
        if(val == 0)
        {
            continue;
        }

        uchar pre_val = _map.ptr<uchar>(i)[j];
        if (pre_val == 0)
        {
            _map.ptr<uchar>(i)[j] = val;
            continue;
        }

        uchar pre_diff = std::abs(pre_val - 128);
        uchar cur_diff = std::abs(val - 128);
        if (cur_diff > pre_diff)
        {
            _map.ptr<uchar>(i)[j] = val;
        }
    }
}

void UNIMAP::update_obs_map0(std::vector<cv::Vec3d> &pts)
{
    // get current obs map
    mtx.lock();
    cv::Mat _obs_map0 = obs_map0.clone();
    mtx.unlock();

    // make height map
    for(size_t p = 0; p < pts.size(); p++)
    {
        cv::Vec2i uv = xy_uv(cv::Vec2d(pts[p][0], pts[p][1]));
        int u = uv[0];
        int v = uv[1];
        if(u < 0 || u >= _obs_map0.cols || v < 0 || v >= _obs_map0.rows)
        {
            continue;
        }

        int pre_d = _obs_map0.ptr<uchar>(v)[u];
        int cur_d = pts[p][2]*255;
        if(cur_d > 255)
        {
            cur_d = 255;
        }

        if(cur_d > pre_d)
        {
            _obs_map0.ptr<uchar>(v)[u] = cur_d;
        }
    }

    // update obs map
    mtx.lock();
    obs_map0 = _obs_map0.clone();
    mtx.unlock();
}

void UNIMAP::update_obs_map(std::vector<cv::Vec2d> &pts)
{
    // get current obs map
    mtx.lock();
    cv::Mat _raw_map = raw_map.clone();
    cv::Mat _obs_map = obs_map.clone();
    mtx.unlock();

    // count down
    for(int i = 0; i < _obs_map.rows; i++)
    {
        for(int j = 0; j < _obs_map.cols; j++)
        {
            int cnt = _obs_map.ptr<uchar>(i)[j];
            cnt--;
            if(cnt < 0)
            {
                cnt = 0;
            }
            _obs_map.ptr<uchar>(i)[j] = cnt;
        }
    }

    // count up
    for(size_t p = 0; p < pts.size(); p++)
    {
        cv::Vec2i uv = xy_uv(pts[p]);
        int u = uv[0];
        int v = uv[1];
        if(u < 0 || u >= _obs_map.cols || v < 0 || v >= _obs_map.rows)
        {
            continue;
        }

        int cnt = _obs_map.ptr<uchar>(v)[u];
        cnt++;
        if(cnt > 10)
        {
            cnt = 10;
        }
        _obs_map.ptr<uchar>(v)[u] = cnt;
    }

    // clear static obstacle area
    cv::dilate(_raw_map, _raw_map, cv::Mat());
    for(int i = 0; i < _obs_map.rows; i++)
    {
        for(int j = 0; j < _obs_map.cols; j++)
        {
            // check
            if(_obs_map.ptr<uchar>(i)[j] > 0)
            {
                std::vector<cv::Vec2i> pts = filled_circle_iterator(cv::Vec2i(j,i), 2);

                bool is_static_obs = false;
                for(size_t k = 0; k < pts.size(); k++)
                {
                    int u = pts[k][0];
                    int v = pts[k][1];
                    if(u < 0 || u >= _raw_map.cols || v < 0 || v >= _raw_map.rows)
                    {
                        continue;
                    }

                    if(_raw_map.ptr<uchar>(v)[u] == 255)
                    {
                        is_static_obs = true;
                        break;
                    }
                }

                if(is_static_obs)
                {
                    _obs_map.ptr<uchar>(i)[j] = 0;
                }
            }
        }
    }

    // clear single point
    for(int i = 2; i < _obs_map.rows-2; i++)
    {
        for(int j = 2; j < _obs_map.cols-2; j++)
        {
            if(_obs_map.ptr<uchar>(i)[j] != 0)
            {
                int cnt = 0;
                for(int my = -2; my <= 2; my++)
                {
                    for(int mx = -2; mx <= 2; mx++)
                    {
                        int ii = i+my;
                        int jj = j+mx;
                        if(_obs_map.ptr<uchar>(ii)[jj] != 0)
                        {
                            cnt++;
                        }
                    }
                }

                if(cnt <= 1)
                {
                    _obs_map.ptr<uchar>(i)[j] = 0;
                }
            }
        }
    }

    // update obs map
    mtx.lock();
    obs_map = _obs_map.clone();
    mtx.unlock();
}

void UNIMAP::set_map(cv::Mat &_map)
{
    mtx.lock();
    raw_map = _map.clone();
    mtx.unlock();
}

void UNIMAP::clear_map()
{
    is_loaded = false;

    mtx.lock();

    // clear map
    raw_map = cv::Mat(map_h, map_w, CV_8U, cv::Scalar::all(0));
    travel_map = cv::Mat(map_h, map_w, CV_8U, cv::Scalar::all(0));
    cost_map = cv::Mat(map_h, map_w, CV_8U, cv::Scalar::all(0));
    cost_map0 = cv::Mat(map_h, map_w, CV_8U, cv::Scalar::all(0));
    obs_map = cv::Mat(map_h, map_w, CV_8U, cv::Scalar::all(0));
    obs_map0 = cv::Mat(map_h, map_w, CV_8U, cv::Scalar::all(0));

    // clear annotations
    serving_loc_list.clear();
    patrol_loc_list.clear();
    charging_loc_list.clear();
    resting_loc_list.clear();

    serving_loc_name_list.clear();
    patrol_loc_name_list.clear();
    charging_loc_name_list.clear();
    resting_loc_name_list.clear();

    mtx.unlock();
}

void UNIMAP::load_map(QString path)
{
    mtx.lock();

    double st_time = get_time();

    // load map
    QString raw_map_path = path + "/map_raw.png";
    QFileInfo raw_map_info(raw_map_path);
    if(raw_map_info.exists() && raw_map_info.isFile())
    {
        raw_map = cv::imread(raw_map_path.toStdString(), cv::IMREAD_GRAYSCALE);
        //logger.write("[UNIMAP] map_raw.png loaded", true);
    }
    else
    {
        //logger.write("[UNIMAP] map_raw.png not found", true);
    }

    QString edited_map_path = path + "/map_edited.png";
    QFileInfo edited_map_info(edited_map_path);
    if(edited_map_info.exists() && edited_map_info.isFile())
    {
        raw_map = cv::imread(edited_map_path.toStdString(), cv::IMREAD_GRAYSCALE);        
        //logger.write("[UNIMAP] map_edited.png loaded", true);
    }
    else
    {
        //logger.write("[UNIMAP] map_edited.png not found", true);
    }

    // load map meta
    QString map_meta_path = path + "/map_meta.ini";
    QFileInfo map_meta_info(map_meta_path);
    if(map_meta_info.exists() && map_meta_info.isFile())
    {
        // load meta data file
        QSettings settings(map_meta_path, QSettings::IniFormat);
        map_w = settings.value("map_metadata/map_w", raw_map.cols).toInt();
        map_h = settings.value("map_metadata/map_h", raw_map.rows).toInt();
        map_ou = settings.value("map_metadata/map_origin_u", raw_map.cols/2).toInt();
        map_ov = settings.value("map_metadata/map_origin_v", raw_map.rows/2).toInt();
        map_grid_width = settings.value("map_metadata/map_grid_width", robot_config.robot_grid_size).toDouble();

        //logger.write("[UNIMAP] map_meta.ini loaded", true);
    }
    else
    {
        //logger.write("[UNIMAP] map_meta.ini not found", true);
    }

    // raw_map initial position clear
    // cv::circle(raw_map, cv::Point(map_ou, map_ov), (robot_config.robot_radius/robot_config.robot_grid_size)*1.5, cv::Scalar(127), -1);

    // set measured map points
    map_pts.clear();
    for(int i = 0; i < raw_map.rows; i++)
    {
        for(int j = 0; j < raw_map.cols; j++)
        {
            if(raw_map.ptr<uchar>(i)[j] == 255)
            {
                map_pts.push_back(uv_xy(cv::Vec2i(j,i)));
            }
        }
    }

    // load annotations
    serving_loc_list.clear();
    patrol_loc_list.clear();
    charging_loc_list.clear();
    resting_loc_list.clear();

    serving_loc_name_list.clear();
    patrol_loc_name_list.clear();
    charging_loc_name_list.clear();
    resting_loc_name_list.clear();

    annotated_loc_path = path + "/annotation.ini";
    QFileInfo annotated_loc_info(annotated_loc_path);
    if(annotated_loc_info.exists() && annotated_loc_info.isFile())
    {
        QSettings settings(annotated_loc_path, QSettings::IniFormat);
        int serving_num = settings.value("serving_locations/num").toInt();
        for(int p = 0; p < serving_num; p++)
        {
            QString sec;
            sec.sprintf("serving_locations/loc%d", p);

            QString str = settings.value(sec).toString();
            QStringList str_list = str.split(",");

            QString name = str_list[0];
            double x = str_list[1].toDouble();
            double y = str_list[2].toDouble();
            double th = str_list[3].toDouble();

            serving_loc_list.push_back(cv::Vec3d(x,y,th));
            serving_loc_name_list.push_back(name);
        }

        int patrol_num = settings.value("patrol_locations/num").toInt();
        for(int p = 0; p < patrol_num; p++)
        {
            QString sec;
            sec.sprintf("patrol_locations/loc%d", p);

            QString str = settings.value(sec).toString();
            QStringList str_list = str.split(",");

            QString name = str_list[0];
            double x = str_list[1].toDouble();
            double y = str_list[2].toDouble();
            double th = str_list[3].toDouble();

            patrol_loc_list.push_back(cv::Vec3d(x,y,th));
            patrol_loc_name_list.push_back(name);
        }

        int charging_num = settings.value("charging_locations/num").toInt();
        for(int p = 0; p < charging_num; p++)
        {
            QString sec;
            sec.sprintf("charging_locations/loc%d", p);

            QString str = settings.value(sec).toString();
            QStringList str_list = str.split(",");

            QString name = str_list[0];
            double x = str_list[1].toDouble();
            double y = str_list[2].toDouble();
            double th = str_list[3].toDouble();

            charging_loc_list.push_back(cv::Vec3d(x,y,th));
            charging_loc_name_list.push_back(name);
        }

        int resting_num = settings.value("resting_locations/num").toInt();
        for(int p = 0; p < resting_num; p++)
        {
            QString sec;
            sec.sprintf("resting_locations/loc%d", p);

            QString str = settings.value(sec).toString();
            QStringList str_list = str.split(",");

            QString name = str_list[0];
            double x = str_list[1].toDouble();
            double y = str_list[2].toDouble();
            double th = str_list[3].toDouble();

            resting_loc_list.push_back(cv::Vec3d(x,y,th));
            resting_loc_name_list.push_back(name);
        }

        int object_num = settings.value("objects/num").toInt();
        for(int p = 0; p < object_num; p++)
        {
            QString sec;
            sec.sprintf("objects/poly%d", p);

            QString str = settings.value(sec).toString();
            QStringList str_list = str.split(",");
            QString name = str_list[0];
            int vertex_num = str_list.size()-1;

            std::vector<cv::Vec2d> poly;
            for(int i = 1; i < vertex_num; i++)
            {
                QStringList vertex_str_list = str_list[i+1].split(":");
                poly.push_back(cv::Vec2d(vertex_str_list[0].toDouble(), vertex_str_list[1].toDouble()));
            }

            object_poly_list.push_back(poly);
            object_poly_name_list.push_back(name);
        }

        //logger.write("[UNIMAP] annotation.ini loaded", true);
    }
    else
    {
        //logger.write("[UNIMAP] annotation.ini not found", true);
    }

    printf("dt0: %f\n", get_time()-st_time);

    // set virtual obstacle map
    cv::Mat obstacle_map(raw_map.rows, raw_map.cols, CV_8U, cv::Scalar(0));
    for(size_t p = 0; p < object_poly_list.size(); p++)
    {
        std::vector<cv::Vec2d> poly = object_poly_list[p];

        std::vector<cv::Point> _poly;
        for(size_t q = 0; q < poly.size(); q++)
        {
            cv::Vec2i uv = xy_uv(poly[q]);
            _poly.push_back(cv::Point(uv[0], uv[1]));
        }

        std::vector<std::vector<cv::Point>> _poly2;
        _poly2.push_back(_poly);

        cv::fillPoly(obstacle_map, _poly2, cv::Scalar(255));
    }

    // add real obstacle
    for(int i = 0; i < raw_map.rows; i++)
    {
        for(int j = 0; j < raw_map.cols; j++)
        {
            uchar val = raw_map.ptr<uchar>(i)[j];
            if(val == 0 || val == 255)
            {
                obstacle_map.ptr<uchar>(i)[j] = 255;
            }
        }
    }

    // make safe magin map
    int magin = std::ceil(robot_config.robot_radius/robot_config.robot_grid_size);
    cv::Mat safe_magin(raw_map.rows, raw_map.cols, CV_8U, cv::Scalar(0));
    for(int i = 0; i < obstacle_map.rows; i++)
    {
        for(int j = 0; j < obstacle_map.cols; j++)
        {
            if(obstacle_map.ptr<uchar>(i)[j] == 255)
            {
                cv::circle(safe_magin, cv::Point(j,i), magin, cv::Scalar(255), -1);
            }
        }
    }

    //cv::imshow("safe_magin", safe_magin);

    printf("dt1: %f\n", get_time()-st_time);

    // thinning
    cv::Mat thin_src = ~safe_magin;
    cv::Mat thin_src2;

    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(magin*3, magin*3));
    cv::erode(thin_src, thin_src2, element);
    cv::dilate(thin_src2, thin_src2, element);
    cv::dilate(thin_src2, thin_src2, cv::Mat(), cv::Point(-1,-1), 1);
    cv::subtract(thin_src, thin_src2, thin_src);
    cv::dilate(thin_src, thin_src, cv::Mat(), cv::Point(-1,-1), 3);

    cv::Mat thined;
    cv::ximgproc::thinning(thin_src, thined, cv::ximgproc::THINNING_ZHANGSUEN);

    printf("dt2: %f\n", get_time()-st_time);

    // smooth junctions
    int round = (robot_config.robot_look_ahead_dist/robot_config.robot_grid_size)*0.5;
    cv::Mat thined2(raw_map.rows, raw_map.cols, CV_8U, cv::Scalar(0));
    for(int i = 0; i < thined.rows; i++)
    {
        for(int j = 0; j < thined.cols; j++)
        {
            if(thined.ptr<uchar>(i)[j] == 255)
            {
                cv::circle(thined2, cv::Point(j,i), round, cv::Scalar(255), -1);
            }
        }
    }

    cv::Mat thined3(raw_map.rows, raw_map.cols, CV_8U, cv::Scalar(255));
    for(int i = 0; i < thined.rows; i++)
    {
        for(int j = 0; j < thined.cols; j++)
        {
            if(thined2.ptr<uchar>(i)[j] == 0)
            {
                cv::circle(thined3, cv::Point(j,i), round, cv::Scalar(0), -1);
            }
        }
    }

    //cv::imshow("thined3", thined3);

    printf("dt3: %f\n", get_time()-st_time);

    // update travel_raw
    QString travel_raw_path = path + "/travel_raw.png";
    cv::imwrite(travel_raw_path.toStdString(), thined3);

    // load travel map
    QString travel_map_path = path + "/travel_edited.png";
    QFileInfo travel_map_info(travel_map_path);
    if(travel_map_info.exists() && travel_map_info.isFile())
    {
        travel_map = cv::imread(travel_map_path.toStdString(), cv::IMREAD_GRAYSCALE);        
        //logger.write("[UNIMAP] travel_edited.png loaded", true);
    }
    else
    {
        travel_map = thined3;
        //logger.write("[UNIMAP] travel_edited.png not found", true);
    }

    // smoothing travel map
    cv::GaussianBlur(travel_map, travel_map, cv::Size(5,5), 1.0, 0);

    // make cost map
    cv::Mat _cost_map(raw_map.rows, raw_map.cols, CV_8U, cv::Scalar::all(255));
    for(int i = 0; i < _cost_map.rows; i++)
    {
        for(int j = 0; j < _cost_map.cols; j++)
        {
            if(safe_magin.ptr<uchar>(i)[j] == 0)
            {
                _cost_map.ptr<uchar>(i)[j] = 127;
            }

            if(safe_magin.ptr<uchar>(i)[j] == 0 && travel_map.ptr<uchar>(i)[j] != 0)
            {
                int cost = _cost_map.ptr<uchar>(i)[j] - travel_map.ptr<uchar>(i)[j];
                if(cost < 0)
                {
                    cost = 0;
                }
                _cost_map.ptr<uchar>(i)[j] = cost;
            }
        }
    }

    // update cost map
    cost_map = _cost_map;

    // update cost map plot
    cv::Mat _cost_map2(raw_map.rows, raw_map.cols, CV_8U, cv::Scalar::all(255));
    for(int i = 0; i < _cost_map.rows; i++)
    {
        for(int j = 0; j < _cost_map.cols; j++)
        {
            if(safe_magin.ptr<uchar>(i)[j] == 0)
            {
                _cost_map2.ptr<uchar>(i)[j] = 127;
            }
        }
    }

    cost_map0 = _cost_map2.clone();
    //cv::imshow("cost_map0", cost_map0);

    cv::Mat cost_map_plot;
    cv::addWeighted(raw_map, 0.5, _cost_map2, 0.5, 0, cost_map_plot);
    QString map_cost_path = path + "/map_cost.png";
    cv::imwrite(map_cost_path.toStdString(), cost_map_plot);

    // flag set
    is_loaded = true;

    QString str;
    str.sprintf("[UNIMAP] map load, spent time: %f", get_time()-st_time);
    logger.write(str, true);

    mtx.unlock();
}

void UNIMAP::edit_location(int type, int idx, cv::Vec3d new_loc)
{
    if(is_loaded == false)
    {
        return;
    }

    mtx.lock();

    if(type == 0)
    {
        // serving
        if(serving_loc_list.size() == 0)
        {
            return;
        }

        // update
        serving_loc_list[idx] = new_loc;

        // save
        QString sec;
        sec.sprintf("serving_locations/loc%d", idx);

        QString name = serving_loc_name_list[idx];

        QString str;
        str.sprintf(",%f,%f,%f", new_loc[0], new_loc[1], new_loc[2]);

        QSettings settings(annotated_loc_path, QSettings::IniFormat);
        settings.setValue(sec, name+str);

    }
    else if(type == 1)
    {
        // patrol
        if(patrol_loc_list.size() == 0)
        {
            return;
        }

        // update
        patrol_loc_list[idx] = new_loc;

        // save
        QString sec;
        sec.sprintf("patrol_locations/loc%d", idx);

        QString name = patrol_loc_name_list[idx];

        QString str;
        str.sprintf(",%f,%f,%f", new_loc[0], new_loc[1], new_loc[2]);

        QSettings settings(annotated_loc_path, QSettings::IniFormat);
        settings.setValue(sec, name+str);

    }
    else if(type == 2)
    {
        // resting
        if(resting_loc_list.size() == 0)
        {
            return;
        }

        // update
        resting_loc_list[idx] = new_loc;

        // save
        QString sec;
        sec.sprintf("resting_locations/loc%d", idx);

        QString name = resting_loc_name_list[idx];

        QString str;
        str.sprintf(",%f,%f,%f", new_loc[0], new_loc[1], new_loc[2]);

        QSettings settings(annotated_loc_path, QSettings::IniFormat);
        settings.setValue(sec, name+str);

    }
    else if(type == 3)
    {
        // charging
        if(charging_loc_list.size() == 0)
        {
            return;
        }

        // update
        charging_loc_list[idx] = new_loc;

        // save
        QString sec;
        sec.sprintf("charging_locations/loc%d", idx);

        QString name = charging_loc_name_list[idx];

        QString str;
        str.sprintf(",%f,%f,%f", new_loc[0], new_loc[1], new_loc[2]);

        QSettings settings(annotated_loc_path, QSettings::IniFormat);
        settings.setValue(sec, name+str);
    }

    mtx.unlock();
}

void UNIMAP::save_map(QString path)
{
    mtx.lock();
    cv::Mat _map = raw_map.clone();
    mtx.unlock();

    // set path
    QString raw_map_path = path + "/map_raw.png";
    QString map_meta_path = path + "/map_meta.ini";

    // raw map file saving
    for(int i = 0; i < _map.rows; i++)
    {
        for(int j = 0; j < _map.cols; j++)
        {
            if(_map.ptr<uchar>(i)[j] >= WALL_THRESHOLD*255)
            {
                _map.ptr<uchar>(i)[j] = 255;
            }
            else if(_map.ptr<uchar>(i)[j] == 0)
            {
                _map.ptr<uchar>(i)[j] = 0;
            }
            else
            {
                _map.ptr<uchar>(i)[j] = 127;
            }
        }
    }

    cv::imwrite(raw_map_path.toStdString(), _map);

    // travel map
    QString travel_map_path = path + "/travel_raw.png";
    cv::Mat travel_map(_map.rows, _map.cols, CV_8U, cv::Scalar(0));
    cv::imwrite(travel_map_path.toStdString(), travel_map);

    // cost map
    QString cost_map_path = path + "/map_cost.png";
    cv::Mat cost_map(_map.rows, _map.cols, CV_8U, cv::Scalar(0));
    cv::imwrite(cost_map_path.toStdString(), cost_map);

    // annotation file saving
    QSettings settings(map_meta_path, QSettings::IniFormat);
    settings.setValue("map_metadata/map_w", map_w);
    settings.setValue("map_metadata/map_h", map_h);
    settings.setValue("map_metadata/map_origin_u", map_ou);
    settings.setValue("map_metadata/map_origin_v", map_ov);
    settings.setValue("map_metadata/map_grid_width", map_grid_width);
}

cv::Mat UNIMAP::get_map_raw()
{
    mtx.lock();
    cv::Mat img = raw_map.clone();
    mtx.unlock();

    return img;
}

cv::Mat UNIMAP::get_map_cost()
{
    mtx.lock();
    cv::Mat img = cost_map.clone();
    mtx.unlock();

    return img;
}

cv::Mat UNIMAP::get_map_cost0()
{
    mtx.lock();
    cv::Mat img = cost_map0.clone();
    mtx.unlock();

    return img;
}

cv::Mat UNIMAP::get_map_plot()
{
    mtx.lock();
    cv::Mat _map = raw_map.clone();
    cv::Mat _cost = cost_map.clone();
    mtx.unlock();

    cv::Mat plot_img;
    cv::addWeighted(_map, 0.5, _cost, 0.5, 0, plot_img);
    cv::cvtColor(plot_img, plot_img, cv::COLOR_GRAY2BGR);
    return plot_img;
}

cv::Mat UNIMAP::get_map_obs()
{
    mtx.lock();
    cv::Mat img = obs_map.clone();
    mtx.unlock();

    return img;
}

cv::Mat UNIMAP::get_map_obs0()
{
    mtx.lock();
    cv::Mat img = obs_map0.clone();
    mtx.unlock();

    return img;
}

std::vector<cv::Vec2d> UNIMAP::get_locations()
{
    std::vector<cv::Vec2d> res;
    mtx.lock();
    for(size_t p = 0; p < serving_loc_list.size(); p++)
    {
        res.push_back(cv::Vec2d(serving_loc_list[p][0], serving_loc_list[p][1]));
    }

    for(size_t p = 0; p < resting_loc_list.size(); p++)
    {
        res.push_back(cv::Vec2d(resting_loc_list[p][0], resting_loc_list[p][1]));
    }

    for(size_t p = 0; p < charging_loc_list.size(); p++)
    {
        res.push_back(cv::Vec2d(charging_loc_list[p][0], charging_loc_list[p][1]));
    }

    for(size_t p = 0; p < patrol_loc_list.size(); p++)
    {
        res.push_back(cv::Vec2d(patrol_loc_list[p][0], patrol_loc_list[p][1]));
    }
    mtx.unlock();

    return res;
}

std::vector<cv::Vec2d> UNIMAP::get_map_pts()
{
    mtx.lock();
    std::vector<cv::Vec2d> _map_pts = map_pts;
    mtx.unlock();

    return _map_pts;
}

bool UNIMAP::is_near_static_obs(cv::Vec2d pt)
{
    cv::Vec2i uv = xy_uv(pt);

    mtx.lock();
    uchar val = cost_map.ptr<uchar>(uv[1])[uv[0]];
    mtx.unlock();

    if(val == 255)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool UNIMAP::is_near_dynamic_obs(cv::Vec2d pt)
{
    cv::Vec2i uv = xy_uv(pt);
    int r = (robot_config.robot_radius+robot_config.robot_obs_magin)/robot_config.robot_grid_size;

    std::vector<cv::Vec2i> circle_pts = filled_circle_iterator(uv, r);

    int cnt = 0;

    mtx.lock();
    for(size_t p = 0; p < circle_pts.size(); p++)
    {
        int u = circle_pts[p][0];
        int v = circle_pts[p][1];
        if(u < 0 || u >= obs_map.cols || v < 0 || v >= obs_map.rows)
        {
            continue;
        }

        int val = obs_map.ptr<uchar>(v)[u];
        if(val >= 3)
        {
            cnt++;
        }
    }
    mtx.unlock();

    if(cnt >= 3)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void UNIMAP::draw_robot(cv::Mat &img, cv::Vec3d xi, cv::Scalar c, int tickness)
{
    cv::Vec2i uv0 = xy_uv(cv::Vec2d(0, 0), xi);
    cv::Vec2i uv1 = xy_uv(cv::Vec2d(robot_config.robot_radius, 0), xi);

    cv::circle(img, cv::Point(uv0[0], uv0[1]), std::ceil(robot_config.robot_radius/robot_config.robot_grid_size), c, tickness);
    cv::line(img, cv::Point(uv0[0], uv0[1]), cv::Point(uv1[0], uv1[1]), c, tickness);
}

void UNIMAP::draw_lidar(cv::Mat &img, cv::Vec3d xi, std::vector<cv::Vec2d> pts, cv::Scalar c)
{
    if(pts.size() > 0)
    {
        // draw current scan
        for (size_t p = 0; p < pts.size(); p++)
        {
            cv::Vec2i uv = xy_uv(pts[p], xi);
            if(uv[0] < 0 || uv[0] >= img.cols || uv[1] < 0 || uv[1] >= img.rows)
            {
                continue;
            }
            img.ptr<cv::Vec3b>(uv[1])[uv[0]] = cv::Vec3b(c[0], c[1], c[2]);
        }
    }
}

void UNIMAP::draw_path(cv::Mat &img, std::vector<PATH_POINT> path, cv::Scalar c)
{
    if(path.size() >= 2)
    {
        for(size_t p = 0; p < path.size()-1; p++)
        {
            cv::Vec2i uv0 = xy_uv(path[p].pt);
            cv::Vec2i uv1 = xy_uv(path[p+1].pt);

            if(path[p].obs == 0)
            {
                double val = path[p].v*path[p].v;
                cv::Scalar _c(0.5*c[0] + 128*val, 0.5*c[1] + 128*val, 0.5*c[2] + 128*val);
                cv::line(img, cv::Point(uv0[0], uv0[1]), cv::Point(uv1[0], uv1[1]), _c, 1);
            }
            else
            {
                cv::line(img, cv::Point(uv0[0], uv0[1]), cv::Point(uv1[0], uv1[1]), cv::Vec3b(0,0,255), 1);
            }
        }
    }
}

void UNIMAP::draw_trajectory(cv::Mat &img, std::vector<cv::Vec6d> traj, cv::Scalar c, int tickness)
{
    if(traj.size() >= 2)
    {
        for(size_t p = 1; p < traj.size(); p++)
        {
            cv::Vec2i pt0 = xy_uv(cv::Vec2d(traj[p-1][0], traj[p-1][1]));
            cv::Vec2i pt1 = xy_uv(cv::Vec2d(traj[p][0], traj[p][1]));
            cv::line(img, cv::Point(pt0[0], pt0[1]), cv::Point(pt1[0], pt1[1]), c, tickness);
        }
    }
}

void UNIMAP::draw_object(cv::Mat &img, std::vector<cv::Vec2d> obj, cv::Scalar c, int tickness)
{
    if(obj.size() > 0)
    {
        std::vector<cv::Point> _poly;
        for(size_t p = 0; p < obj.size(); p++)
        {
            cv::Vec2i uv = xy_uv(obj[p]);
            _poly.push_back(cv::Point(uv[0], uv[1]));
        }

        cv::polylines(img, _poly, true, c, std::abs(tickness));
    }
}

void UNIMAP::draw_point(cv::Mat &img, cv::Vec2d pt)
{
    cv::Vec2i uv = xy_uv(pt);
    cv::circle(img, cv::Point(uv[0], uv[1]), 3, cv::Scalar(255,255,255), 1);
}

void UNIMAP::draw_obs_map0(cv::Mat &img)
{
    // draw obstacle map
    mtx.lock();
    cv::Mat _obs_map0 = obs_map0.clone();
    mtx.unlock();

    for(int i = 0; i < _obs_map0.rows; i++)
    {
        for(int j = 0; j < _obs_map0.cols; j++)
        {
            int cnt = _obs_map0.ptr<uchar>(i)[j];
            if(cnt < 10)
            {
                continue;
            }

            img.ptr<cv::Vec3b>(i)[j] = cv::Vec3b(0, cnt, cnt);
        }
    }
}

void UNIMAP::draw_obs_map(cv::Mat &img)
{
    // draw obstacle map
    mtx.lock();
    cv::Mat _obs_map = obs_map.clone();
    mtx.unlock();

    for(int i = 0; i < _obs_map.rows; i++)
    {
        for(int j = 0; j < _obs_map.cols; j++)
        {
            int cnt = _obs_map.ptr<uchar>(i)[j];
            if(cnt == 0)
            {
                continue;
            }

            img.ptr<cv::Vec3b>(i)[j] = cv::Vec3b(155 + 10*cnt, 155 + 10*cnt, 0);
        }
    }
}

void UNIMAP::draw_waypoints(cv::Mat &img, std::vector<cv::Vec3d> list)
{
    if(list.size() == 1)
    {
        cv::Vec2i uv = xy_uv(cv::Vec2d(list[0][0], list[0][1]));
        cv::circle(img, cv::Point(uv[0], uv[1]), (robot_config.robot_radius/robot_config.robot_grid_size)+1, cv::Scalar(0,255,0), 1);
    }
    else if(list.size() >= 2)
    {
        for(size_t p = 0; p < list.size()-1; p++)
        {
            cv::Vec2i uv0 = xy_uv(cv::Vec2d(list[p][0], list[p][1]));
            cv::Vec2i uv1 = xy_uv(cv::Vec2d(list[p+1][0], list[p+1][1]));
            cv::line(img, cv::Point(uv0[0], uv0[1]), cv::Point(uv1[0], uv1[1]), cv::Scalar(0,255,0), 1);
        }

        cv::Vec2i uv = xy_uv(cv::Vec2d(list.back()[0], list.back()[1]));
        cv::circle(img, cv::Point(uv[0], uv[1]), (robot_config.robot_radius/robot_config.robot_grid_size)+1, cv::Scalar(0,255,0), 1);
    }
}

void UNIMAP::draw_other_robot(cv::Mat &img, ROBOT_SHARED &other)
{
    if(other.path.size() >= 2)
    {
        for(size_t p = 0; p < other.path.size()-1; p++)
        {
            cv::Vec2i uv0 = xy_uv(other.path[p]);
            cv::Vec2i uv1 = xy_uv(other.path[p+1]);
            cv::line(img, cv::Point(uv0[0], uv0[1]), cv::Point(uv1[0], uv1[1]), cv::Scalar(128,128,255), 1);
        }
    }

    cv::Vec2i uv = xy_uv(other.path[0]);
    cv::circle(img, cv::Point(uv[0], uv[1]), (robot_config.robot_radius/robot_config.robot_grid_size), cv::Scalar(128,128,255), 1);
}

cv::Vec2i UNIMAP::xy_uv(cv::Vec2d P, cv::Vec3d xi)
{
    cv::Matx22d R;
    R(0, 0) = std::cos(xi[2]);
    R(0, 1) = -std::sin(xi[2]);
    R(1, 0) = std::sin(xi[2]);
    R(1, 1) = std::cos(xi[2]);

    cv::Vec2d t;
    t[0] = xi[0];
    t[1] = xi[1];

    cv::Vec2d _P = R * P + t;

    int u = std::round(_P[0] / map_grid_width + map_ou);
    int v = std::round(_P[1] / map_grid_width + map_ov);
    return cv::Vec2i(u, v);
}

cv::Vec2i UNIMAP::xy_uv(cv::Vec2d P)
{
    int u = std::round(P[0] / map_grid_width + map_ou);
    int v = std::round(P[1] / map_grid_width + map_ov);
    return cv::Vec2i(u, v);
}

cv::Vec2d UNIMAP::xy_uvd(cv::Vec2d P)
{
    double u = P[0] / map_grid_width + map_ou;
    double v = P[1] / map_grid_width + map_ov;
    return cv::Vec2d(u, v);
}

cv::Vec2d UNIMAP::uv_xy(cv::Vec2i uv)
{
    double x = (uv[0] - map_ou) * map_grid_width;
    double y = (uv[1] - map_ov) * map_grid_width;
    return cv::Vec2d(x, y);
}

cv::Vec2d UNIMAP::uv_xy(cv::Vec2i uv, cv::Vec3d xi)
{
    double x = (uv[0] - map_ou) * map_grid_width;
    double y = (uv[1] - map_ov) * map_grid_width;

    cv::Vec2d P(x, y);

    cv::Matx22d R;
    R(0, 0) = std::cos(xi[2]);
    R(0, 1) = -std::sin(xi[2]);
    R(1, 0) = std::sin(xi[2]);
    R(1, 1) = std::cos(xi[2]);

    cv::Vec2d t;
    t[0] = xi[0];
    t[1] = xi[1];

    cv::Vec2d _P = R * P + t;

    return _P;
}

std::vector<cv::Vec2i> UNIMAP::filled_circle_iterator(cv::Vec2i pt, int r)
{
    std::vector<cv::Vec2i> res;
    for(int i = -r; i <= r; i++)
    {
        for(int j = -r; j <= r; j++)
        {
            double d = i*i + j*j;
            if(d <= r*r)
            {
                res.push_back(cv::Vec2i(i,j) + pt);
            }
        }
    }
    return res;
}
