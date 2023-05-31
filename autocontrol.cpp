#include "autocontrol.h"

AUTOCONTROL::AUTOCONTROL(QObject *parent) : QObject(parent)
{
    whole_gain = robot_config.robot_vel_gain;

    ifsm_state = STATE_GOAL_REACHED;
    ofsm_state = STATE_IDLE;    
    is_pause = false;

    robots.resize(10);
}

AUTOCONTROL::~AUTOCONTROL()
{
    if(ofsmThread != NULL)
    {
        ofsmFlag = false;
        ofsmThread->join();
        ofsmThread = NULL;
    }

    if(ifsmThread != NULL)
    {
        ifsmFlag = false;
        ifsmThread->join();
        ifsmThread = NULL;
    }
}

void AUTOCONTROL::init(MOBILE *_mobile, SLAM_2D *_slam, UNIMAP *_unimap)
{
    mobile = _mobile;
    slam = _slam;
    unimap = _unimap;
    whole_gain = robot_config.robot_vel_gain;

    if (ofsmThread == NULL)
    {
        ofsmFlag = true;
        ofsmThread = new std::thread(&AUTOCONTROL::ofsmLoop, this);
    }
}

void AUTOCONTROL::shared_path_callback(const lcm::ReceiveBuffer *rbuf, const std::string &chan, const shared_path *msg)
{
    if(robot_config.robot_use_multi)
    {
        double t = get_time();

        std::vector<cv::Vec2d> path;
        for(int p = 0; p < msg->num; p++)
        {
            double x = msg->path[p][0];
            double y = msg->path[p][1];
            path.push_back(cv::Vec2d(x, y));
        }

        ROBOT_SHARED robot;
        robot.id = msg->id;
        robot.t = t;
        robot.path = path;

        mtx.lock();

        // storing
        robots[robot.id] = robot;

        // clear no update robot
        for(size_t p = 0; p < robots.size(); p++)
        {
            if(robots[p].id != -1 && t - robots[p].t > 1.0)
            {
                robots[p] = ROBOT_SHARED();
            }
        }

        mtx.unlock();
    }
}

std::vector<PATH_POINT> AUTOCONTROL::get_cur_path()
{
    mtx.lock();
    std::vector<PATH_POINT> path = cur_path;
    mtx.unlock();

    return path;
}

std::vector<cv::Vec3d> AUTOCONTROL::get_goal_list()
{
    mtx.lock();
    std::vector<cv::Vec3d> _goal_list = goal_list;
    mtx.unlock();

    return _goal_list;
}

std::vector<ROBOT_SHARED> AUTOCONTROL::get_robots()
{
    mtx.lock();
    std::vector<ROBOT_SHARED> _robots = robots;
    mtx.unlock();

    return _robots;
}

void AUTOCONTROL::add_waypoint(cv::Vec3d wp)
{
    mtx.lock();
    cv::Vec3d goal = goal_list.back();
    goal_list.clear();
    goal_list.push_back(wp);
    goal_list.push_back(goal);
    mtx.unlock();
}

void AUTOCONTROL::ofsm_run(cv::Vec3d goal)
{
    mtx.lock();
    goal_list.clear();
    std::cout<<"goal : "<<goal<<std::endl;
    goal_list.push_back(goal);
    mtx.unlock();
}

void AUTOCONTROL::ifsm_run(cv::Vec3d goal, bool is_avoid = false, bool is_align = true)
{
    if(ifsmThread != NULL)
    {
        ifsmFlag = false;
        ifsmThread->join();
        ifsmThread = NULL;
    }

    if (ifsmThread == NULL)
    {
        // inner fsm start
        printf("[IFSM] control loop start\n");
        ifsmFlag = true;
        ifsmThread = new std::thread(&AUTOCONTROL::ifsmLoop, this, goal, is_avoid, is_align);
    }
}

void AUTOCONTROL::ifsm_stop()
{
    if(ifsmThread != NULL)
    {
        ifsmFlag = false;
        ifsmThread->join();
        ifsmThread = NULL;
    }

    is_pause = false;
}

double AUTOCONTROL::sgn(double val)
{
    if(val < 0)
    {
        return -1;
    }
    else if(val == 0)
    {
        return 0;
    }
    else
    {
        return 1;
    }
}

double AUTOCONTROL::saturation(double val, double min, double max)
{
    double _min = std::min<double>(min, max);
    double _max = std::max<double>(min, max);

    if(val <= min)
    {
        val = _min;
    }
    else if(val >= max)
    {
        val = _max;
    }

    return val;
}

std::vector<cv::Vec2d> AUTOCONTROL::transform(std::vector<cv::Vec2d> pts, cv::Vec3d xi)
{
    cv::Matx22d R;
    R(0, 0) = std::cos(xi[2]);
    R(0, 1) = -std::sin(xi[2]);
    R(1, 0) = std::sin(xi[2]);
    R(1, 1) = std::cos(xi[2]);

    cv::Vec2d T;
    T[0] = xi[0];
    T[1] = xi[1];

    std::vector<cv::Vec2d> res;
    for(size_t p = 0; p < pts.size(); p++)
    {
        cv::Vec2d _P = R * pts[p] + T;
        res.push_back(_P);
    }
    return res;
}

double AUTOCONTROL::calc_point_to_line_segment_dist(cv::Vec2d P0, cv::Vec2d P1, cv::Vec2d P)
{
    double dx = P1[0]-P0[0];
    double dy = P1[1]-P0[1];
    double l2 = dx*dx + dy*dy;
    if(l2 == 0)
    {
        return cv::norm(P1-P);
    }

    double t = std::max<double>(0.0, std::min<double>(1.0, (P-P0).dot(P1-P0)/l2));
    cv::Vec2d proj = P0 + t*(P1-P0);

    return cv::norm(proj-P);
}

std::vector<cv::Vec2i> AUTOCONTROL::line_iterator(cv::Vec2i pt0, cv::Vec2i pt1)
{
    std::vector<cv::Vec2i> res;

    int x1 = pt0[0];
    int y1 = pt0[1];
    int x2 = pt1[0];
    int y2 = pt1[1];

    int add_x = 0;
    int add_y = 0;
    int count = 0;

    int dx = x2-x1;
    int dy = y2-y1;

    if(dx < 0)
    {
        add_x = -1;
        dx = -dx;
    }
    else
    {
        add_x = 1;
    }

    if(dy < 0)
    {
        add_y = -1;
        dy = -dy;
    }
    else
    {
        add_y = 1;
    }

    int x = x1;
    int y = y1;

    if(dx >= dy)
    {
        for(int i = 0; i < dx; i++)
        {
            x += add_x;
            count += dy;

            if(count >= dx)
            {
                y += add_y;
                count -= dx;
            }

            res.push_back(cv::Vec2i(x,y));
        }
    }
    else
    {
        for(int i = 0; i < dy; i++)
        {
            y += add_y;
            count += dx;

            if(count >= dy)
            {
                x += add_x;
                count -= dy;
            }

            res.push_back(cv::Vec2i(x,y));
        }
    }

    return res;
}

std::vector<cv::Vec2i> AUTOCONTROL::filled_circle_iterator(cv::Vec2i pt, int r)
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

int AUTOCONTROL::find_nn_path_idx(cv::Vec2d pt, std::vector<PATH_POINT>& path)
{
    int min_id = 0;
    double min_d = 99999999;
    for(size_t p = 0; p < path.size(); p++)
    {
        double d = cv::norm(path[p].pt - pt);
        if(d < min_d)
        {
            min_d = d;
            min_id = p;
        }
    }
    return min_id;
}

cv::Vec2d AUTOCONTROL::find_nn_path_point(cv::Vec2d pt, std::vector<PATH_POINT>& path)
{
    int min_id = 0;
    double min_d = 99999999;
    for(size_t p = 0; p < path.size(); p++)
    {
        double d = cv::norm(path[p].pt - pt);
        if(d < min_d)
        {
            min_d = d;
            min_id = p;
        }
    }
    return path[min_id].pt;
}

double AUTOCONTROL::find_nn_path_dist(cv::Vec2d pt, std::vector<PATH_POINT>& path)
{
    double min_d = 99999999;
    for(size_t p = 0; p < path.size(); p++)
    {
        double d = cv::norm(path[p].pt - pt);
        if(d < min_d)
        {
            min_d = d;
        }
    }
    return min_d;
}

std::vector<cv::Vec2i> AUTOCONTROL::path_simplify(cv::Mat& cost, std::vector<cv::Vec2i>& path)
{
    std::vector<cv::Vec2i> res;
    res.push_back(path.front());

    int last_p = 0;
    while(1)
    {
        if(cost.ptr<uchar>(path[last_p+1][1])[path[last_p+1][0]] == 0)
        {
            res.push_back(path[last_p+1]);
            last_p = last_p+1;
        }
        else
        {
            int max_los_p = last_p+1;
            for(int p = last_p+1; p < (int)path.size(); p++)
            {
                bool is_los = true;
                std::vector<cv::Vec2i> line = line_iterator(path[last_p], path[p]);
                for(size_t i = 0; i < line.size(); i++)
                {
                    int u = line[i][0];
                    int v = line[i][1];
                    if(cost.ptr<uchar>(v)[u] == 255 || cost.ptr<uchar>(v)[u] < 127)
                    {
                        is_los = false;
                        break;
                    }
                }

                if(is_los)
                {
                    max_los_p = p;
                }
            }

            res.push_back(path[max_los_p]);
            last_p = max_los_p;
        }

        if(last_p >= (int)path.size()-1)
        {
            break;
        }
    }

    return res;
}

void AUTOCONTROL::path_smoother(std::vector<cv::Vec2d> &path)
{
    if(path.size() < 7)
    {
        return;
    }

    std::array<double, 7> filter{-2.0/21.0, 3.0/21.0, 6.0/21.0, 7.0/21.0, 6.0/21.0, 3.0/21.0, -2.0/21.0};
    auto applyFilter = [&](const std::vector<double> & data) -> float
    {
        float val = 0.0;
        for (unsigned int i = 0; i != filter.size(); i++)
        {
          val += filter[i] * data[i];
        }
        return val;
    };

    for(size_t p = 3; p < path.size()-3; p++)
    {
        path[p][0] = applyFilter({path[p-3][0], path[p-2][0], path[p-1][0], path[p+0][0], path[p+1][0], path[p+2][0], path[p+3][0]});
        path[p][1] = applyFilter({path[p-3][1], path[p-2][1], path[p-1][1], path[p+0][1], path[p+1][1], path[p+2][1], path[p+3][1]});
    }
}

std::vector<PATH_POINT> AUTOCONTROL::calc_path(cv::Vec3d st_pose, cv::Vec3d ed_pose, std::vector<cv::Vec2d> cur_scan, bool avoid)
{
    // st, ed same position
    double st_ed_dist = cv::norm(cv::Vec2d(ed_pose[0], ed_pose[1]) - cv::Vec2d(st_pose[0], st_pose[1]));
    if(st_ed_dist < robot_config.robot_goal_near_dist)
    {
        logger.write("[PATH] ST, ED location very close, anyway we can go", true);

        PATH_POINT st;
        st.pt = cv::Vec2d(st_pose[0], st_pose[1]);
        st.od = 0;
        st.th = st_pose[2];
        st.v = robot_config.robot_st_v;

        PATH_POINT ed;
        ed.pt = cv::Vec2d(ed_pose[0], ed_pose[1]);
        ed.od = st_ed_dist;
        ed.th = st_pose[2];
        ed.v = robot_config.robot_goal_v;

        PATH_POINT mid;
        mid.pt = cv::Vec2d((st_pose[0]+ed_pose[0])/2, (st_pose[1]+ed_pose[1])/2);
        mid.od = st_ed_dist/2;
        mid.th = st_pose[2];
        mid.v = robot_config.robot_st_v;

        std::vector<PATH_POINT> res;
        res.push_back(st);        
        res.push_back(mid);
        res.push_back(ed);
        return res;
    }

    // get ref cost map
    cv::Mat cost_map = unimap->get_map_cost();

    // scan to cost map    
    cv::Mat scan(cost_map.rows, cost_map.cols, CV_8U, cv::Scalar(0));
    if(avoid == true)
    {
        // scan to scan map
        for(size_t p = 0; p < cur_scan.size(); p++)
        {
            cv::Vec2i uv = unimap->xy_uv(cur_scan[p]);

            const int r = robot_config.robot_radius/robot_config.robot_grid_size;
            cv::circle(scan, cv::Point(uv[0], uv[1]), r, cv::Scalar(255), -1);
        }

        // robot to scan map
        if(robot_config.robot_use_multi)
        {
            mtx.lock();
            const double robot_diameter = 2.0*robot_config.robot_radius;
            for(size_t p = 0; p < robots.size(); p++)
            {
                if(robots[p].id == -1 || robots[p].id == robot_config.robot_id)
                {
                    continue;
                }

                const double magin = 0.2;
                int r = (robot_diameter + magin)/robot_config.robot_grid_size;

                cv::Vec2i uv = unimap->xy_uv(robots[p].path[0]);
                cv::circle(scan, cv::Point(uv[0], uv[1]), r, cv::Scalar(255), -1);
            }
            mtx.unlock();
        }

        // set cost map
        for(int i = 0; i < cost_map.rows; i++)
        {
            for(int j = 0; j < cost_map.cols; j++)
            {
                if(scan.ptr<uchar>(i)[j] == 255)
                {
                    cost_map.ptr<uchar>(i)[j] = 255;
                }
            }
        }
    }

    // convert st and ed point to uv
    cv::Vec2i st_uv = unimap->xy_uv(cv::Vec2d(st_pose[0], st_pose[1]));
    cv::Vec2i ed_uv = unimap->xy_uv(cv::Vec2d(ed_pose[0], ed_pose[1]));

    // check
    {
        cv::Mat cost_map0 = unimap->get_map_cost0();
        cv::floodFill(cost_map0, cv::Point(st_uv[0], st_uv[1]), cv::Scalar(0));
        if(cost_map0.ptr<uchar>(ed_uv[1])[ed_uv[0]] != 0)
        {
            logger.write("[PATH] st, ed not connected, there are no way, no path", true);
            return std::vector<PATH_POINT>();
        }
    }

    // path planning
    ASTAR astar;
    std::vector<cv::Vec2i> path_uv = astar.path_finding(cost_map, st_uv, ed_uv, 1.0);
    if(path_uv.size() == 0)
    {
        // no path
        logger.write("[PATH] astar no path", true);
        return std::vector<PATH_POINT>();
    }

    // simplify
    path_uv = path_simplify(cost_map, path_uv);

    // convert uv to xy
    std::vector<cv::Vec2d> path_xy;
    path_xy.push_back(cv::Vec2d(st_pose[0], st_pose[1]));
    for(size_t p = 1; p < path_uv.size()-1; p++)
    {
        path_xy.push_back(unimap->uv_xy(path_uv[p]));
    }
    path_xy.push_back(cv::Vec2d(ed_pose[0], ed_pose[1]));

    // add mid point for interpolation
    if(path_xy.size() == 2)
    {
        path_xy.insert(path_xy.begin()+1, (path_xy[0]+path_xy[1])/2);
    }

    // linear interpolation
    if(path_xy.size() >= 3)
    {
        std::vector<cv::Vec2d> intp_path;

        // set data
        std::vector<double> D;
        std::vector<double> X;
        std::vector<double> Y;

        double sum_d = 0;
        for(size_t p = 0; p < path_xy.size(); p++)
        {
            X.push_back(path_xy[p][0]);
            Y.push_back(path_xy[p][1]);

            if(p >= 1)
            {
                sum_d += cv::norm(path_xy[p] - path_xy[p-1]);
            }
            D.push_back(sum_d);
        }

        tk::spline::spline_type type = tk::spline::linear;
        tk::spline sx, sy;
        sx.set_points(D, X, type);
        sy.set_points(D, Y, type);

        for(double d = 0; d <= sum_d; d += 0.1)
        {
            intp_path.push_back(cv::Vec2d(sx(d), sy(d)));
        }

        if(intp_path.back() != cv::Vec2d(ed_pose[0], ed_pose[1]))
        {
            intp_path[intp_path.size()-1] = cv::Vec2d(ed_pose[0], ed_pose[1]);
        }

        path_xy = intp_path;
    }

    // spline path smoothing
    std::vector<PATH_POINT> path;
    if(path_xy.size() >= 3)
    {
        // set data
        std::vector<double> D;
        std::vector<double> X;
        std::vector<double> Y;

        double sum_d = 0;
        for(size_t p = 0; p < path_xy.size(); p++)
        {
            X.push_back(path_xy[p][0]);
            Y.push_back(path_xy[p][1]);

            if(p >= 1)
            {
                sum_d += cv::norm(path_xy[p] - path_xy[p-1]);
            }
            D.push_back(sum_d);
        }

        tk::spline::spline_type type = tk::spline::cspline_hermite;
        tk::spline sx, sy;
        sx.set_points(D, X, type);
        sx.make_monotonic();
        sy.set_points(D, Y, type);
        sy.make_monotonic();

        // set path tree
        std::vector<cv::Vec2d> path_xy2;
        for(double d = 0; d <= sum_d; d += 0.01)
        {
            path_xy2.push_back(cv::Vec2d(sx(d), sy(d)));
        }

        if(path_xy2.back() != cv::Vec2d(ed_pose[0], ed_pose[1]))
        {
            path_xy2[path_xy2.size()-1] = cv::Vec2d(ed_pose[0], ed_pose[1]);
        }

        // SG smoother
        path_smoother(path_xy2);

        // set result
        for(size_t p = 0; p < path_xy2.size(); p++)
        {
            PATH_POINT ppt;
            ppt.pt = path_xy2[p];
            if(p == 0)
            {
                ppt.od = 0;
            }
            else
            {
                double d = cv::norm(path_xy2[p+1]-path_xy2[p]);
                ppt.od = d;
            }

            path.push_back(ppt);
        }
    }

    // set global th
    for(size_t p = 0; p < path.size()-1; p++)
    {
        cv::Vec2d dpt = path[p+1].pt-path[p].pt;
        double th = std::atan2(dpt[1], dpt[0]);
        path[p].th = th;
    }
    path[path.size()-1].th = path[path.size()-2].th;

    // th smoothing and set delta th
    for(size_t p = 0; p < path.size()-1; p++)
    {
        double dth = path[p+1].th - path[p].th;
        if(std::abs(dth) < 0.00001)
        {
            dth = 0;
        }

        while (dth > M_PI/2.0)
        {
            path[p+1].th -= M_PI*2.0;
            dth = path[p+1].th - path[p].th;
        }

        while (dth < -M_PI/2.0)
        {
            path[p+1].th += M_PI*2.0;
            dth = path[p+1].th - path[p].th;
        }

        path[p].dth = dth;
    }

    // calc base velocity profile    
    double look_ahead_dist = robot_config.robot_look_ahead_dist;
    double v_limit = robot_config.robot_limit_v * whole_gain;
    double w_limit = robot_config.robot_limit_w;

    std::vector<double> ref_v_list;    
    for(size_t p = 0; p < path.size(); p++)
    {
        // pp
        int cur_idx = p;
        int look_ahead_idx = get_look_ahead_idx(cur_idx, look_ahead_dist, path);

        cv::Vec2d tgt = path[look_ahead_idx].pt;
        double dx = tgt[0] - path[cur_idx].pt[0];
        double dy = tgt[1] - path[cur_idx].pt[1];
        double th = std::atan2(dy, dx);

        double err_d = std::sqrt(dx*dx + dy*dy);
        double err_th = deltaRad(th, path[cur_idx].th) * 2.0;

        double t_v = err_d/v_limit;
        double _w = std::abs(err_th/t_v);
        double ratio = 1.0;
        if(_w > w_limit)
        {
            ratio = w_limit/_w;
        }

        double v = v_limit*ratio;
        ref_v_list.push_back(v);
    }
    ref_v_list[ref_v_list.size()-1] = ref_v_list[ref_v_list.size()-2];

    // narrow area deceleration
    int chk_radius = std::ceil(0.15/robot_config.robot_grid_size);
    for(size_t p = 0; p < path.size(); p++)
    {
        cv::Vec2i uv = unimap->xy_uv(path[p].pt);
        std::vector<cv::Vec2i> circle_pts = filled_circle_iterator(uv, chk_radius);

        bool is_near = false;
        for(size_t q = 0; q < circle_pts.size(); q++)
        {
            int u = circle_pts[q][0];
            int v = circle_pts[q][1];
            if(u < 0 || u >= cost_map.cols || v < 0 || v >= cost_map.rows)
            {
                continue;
            }

            if(cost_map.ptr<uchar>(v)[u] == 255)
            {
                is_near = true;
                break;
            }
        }

        if(is_near)
        {
            ref_v_list[p] *= robot_config.robot_narrow_decel_ratio;
        }
    }

    // filtering    
    ref_v_list = smoothing_ref_v(ref_v_list);
    ref_v_list = average_filter(ref_v_list, 15);

    // update result
    for(size_t p = 0; p < path.size(); p++)
    {
        path[p].v = ref_v_list[p];
        path[p].obs_v = v_limit;

        //printf("%f\n", path[p].v);
    }

    return path;
}

std::vector<double> AUTOCONTROL::median_filter(std::vector<double>& src, int mask)
{
    std::vector<double> res = src;
    for(int p = mask; p < (int)src.size()-mask; p++)
    {
        std::vector<double> vels;
        for(int q = -mask; q <= mask; q++)
        {
            int i = p+q;
            vels.push_back(src[i]);
        }

        auto m = vels.begin() + vels.size()/2;
        std::nth_element(vels.begin(), m, vels.end());

        double median = vels[vels.size()/2];
        res[p] = median;
    }

    return res;
}

std::vector<double> AUTOCONTROL::erode_filter(std::vector<double>& src, int mask)
{
    std::vector<double> res = src;
    for(int p = mask; p < (int)src.size()-mask; p++)
    {
        double max = -9999;
        for(int q = -mask; q <= mask; q++)
        {
            int i = p+q;
            double val = src[i];
            if(val > max)
            {
                max = val;
            }
        }
        res[p] = max;
    }

    return res;
}

std::vector<double> AUTOCONTROL::dilate_filter(std::vector<double>& src, int mask)
{
    std::vector<double> res = src;
    for(int p = mask; p < (int)src.size()-mask; p++)
    {
        double min = 9999;
        for(int q = -mask; q <= mask; q++)
        {
            int i = p+q;
            double val = src[i];
            if(val < min)
            {
                min = val;
            }
        }
        res[p] = min;
    }

    return res;
}

std::vector<double> AUTOCONTROL::average_filter(std::vector<double>& src, int mask)
{
    std::vector<double> res = src;
    for(int p = mask; p < (int)src.size()-mask; p++)
    {
        int cnt = 0;
        double sum = 0;
        for(int q = -mask; q <= mask; q++)
        {
            int i = p+q;
            double val = src[i];
            sum += val;
            cnt += 1;
        }
        res[p] = sum/cnt;
    }

    return res;
}

std::vector<double> AUTOCONTROL::smoothing_ref_v(std::vector<double>& src)
{
    double delta_v = robot_config.robot_limit_v_acc/100.0;
    double v_limit = robot_config.robot_limit_v * whole_gain;

    std::vector<double> list0(src.size());
    double vv0 = robot_config.robot_st_v;
    for(int p = 0; p < (int)src.size(); p++)
    {
        if(src[p] > vv0)
        {    
            vv0 += delta_v;
        }
        else
        {
            vv0 = src[p];
        }

        if(vv0 > v_limit)
        {
            vv0 = v_limit;
        }

        list0[p] = vv0;
    }

    std::vector<double> list1(src.size());
    double vv1 = robot_config.robot_goal_v;
    for(int p = src.size()-1; p >= 0; p--)
    {
        if(src[p] > vv1)
        {
            vv1 += delta_v;
        }
        else
        {
            vv1 = src[p];
        }

        if(vv1 > v_limit)
        {
            vv1 = v_limit;
        }

        list1[p] = vv1;
    }

    std::vector<double> res(src.size());
    for(size_t p = 0; p < src.size(); p++)
    {
        res[p] = std::min<double>(list0[p], list1[p]);
    }

    return res;
}

cv::Vec2d AUTOCONTROL::get_look_ahead_point(cv::Vec2d pt, double r, std::vector<PATH_POINT>& path)
{
    int idx = find_nn_path_idx(pt, path);
    int look_ahead_offset = r/0.01;
    int look_ahead_idx = idx+look_ahead_offset;
    if(look_ahead_idx > (int)path.size()-1)
    {
        look_ahead_idx = path.size()-1;
    }

    cv::Vec2d tgt = path[look_ahead_idx].pt;
    return tgt;
}

int AUTOCONTROL::get_look_ahead_idx(int cur_idx, double r, std::vector<PATH_POINT>& path)
{
    int idx = cur_idx;
    int look_ahead_offset = r/0.01;
    int look_ahead_idx = idx+look_ahead_offset;
    if(look_ahead_idx > (int)path.size()-1)
    {
        look_ahead_idx = path.size()-1;
    }

    return look_ahead_idx;
}

double AUTOCONTROL::calc_motion_time(double _s, double _v0, double _v1, double _acc)
{
    if(std::abs(_s) == 0)
    {
        return 0;
    }

    if((_v0 >= 0 && _v1 >= 0) || (_v0 < 0 && _v1 < 0))
    {
        // v0, v1 same sign
        double v0 = std::abs(_v0);
        double v1 = std::abs(_v1);
        double s = std::abs(_s);
        double acc;
        if(v1 - v0 > 0)
        {
            // accel
            acc = _acc;
        }
        else if(v1 - v0 < 0)
        {
            // decel
            acc = -_acc;
        }
        else
        {
            // constant case
            double t = s/v0;
            return t;
        }

        double t0 = (v1-v0)/acc;
        double s0 = v0*t0+0.5*acc*t0*t0;
        if(s0 >= s)
        {
            double a = 0.5*acc;
            double b = v0;
            double c = -s;

            // always b^2 - 4ac > 0, because c is minus
            double tmp0 = (-b - std::sqrt(b*b - 4*a*c))/(2*a);
            double tmp1 = (-b + std::sqrt(b*b - 4*a*c))/(2*a);
            double t = std::max<double>(tmp0, tmp1);
            return t;
        }

        double t1 = (s - v0*t0 - 0.5*(v1-v0)*t0 + v1*t0)/v1;
        return t1;
    }
    else if((_v0 >= 0 && _v1 < 0) || (_v0 < 0 && _v1 >= 0))
    {
        // v0, v1 different sign
        double v0 = std::abs(_v0);
        double v1 = std::abs(_v1);
        double s = std::abs(_s);
        double acc = std::abs(_acc);

        double t0 = v0/acc;
        double t1 = t0 + v1/acc;

        double s1 = -0.5*v0*t0 + 0.5*v1*(t1-t0);
        if(s1 >= s)
        {
            v0 *= -1;
            double a = 0.5*acc;
            double b = v0;
            double c = -s;

            // always b^2 - 4ac > 0, because c is minus
            double tmp0 = (-b - std::sqrt(b*b - 4*a*c))/(2*a);
            double tmp1 = (-b + std::sqrt(b*b - 4*a*c))/(2*a);
            double t = std::max<double>(tmp0, tmp1);
            return t;
        }

        double t2 = (s + 0.5*v0*t0 - 0.5*v1*(t1-t0) + v1*t1)/v1;
        return t2;
    }
}

cv::Vec2d AUTOCONTROL::calc_pp(cv::Vec3d cur_pose, cv::Vec2d cur_vw, std::vector<PATH_POINT>& path, double dt)
{
    // pp
    cv::Vec2d cur_pt = cv::Vec2d(cur_pose[0], cur_pose[1]);
    int cur_idx = find_nn_path_idx(cur_pt, path);

    double v_acc = robot_config.robot_limit_v_acc;
    double w_acc = robot_config.robot_limit_w_acc;

    double v0 = cur_vw[0];
    double w0 = cur_vw[1];

    double v1 = std::min<double>(path[cur_idx].v, path[cur_idx].obs_v);
    double w1 = robot_config.robot_limit_w;

    double look_ahead_dist = std::max(robot_config.robot_min_look_ahead_dist, std::min<double>(v1, robot_config.robot_look_ahead_dist));
    w1 *= (look_ahead_dist*2);

    int look_ahead_idx = get_look_ahead_idx(cur_idx, look_ahead_dist, path);

    cv::Vec2d tgt = path[look_ahead_idx].pt;
    double dx = tgt[0] - cur_pt[0];
    double dy = tgt[1] - cur_pt[1];
    double th = std::atan2(dy, dx);

    double err_d = std::sqrt(dx*dx + dy*dy);
    double err_th = deltaRad(th, cur_pose[2]) * 2.0;    

    double t_v = calc_motion_time(err_d, v0, v1, v_acc);
    double t_w = calc_motion_time(err_th, w0, w1, w_acc);

    double t = std::max<double>(t_v, t_w);
    double v = err_d/t;
    double w = err_th/t;
    double dv = (v-cur_vw[0]);

    double ref_dv = robot_config.robot_limit_v_acc * dt;    
    if(std::abs(dv) > ref_dv)
    {
        dv = sgn(dv)*ref_dv;
    }

    v = cur_vw[0] + dv;
    v = saturation(v, -v1, v1);

    cv::Vec2d res(v, w);
    return res;
}

std::vector<cv::Vec6d> AUTOCONTROL::calc_trajectory(cv::Vec2d vw, double predict_time, double dt, cv::Vec3d cur_pose, cv::Vec2d cur_vw)
{
    double v_acc = robot_config.motor_limit_v_acc;
    double w_acc = robot_config.motor_limit_w_acc;
    double v0 = cur_vw[0];
    double w0 = cur_vw[1];

    double v = vw[0];
    double w = vw[1];
    double sgn_v = sgn(v-v0);
    double sgn_w = sgn(w-w0);

    cv::Vec6d state(0, 0, 0, v0, w0, 0); // x, y, th, v, w, sum_dt

    std::vector<cv::Vec6d> traj;
    traj.push_back(state);

    for(double sum_dt = 0; sum_dt <= predict_time; sum_dt += dt)
    {
        double _v = state[3] + sgn_v*v_acc*dt;
        if(sgn_v < 0)
        {
            if(_v <= v)
            {
                _v = v;
            }
        }
        else if(sgn_v > 0)
        {
            if(_v >= v)
            {
                _v = v;
            }
        }

        double _w = state[4] + sgn_w*w_acc*dt;
        if(sgn_w < 0)
        {
            if(_w <= w)
            {
                _w = w;
            }
        }
        else if(sgn_w > 0)
        {
            if(_w >= w)
            {
                _w = w;
            }
        }

        state[2] = toWrap(state[2] + _w*dt);
        state[0] += _v*std::cos(state[2])*dt; // x
        state[1] += _v*std::sin(state[2])*dt; // y
        state[3] = _v;
        state[4] = _w;
        state[5] = sum_dt;

        // save trajectory
        traj.push_back(state);
    }

    // trajectory global transformation
    cv::Matx22d R;
    R(0, 0) = std::cos(cur_pose[2]);
    R(0, 1) = -std::sin(cur_pose[2]);
    R(1, 0) = std::sin(cur_pose[2]);
    R(1, 1) = std::cos(cur_pose[2]);

    cv::Vec2d T;
    T[0] = cur_pose[0];
    T[1] = cur_pose[1];

    for(size_t p = 0; p < traj.size(); p++)
    {
        cv::Vec2d P(traj[p][0], traj[p][1]);
        cv::Vec2d _P = R * P + T;
        traj[p][0] = _P[0]; // x
        traj[p][1] = _P[1]; // y
        traj[p][2] = toWrap(cur_pose[2]+traj[p][2]); // th
    }

    return traj;
}

void AUTOCONTROL::marking_obstacle(int cur_idx, std::vector<PATH_POINT>& path, double deadzone)
{
    // clear
    double v_limit = robot_config.robot_limit_v * whole_gain;
    std::vector<double> ref_v(path.size(), v_limit);
    for(int p = 0; p < (int)path.size(); p++)
    {
        path[p].obs = 0;
        path[p].obs_v = v_limit;
    }

    // marking
    int mask_range = deadzone/0.01;
    for(int p = cur_idx; p < (int)path.size(); p++)
    {
        if(unimap->is_near_dynamic_obs(path[p].pt))
        {
            // fill mask
            for(int q = -mask_range; q <= mask_range; q++)
            {
                int i = p+q;
                if(i < 0 || i >= (int)path.size())
                {
                    continue;
                }

                path[i].obs = 255;
                ref_v[i] = robot_config.robot_goal_v;
            }
        }

        if(robot_config.robot_use_multi)
        {
            mtx.lock();
            for(size_t k = 0; k < robots.size(); k++)
            {
                if(robots[k].id == -1 || robots[k].id == robot_config.robot_id)
                {
                    continue;
                }

                const double robot_diameter = 2.0*robot_config.robot_radius;
                double d = cv::norm(path[p].pt - robots[k].path[0]);
                if(d < robot_diameter)
                {
                    // fill mask
                    int mask_range2 = (robot_diameter + deadzone)/0.01;
                    for(int q = -mask_range2; q <= mask_range2; q++)
                    {
                        int i = p+q;
                        if(i < 0 || i >= (int)path.size())
                        {
                            continue;
                        }

                        path[i].obs = robots[k].id;
                        ref_v[i] = robot_config.robot_goal_v;
                    }
                }
            }
            mtx.unlock();
        }
    }

    ref_v = smoothing_ref_v(ref_v);
    for(int p = 0; p < (int)path.size(); p++)
    {
        path[p].obs_v = ref_v[p];
    }
}

void AUTOCONTROL::clear_obstacle(std::vector<PATH_POINT>& path)
{
    double v_limit = robot_config.robot_limit_v * whole_gain;
    for(int p = 0; p < (int)path.size(); p++)
    {
        path[p].obs = 0;
        path[p].obs_v = v_limit;
    }
}

void AUTOCONTROL::ifsmLoop(cv::Vec3d _goal_pose, bool is_avoid, bool is_align)
{
    // real time loop
    const double dt = 0.02; // 50hz

    // params
    double pivot_limit = robot_config.robot_limit_pivot;
    double pivot_acc = robot_config.robot_limit_pivot_acc;

    // init variable
    int obs_type = 0;
    int path_fail_cnt = 0;
    double pre_loop_time = 0;

    cv::Vec2d pre_u(0, 0);
    std::vector<PATH_POINT> path;    
    cv::Mat cost_map0 = unimap->get_map_cost0();

    // state machine
    logger.write("[IFSM] start", true);
    ifsm_state = STATE_PATH_FINDING;
    while(ifsmFlag)
    {
        // get current pose and sensor data        
        MOBILE_STATUS mobile_status = mobile->get_status();
        if(mobile_status.is_ok && mobile_status.emo_state == 0)
        {
            // emergency stop
            logger.write("[IFSM] stop", true);

            mobile->move(0, 0);
            pre_u = cv::Vec2d(0, 0);
            ifsm_state = STATE_STOPPED;
            return;
        }

        MOBILE_POSE mobile_pose = mobile->get_pose();        
        cv::Vec2d cur_vw = mobile_pose.vw;
        cv::Vec3d cur_pose = slam->get_cur_pose();        
        std::vector<cv::Vec2d> cur_scan = slam->get_cur_scan_global();
        std::vector<ROBOT_SHARED> _robots = get_robots();

        // finite state machine
        if(ifsm_state == STATE_PATH_FINDING)
        {
            // pause
            if(is_pause == true)
            {
                logger.write("[IFSM] STATE_PATH_FINDING -> PAUSE", true);

                ifsm_state = STATE_PAUSE;
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                continue;
            }

            // finding path
            std::vector<PATH_POINT> _path = calc_path(cur_pose, _goal_pose, cur_scan, is_avoid);
            if(_path.size() > 0)
            {
                logger.write("[IFSM] PATH_FINDING SUCCESS -> FIRST_ALIGN", true);

                // set current path for plot
                mtx.lock();
                path = _path;
                cur_path = _path;
                mtx.unlock();

                // clear path fail count
                path_fail_cnt = 0;                
                ifsm_state = STATE_FIRST_ALIGN;
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                continue;
            }
            else
            {
                // try 20, wait 10 sec
                path_fail_cnt++;
                if(path_fail_cnt > 20)
                {
                    logger.write("[IFSM] STATE_PATH_FINDING -> STATE_PATH_FAILED", true);

                    // clear path fail count
                    path_fail_cnt = 0;
                    ifsm_state = STATE_PATH_FAILED;
                    return;
                }

                logger.write("[IFSM] PATH_FINDING FAILED", true);
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                continue;
            }
        }
        else if(ifsm_state == STATE_FIRST_ALIGN)
        {
            // pause
            if(is_pause == true)
            {
                logger.write("[IFSM] STATE_FIRST_ALIGN -> PAUSE", true);

                ifsm_state = STATE_PAUSE;
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                continue;
            }

            // checking obstacle
            marking_obstacle(0, path, robot_config.robot_obs_deadzone);
            if(path[0].obs != 0)
            {
                obs_type = path[0].obs;
                logger.write("[IFSM] FIRST_ALIGN -> STATE_OBSTACLE", true);

                mobile->move(0, 0);
                pre_u = cv::Vec2d(0, 0);
                ifsm_state = STATE_OBSTACLE;
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                continue;
            }

            // first align            
            cv::Vec2d tgt = get_look_ahead_point(cv::Vec2d(cur_pose[0], cur_pose[1]), robot_config.robot_min_look_ahead_dist, path);
            double dx = tgt[0] - cur_pose[0];
            double dy = tgt[1] - cur_pose[1];
            double err_th = deltaRad(std::atan2(dy, dx), cur_pose[2]);
            double u_w = 0;

            // trapezoidal control
            if(err_th >= 0)
            {
                std::vector<double> w_candidates;
                w_candidates.push_back(pre_u[1] + pivot_acc * dt);
                w_candidates.push_back(pivot_limit);
                w_candidates.push_back(std::sqrt(2.0*pivot_acc*std::abs(err_th)));

                double min_w = 99999999;
                for(size_t p = 0; p < w_candidates.size(); p++)
                {
                    if(w_candidates[p] < min_w)
                    {
                        min_w = w_candidates[p];
                    }
                }

                u_w = min_w;
            }
            else
            {
                std::vector<double> w_candidates;
                w_candidates.push_back(pre_u[1] - pivot_acc * dt);
                w_candidates.push_back(-pivot_limit);
                w_candidates.push_back(-std::sqrt(2.0*pivot_acc*std::abs(err_th)));

                double max_w = -99999999;
                for(size_t p = 0; p < w_candidates.size(); p++)
                {
                    if(w_candidates[p] > max_w)
                    {
                        max_w = w_candidates[p];
                    }
                }

                u_w = max_w;
            }

            mobile->move(0, u_w);
            pre_u = cv::Vec2d(0, u_w);

            // check aligned
            if(std::abs(err_th) < robot_config.robot_goal_th)
            {
                QString str;
                str.sprintf("[IFSM] FIRST_ALIGN -> STATE_PURE_PURSUIT, err_th: %.2f/%.2f", err_th*R2D, robot_config.robot_goal_th*R2D);
                logger.write(str, true);

                mobile->move(0, 0);
                pre_u = cv::Vec2d(0, 0);
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));

                ifsm_state = STATE_PURE_PURSUIT;
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                continue;
            }
        }
        else if(ifsm_state == STATE_PURE_PURSUIT)
        {
            // pause
            if(is_pause == true)
            {
                logger.write("[IFSM] STATE_PURE_PURSUIT -> PAUSE", true);

                ifsm_state = STATE_PAUSE;
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                continue;
            }

            // checking obstacle
            int cur_idx = find_nn_path_idx(cv::Vec2d(cur_pose[0], cur_pose[1]), path);
            marking_obstacle(cur_idx, path, robot_config.robot_obs_deadzone);
            if(path[cur_idx].obs != 0)
            {
                obs_type = path[cur_idx].obs;
                logger.write("[IFSM] STATE_PURE_PURSUIT -> STATE_OBSTACLE", true);

                mobile->move(0, 0);
                pre_u = cv::Vec2d(0, 0);
                ifsm_state = STATE_OBSTACLE;
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                continue;
            }

            // check goal
            double ed_err_d = cv::norm(cv::Vec2d(_goal_pose[0], _goal_pose[1]) - cv::Vec2d(cur_pose[0], cur_pose[1]));
            if(std::abs(ed_err_d) < robot_config.robot_goal_dist)
            {
                // goal reached
                QString str;
                str.sprintf("[IFSM] STATE_PURE_PURSUIT -> FINAL_ALIGN, err_d: %.3f/%.3f", ed_err_d, robot_config.robot_goal_dist);
                logger.write(str, true);

                mobile->move(0, 0);
                pre_u = cv::Vec2d(0, 0);
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));

                ifsm_state = STATE_FINAL_ALIGN;
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                continue;
            }

            // check path out
            double path_d = cv::norm(path[cur_idx].pt - cv::Vec2d(cur_pose[0], cur_pose[1]));
            if(path_d > robot_config.robot_path_out_dist)
            {
                logger.write("[IFSM] STATE_PURE_PURSUIT -> GOAL_FAILED(path out)", true);

                mobile->move(0, 0);
                pre_u = cv::Vec2d(0, 0);
                ifsm_state = STATE_PATH_OUT;
                return;
            }

            // PP
            cv::Vec2d u_pp = calc_pp(cur_pose, pre_u, path, dt);
            mobile->move(u_pp[0], u_pp[1]);
            pre_u = u_pp;
        }        
        else if(ifsm_state == STATE_FINAL_ALIGN)
        {
            // pause
            if(is_pause == true)
            {
                logger.write("[IFSM] STATE_FINAL_ALIGN -> PAUSE", true);

                ifsm_state = STATE_PAUSE;
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                continue;
            }

            // no final align
            if(!is_align)
            {
                logger.write("[IFSM] FINAL_ALIGN PASSED", true);

                mobile->move(0, 0);
                pre_u = cv::Vec2d(0, 0);
                ifsm_state = STATE_GOAL_REACHED;
                return;
            }

            // final align
            double err_th = deltaRad(_goal_pose[2], cur_pose[2]);
            double u_w = 0;

            // trapezoidal control
            if(err_th >= 0)
            {
                std::vector<double> w_candidates;
                w_candidates.push_back(pre_u[1] + pivot_acc * dt);
                w_candidates.push_back(pivot_limit);
                w_candidates.push_back(std::sqrt(2.0*pivot_acc*std::abs(err_th)));

                double min_w = 99999999;
                for(size_t p = 0; p < w_candidates.size(); p++)
                {
                    if(w_candidates[p] < min_w)
                    {
                        min_w = w_candidates[p];
                    }
                }

                u_w = min_w;
            }
            else
            {
                std::vector<double> w_candidates;
                w_candidates.push_back(pre_u[1] - pivot_acc * dt);
                w_candidates.push_back(-pivot_limit);
                w_candidates.push_back(-std::sqrt(2.0*pivot_acc*std::abs(err_th)));

                double max_w = -99999999;
                for(size_t p = 0; p < w_candidates.size(); p++)
                {
                    if(w_candidates[p] > max_w)
                    {
                        max_w = w_candidates[p];
                    }
                }

                u_w = max_w;
            }

            mobile->move(0, u_w);
            pre_u = cv::Vec2d(0, u_w);

            // check final align
            if(std::abs(err_th) < robot_config.robot_goal_th)
            {
                QString str;
                str.sprintf("[IFSM] FINAL_ALIGN -> GOAL_REACHED, err_th: %.2f/%.2f", err_th*R2D, robot_config.robot_goal_th*R2D);
                logger.write(str, true);

                mobile->move(0, 0);
                pre_u = cv::Vec2d(0, 0);                
                ifsm_state = STATE_GOAL_REACHED;
                return;
            }
        }
        else if(ifsm_state == STATE_OBSTACLE)
        {
            // obstacle not robot
            if(obs_type == 255 || obs_type > robot_config.robot_id)
            {
                logger.write("[IFSM] STATE_OBSTACLE, just wait", true);                
                ifsm_state = STATE_WAIT;
                return;
            }

            // obstacle is robot
            if(robot_config.robot_use_multi)
            {
                const double robot_diameter = 2.0*robot_config.robot_radius;

                int obs_id = obs_type;
                ROBOT_SHARED obs_robot = _robots[obs_id];

                // robot mask
                cv::Mat robot_mask(cost_map0.rows, cost_map0.cols, CV_8U, cv::Scalar(0));
                for(size_t p = 0; p < _robots.size(); p++)
                {
                    if(_robots[p].id == -1 || _robots[p].id == robot_config.robot_id)
                    {
                        continue;
                    }

                    const double magin = 0.1;
                    int r = (robot_diameter + magin)/robot_config.robot_grid_size;

                    cv::Vec2i uv = unimap->xy_uv(_robots[p].path[0]);
                    cv::circle(robot_mask, cv::Point(uv[0], uv[1]), r, cv::Scalar(255), -1);
                }

                // avoid candidate mask
                cv::Mat candidate_mask(cost_map0.rows, cost_map0.cols, CV_8U, cv::Scalar(0));
                for(size_t p = 0; p < _robots[obs_id].path.size(); p++)
                {
                    const double magin = 1.0;
                    int r = (robot_diameter + magin)/robot_config.robot_grid_size;

                    cv::Vec2i uv = unimap->xy_uv(_robots[obs_id].path[p]);
                    cv::circle(candidate_mask, cv::Point(uv[0], uv[1]), r, cv::Scalar(255), -1);
                }

                for(size_t p = 0; p < _robots[obs_id].path.size(); p++)
                {
                    const double magin = 0.3;
                    int r = (robot_diameter + magin)/robot_config.robot_grid_size;

                    cv::Vec2i uv = unimap->xy_uv(_robots[obs_id].path[p]);
                    cv::circle(candidate_mask, cv::Point(uv[0], uv[1]), r, cv::Scalar(0), -1);
                }

                // avoid candidate pts
                std::vector<cv::Vec2d> avoid_pts;
                for(int i = 0; i < candidate_mask.rows; i++)
                {
                    for(int j = 0; j < candidate_mask.cols; j++)
                    {
                        if(candidate_mask.ptr<uchar>(i)[j] == 255)
                        {
                            if(robot_mask.ptr<uchar>(i)[j] == 0 && cost_map0.ptr<uchar>(i)[j] != 255)
                            {
                                cv::Vec2d pt = unimap->uv_xy(cv::Vec2i(j,i));
                                avoid_pts.push_back(pt);
                            }
                        }
                    }
                }

                // calc distance
                std::vector<std::pair<double, int>> dist_idx;
                for(size_t p = 0; p < avoid_pts.size(); p++)
                {
                    double d = cv::norm(avoid_pts[p] - cv::Vec2d(cur_pose[0], cur_pose[1]));
                    dist_idx.push_back(std::make_pair(d, p));
                }

                std::sort(dist_idx.begin(), dist_idx.end());

                // get one
                cv::Vec2d min_pt(0, 0);
                for(size_t p = 0; p < dist_idx.size(); p++)
                {
                    int idx = dist_idx[p].second;
                    cv::Vec3d pose(avoid_pts[idx][0], avoid_pts[idx][1], 0);
                    std::vector<PATH_POINT> path = calc_path(cur_pose, pose, std::vector<cv::Vec2d>(), true);
                    if(path.size() != 0)
                    {
                        min_pt = avoid_pts[idx];
                        break;
                    }
                }

                // add avoid location
                add_waypoint(cv::Vec3d(min_pt[0], min_pt[1], 0));

                logger.write("[IFSM] STATE_OBSTACLE, avoiding", true);                
                ifsm_state = STATE_AVOID;
                return;
            }
        }
        else if(ifsm_state == STATE_PAUSE)
        {
            // slow down stop            
            double v = pre_u[0] - 2.0*(robot_config.robot_limit_v_acc*dt);
            if(v < 0)
            {
                v = 0;
            }
            mobile->move(v, 0);
            pre_u = cv::Vec2d(v, 0);

            // resume
            if(is_pause == false)
            {
                logger.write("[IFSM] STATE_PAUSE -> RESUME -> STATE_PATH_FINDING", true);                
                mobile->move(0, 0);
                pre_u = cv::Vec2d(0, 0);

                ifsm_state = STATE_PATH_FINDING;
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                continue;
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
            printf("[IFSM] loop time drift, dt:%f\n", delta_loop_time);
        }
        pre_loop_time = get_time();
    }

    // emergency stop
    logger.write("[IFSM] STATE_STOPPED", true);
    mobile->move(0, 0);
    pre_u = cv::Vec2d(0, 0);
    ifsm_state = STATE_STOPPED;
}

void AUTOCONTROL::ofsmLoop()
{
    // init variable
    int wait_st_time = 0;

    // set default state
    ofsm_state = STATE_IDLE;

    // current desired goal
    cv::Vec3d pre_goal(0,0,0);

    // fsm start
    printf("auto loop start\n");
    while(ofsmFlag)
    {
        if(ofsm_state == STATE_IDLE || ofsm_state == STATE_FAILED)
        {
            mtx.lock();
            int goal_n = goal_list.size();
            mtx.unlock();

            if(goal_n > 0)
            {
                // change state
                ofsm_state = STATE_DRIVING;
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }
        }
        else if(ofsm_state == STATE_DRIVING)
        {
            // inner fsm start
            mtx.lock();
            cv::Vec3d goal = goal_list[0];
            mtx.unlock();

            if(pre_goal != goal)
            {
                if(goal_list.size() == 1)
                {
                    ifsm_run(goal, robot_config.robot_use_avoid, true);
                }
                else
                {
                    ifsm_run(goal, true, false);
                }

                pre_goal = goal;
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }

            // check state            
            if(ifsm_state == STATE_GOAL_REACHED)
            {
                // path clear
                mtx.lock();
                cur_path.clear();                
                goal_list.erase(goal_list.begin());
                int goal_n = goal_list.size();
                cv::Vec3d goal = goal_list.back();
                mtx.unlock();

                // change state
                if(goal_n == 0)
                {
                    ofsm_state = STATE_IDLE;
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    continue;
                }
                else
                {
                    // wait path clear
                    std::vector<PATH_POINT> path = calc_path(pre_goal, goal, std::vector<cv::Vec2d>(), false);

                    int cnt = 0;
                    while(1)
                    {
                        marking_obstacle(0, path, robot_config.robot_obs_deadzone);

                        bool is_clear = true;
                        for(size_t p = 0; p < path.size(); p++)
                        {
                            if(path[p].obs != 0)
                            {
                                is_clear = false;
                                break;
                            }
                        }

                        if(is_clear)
                        {
                            break;
                        }

                        cnt++;
                        if(cnt >= 10)
                        {
                            break;
                        }

                        std::this_thread::sleep_for(std::chrono::milliseconds(500));
                    }
                }
            }
            else if(ifsm_state == STATE_STOPPED)
            {
                // path clear
                mtx.lock();
                cur_path.clear();
                goal_list.clear();
                pre_goal = cv::Vec3d(0,0,0);
                mtx.unlock();

                // change state
                ofsm_state = STATE_IDLE;
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }
            else if(ifsm_state == STATE_PATH_OUT || ifsm_state == STATE_PATH_FAILED)
            {
                // path clear
                mtx.lock();
                cur_path.clear();
                goal_list.clear();
                pre_goal = cv::Vec3d(0,0,0);
                mtx.unlock();

                // change state
                ofsm_state = STATE_FAILED;
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }
            else if(ifsm_state == STATE_WAIT)
            {
                // save current time
                wait_st_time = get_time();                
                logger.write("[OFSM] OBSTACLE, wait some time", true);

                // change state
                ofsm_state = STATE_WAITING;
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }
        }
        else if(ofsm_state == STATE_WAITING)
        {
            // pause
            if(is_pause == true)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }

            // waiting 5 sec
            if(get_time() - wait_st_time > robot_config.robot_obs_wait_time)
            {
                // pre goal clear
                pre_goal = cv::Vec3d(0,0,0);

                ofsm_state = STATE_DRIVING;
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}
