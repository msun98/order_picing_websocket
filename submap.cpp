#include "submap.h"

SUBMAP::SUBMAP()
{
	xi = cv::Vec3d(0, 0, 0);    
	dxi = cv::Vec3d(0, 0, 0);
}

SUBMAP::~SUBMAP()
{
}

void SUBMAP::add_scan(std::vector<cv::Vec2d> &pts, cv::Vec3d _dxi)
{
	// set dxi
	dxi = _dxi;

	// 1. scan data
	cv::Vec2d lrf(0, 0);
	cv::Vec2i lrf_uv = xy_uv(lrf, _dxi);

	// scan to uv
    std::vector<cv::Vec2d> pts_raw;
    for(size_t p = 0; p < pts.size(); p++)
    {
        double range = cv::norm(pts[p]);
        if(range < 1.0)
        {
            continue;
        }

        pts_raw.push_back(pts[p]);
    }

	std::vector<cv::Vec2i> pts_raw_uv = xy_uv(pts_raw, _dxi);

    // append lrf center
	pts_raw_uv.push_back(lrf_uv);

	// min max
    int min_u = 999999;
    int max_u = -999999;
    int min_v = 999999;
    int max_v = -999999;
    for (size_t p = 0; p < pts_raw_uv.size(); p++)
	{
		cv::Vec2i uv = pts_raw_uv[p];
		if (uv[0] < min_u)
		{
			min_u = uv[0];
		}

		if (uv[0] > max_u)
		{
			max_u = uv[0];
		}

		if (uv[1] < min_v)
		{
			min_v = uv[1];
		}

		if (uv[1] > max_v)
		{
			max_v = uv[1];
		}
	}

	for (auto it = map.begin(); it != map.end(); it++)
	{
		int idx = it->first;
		cv::Vec2i uv = get_uv(idx);

		if (uv[0] < min_u)
		{
			min_u = uv[0];
		}

		if (uv[0] > max_u)
		{
			max_u = uv[0];
		}

		if (uv[1] < min_v)
		{
			min_v = uv[1];
		}

		if (uv[1] > max_v)
		{
			max_v = uv[1];
		}
	}
	
    // append remove lrf center
	pts_raw_uv.erase(pts_raw_uv.end() - 1);

	// resize map
	int du = -min_u + 1;
	int dv = -min_v + 1;
	w = (max_u - min_u) + 3;
	h = (max_v - min_v) + 3;
	ou += du;
	ov += dv;

	// recalc already map points 
	std::map<int, double> _map;
	for (auto it = map.begin(); it != map.end(); it++)
	{
		int idx = it->first;
		double val = it->second;

		cv::Vec2i uv = get_uv(idx) + cv::Vec2i(du, dv);
		int key = get_key(uv);
		_map[key] = val;
	}
	map.swap(_map);

	// hit table
	std::vector<cv::Vec2i> pts_hit_uv;
	std::unordered_set<int> hit_set;
    for (size_t p = 0; p < pts_raw_uv.size(); p++)
	{
		cv::Vec2i uv = pts_raw_uv[p] + cv::Vec2i(du, dv);
		int key = get_key(uv);
		if (hit_set.find(key) == hit_set.end())
		{
			hit_set.insert(key);
		}
		pts_hit_uv.push_back(uv);
	}

	// probability update ray casting
	std::vector<std::vector<cv::Vec2i>> rays;	
    for (size_t p = 0; p < pts_raw.size(); p++)
	{
		double x = pts_raw[p][0] - lrf[0];
		double y = pts_raw[p][1] - lrf[1];
		double d = std::sqrt(x * x + y * y);
		double r = std::atan2(y, x);

		std::vector<cv::Vec2i> ray;
		std::unordered_set<int> set;
        for (double _d = 0; _d <= d; _d += (robot_config.robot_grid_size*0.4))
		{
			double _x = _d * cos(r) + lrf[0];
			double _y = _d * sin(r) + lrf[1];

			cv::Vec2i uv = xy_uv(cv::Vec2d(_x, _y), _dxi);
			int key = get_key(uv);
			if (hit_set.find(key) != hit_set.end())
			{
				break;
			}

			if (set.find(key) == set.end())
			{
				set.insert(key);
				ray.push_back(uv);
			}
			
		}
		rays.push_back(ray);
	}

    /*
	// robot location
	cv::Mat temp(h, w, CV_8U, cv::Scalar(0));
	cv::Vec2i center = xy_uv(cv::Vec2d(0, 0), _dxi);
    cv::circle(temp, center, std::ceil(robot_config.robot_radius/GRID_WIDTH), cv::Scalar(255), -1);
	for (int i = 0; i < h; i++)
	{
		for (int j = 0; j < w; j++)
		{
			if (temp.ptr<uchar>(i)[j] == 255)
			{
				// P_miss
				int key = get_key(cv::Vec2i(j, i));
				map[key] = P_min;
			}
		}
	}
    */

	// update probability map
    for (size_t p = 0; p < rays.size(); p++)
	{
        for (size_t q = 0; q < rays[p].size(); q++)
		{
			// P_miss
			int key = get_key(rays[p][q]);
			if (hit_set.find(key) != hit_set.end())
			{
				break;
			}

			if (num == 0)
			{
				map[key] = P_min;
				continue;
			}
							
			if (map.find(key) == map.end())
			{
				map[key] = P_miss;
			}
			else
			{
				double m_old = map[key];
				double m_new = prob(m_old, P_miss);
				map[key] = m_new;
			}
		}
	}

    for (size_t p = 0; p < pts_hit_uv.size(); p++)
	{	
		// P_hit
		int key = get_key(pts_hit_uv[p]);
	
		if (num == 0)
		{
			map[key] = P_max;
			continue;
		}

		if (map.find(key) == map.end())
		{
			map[key] = P_hit;
		}
		else
		{
			double m_old = map[key];
			double m_new = prob(m_old, P_hit);
			map[key] = m_new;
		}
	}

	// increase num of scans
	num++;
}

int SUBMAP::get_key(cv::Vec2i uv)
{
    return uv[0] * 100000 + uv[1];
}

cv::Vec2i SUBMAP::get_uv(int idx)
{
    int u = idx / 100000;
    int v = idx % 100000;
	return cv::Vec2i(u, v);
}

std::vector<cv::Vec2i> SUBMAP::xy_uv(std::vector<cv::Vec2d> &pts, cv::Vec3d _xi)
{
	std::vector<cv::Vec2i> pts_uv;
    for (size_t p = 0; p < pts.size(); p++)
	{
		cv::Vec2d res = transform(_xi, pts[p]);
		
        int u = std::round(res[0] / robot_config.robot_grid_size + ou);
        int v = std::round(res[1] / robot_config.robot_grid_size + ov);

		pts_uv.push_back(cv::Vec2i(u, v));
	}
	return pts_uv;
}

cv::Vec2i SUBMAP::xy_uv(cv::Vec2d _P, cv::Vec3d _xi)
{
	cv::Vec2d res = transform(_xi, _P);

    int u = std::round(res[0] / robot_config.robot_grid_size + ou);
    int v = std::round(res[1] / robot_config.robot_grid_size + ov);

	return cv::Vec2i(u, v);
}

cv::Vec2d SUBMAP::uv_xy(cv::Vec2i uv)
{
    double x = (uv[0] - ou) * robot_config.robot_grid_size;
    double y = (uv[1] - ov) * robot_config.robot_grid_size;
    return cv::Vec2d(x, y);
}

double SUBMAP::odds(double p)
{
	return p / (1.0 - p);
}

double SUBMAP::odds_inv(double odd)
{
	return odd/(odd+1.0);
}

double SUBMAP::clamp(double p, double min, double max)
{
	if (p > max)
	{
		p = max;
	}
	else if (p < min)
	{
		p = min;
	}
	return p;
}

double SUBMAP::prob(double m_old, double P)
{
	return clamp(odds_inv(odds(m_old)*odds(P)), P_min, P_max);
}

void SUBMAP::get_gradient(cv::Vec2d _P, cv::Vec3d _xi, double & gu, double & gv)
{
	cv::Vec2d res = transform(_xi, _P);

    int u = std::round(res[0] / robot_config.robot_grid_size + ou);
    int v = std::round(res[1] / robot_config.robot_grid_size + ov);

	double u0 = 0;
	double u1 = 0;
	double v0 = 0;
	double v1 = 0;

	if (map.find(get_key(cv::Vec2i(u - 1, v))) != map.end())
	{
		u0 = map[get_key(cv::Vec2i(u - 1, v))];
	}

	if (map.find(get_key(cv::Vec2i(u + 1, v))) != map.end())
	{
		u1 = map[get_key(cv::Vec2i(u + 1, v))];
	}
	
	if (map.find(get_key(cv::Vec2i(u, v - 1))) != map.end())
	{
		v0 = map[get_key(cv::Vec2i(u, v - 1))];
	}

	if (map.find(get_key(cv::Vec2i(u, v + 1))) != map.end())
	{
		v1 = map[get_key(cv::Vec2i(u, v + 1))];
	}

	gu = (u1 - u0) / 2;
	gv = (v1 - v0) / 2;
}

double SUBMAP::get_probability(cv::Vec2d _P, cv::Vec3d _xi)
{
	cv::Vec2d res = transform(_xi, _P);

    double u = res[0] / robot_config.robot_grid_size + ou;
    double v = res[1] / robot_config.robot_grid_size + ov;

	if (u < 0 || u >= w || v < 0 || v >= h)
	{
		return -1;
	}

	// get interpolated probability bilinear
	int u_n = (int)u;
	int v_n = (int)v;
	double u_r = u - u_n;
	double v_r = v - v_n;
	double val0 = 0;
	double val1 = 0;
	double val2 = 0;
	double val3 = 0;

	if (map.find(get_key(cv::Vec2i(u_n, v_n))) != map.end())
	{
		val0 = map[get_key(cv::Vec2i(u_n, v_n))];
	}

	if (map.find(get_key(cv::Vec2i(u_n + 1, v_n))) != map.end())
	{
		val1 = map[get_key(cv::Vec2i(u_n + 1, v_n))];
	}

	if (map.find(get_key(cv::Vec2i(u_n + 1, v_n + 1))) != map.end())
	{
		val2 = map[get_key(cv::Vec2i(u_n + 1, v_n + 1))];
	}

	if (map.find(get_key(cv::Vec2i(u_n, v_n + 1))) != map.end())
	{
		val3 = map[get_key(cv::Vec2i(u_n, v_n + 1))];
	}

	double val_u0 = lerp(val0, val1, u_r);
	double val_u1 = lerp(val3, val2, u_r);
	double val = lerp(val_u0, val_u1, v_r);
	return val;
}

double SUBMAP::lerp(double st, double ed, double t)
{
	return st + (ed - st) * t;
}

cv::Mat SUBMAP::get_map_gray()
{
	// build image
	cv::Mat img(h, w, CV_8U, cv::Scalar(0));
	for (auto it = map.begin(); it != map.end(); it++)
	{
		int idx = it->first;
		double val = it->second;
		cv::Vec2i uv = get_uv(idx);
		img.ptr<uchar>(uv[1])[uv[0]] = val * 255;
	}
	return img;
}

cv::Mat SUBMAP::get_map_real()
{
	// build image
	cv::Mat img(h, w, CV_64F, cv::Scalar(0));
	for (auto it = map.begin(); it != map.end(); it++)
	{
		int idx = it->first;
		double val = it->second;
		cv::Vec2i uv = get_uv(idx);
		img.ptr<double>(uv[1])[uv[0]] = val;
	}
	return img;
}

cv::Vec2d SUBMAP::transform(cv::Vec3d & xi, cv::Vec2d &vec)
{
	cv::Matx22d R;
	R(0, 0) = std::cos(xi[2]);
	R(0, 1) = -std::sin(xi[2]);
	R(1, 0) = std::sin(xi[2]);
	R(1, 1) = std::cos(xi[2]);

	cv::Vec2d t;
	t[0] = xi[0];
	t[1] = xi[1];

	cv::Vec2d res = R * vec + t;
	return res;
}
