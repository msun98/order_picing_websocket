#pragma once

// global values
#include "global_defines.h"

// Qt
#include <QObject>

// STL
#include <map>
#include <unordered_set>
#include <algorithm>

// OpenCV
#include <opencv2/opencv.hpp>

class SUBMAP : public QObject
{
	Q_OBJECT

public:
	SUBMAP();
	~SUBMAP();

    SUBMAP(const SUBMAP &p)
	{
		this->id = p.id;
		this->map = p.map;
		this->xi = p.xi;        
        this->dxi = p.dxi;
		this->w = p.w;
		this->h = p.h;
		this->ou = p.ou;
		this->ov = p.ov;
		this->num = p.num;
	}

	SUBMAP& operator=(const SUBMAP& p)
	{
		this->id = p.id;
		this->map = p.map;
		this->xi = p.xi;        
        this->dxi = p.dxi;
		this->w = p.w;
		this->h = p.h;
		this->ou = p.ou;
		this->ov = p.ov;
		this->num = p.num;
		return *this;
	}

	// grid
	std::map<int, double> map;
	
    // submap global pose
	cv::Vec3d xi;

    // delta pose
	cv::Vec3d dxi;
	   
	// variables
	int id = 0;
	int w = 100;
	int h = 100;
	int ou = 50;
	int ov = 50;
	int num = 0;

	// functions    
    void add_scan(std::vector<cv::Vec2d> &pts, cv::Vec3d _dxi);
    int get_key(cv::Vec2i uv);
    cv::Vec2i get_uv(int idx);
	std::vector<cv::Vec2i> xy_uv(std::vector<cv::Vec2d> &pts, cv::Vec3d _xi);
	cv::Vec2i xy_uv(cv::Vec2d _P, cv::Vec3d _xi);
    cv::Vec2d uv_xy(cv::Vec2i uv);
	
	double odds(double p);
	double odds_inv(double odd);
	double clamp(double p, double min, double max);
	double prob(double m_old, double P);
    void get_gradient(cv::Vec2d _P, cv::Vec3d _xi, double &gu, double &gv);
    double get_probability(cv::Vec2d _P, cv::Vec3d _xi);
	double lerp(double st, double ed, double t);

    cv::Mat get_map_gray(); // get grayscale map (8bit)
    cv::Mat get_map_real(); // 0 to 1 probabilistic gridmap (double)

	cv::Vec2d transform(cv::Vec3d &xi, cv::Vec2d &vec);
};
