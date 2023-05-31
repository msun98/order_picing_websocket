#ifndef ASTAR_H
#define ASTAR_H

#include "global_defines.h"

class ASTAR
{

public:
    struct NODE
    {
        NODE* parent = NULL;
        cv::Vec2i pos;
        double g = 0;
        double h = 0;
        double f = 0;

        NODE()
        {
        }
        NODE(const NODE& p)
        {
            parent = p.parent;
            pos = p.pos;
            g = p.g;
            h = p.h;
            f = p.f;
        }
    };

public:
    ASTAR();

    std::vector<cv::Vec2i> path_finding(cv::Mat map, cv::Vec2i st, cv::Vec2i ed, double scale);
    std::vector<cv::Vec2i> path_simplify(cv::Mat map, std::vector<cv::Vec2i>& raw_path);
    std::vector<cv::Vec2i> line_iterator(cv::Vec2i pt0, cv::Vec2i pt1);
    std::vector<cv::Vec2i> circle_iterator(cv::Vec2i pt, int r);
    std::vector<cv::Vec2i> filled_circle_iterator(cv::Vec2i pt, int r);
};

#endif // ASTAR_H
