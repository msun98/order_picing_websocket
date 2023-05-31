#include "astar.h"

ASTAR::ASTAR()
{

}

std::vector<cv::Vec2i> ASTAR::path_finding(cv::Mat map, cv::Vec2i st, cv::Vec2i ed, double scale)
{
    // no need to path planning in this case
    if(st == ed)
    {
        printf("[ASTAR] st, ed same location\n");
        std::vector<cv::Vec2i> res;
        res.push_back(st);
        res.push_back(ed);
        return res;
    }

    // resize cost map
    cv::Vec2i _st;
    cv::Vec2i _ed;
    cv::Mat _map;
    if(scale == 1.0)
    {
        _st = st;
        _ed = ed;
        _map = map.clone();
    }
    else
    {
        // scaling st ed
        _st[0] = st[0]*scale;
        _st[1] = st[1]*scale;

        _ed[0] = ed[0]*scale;
        _ed[1] = ed[1]*scale;

        // scaling cost map
        _map = cv::Mat(map.rows*scale, map.cols*scale, CV_8U, cv::Scalar::all(255));
        for(int i = 0; i < map.rows; i++)
        {
            for(int j = 0; j < map.cols; j++)
            {
                double val = map.ptr<uchar>(i)[j];

                int u = std::round(j*scale);
                int v = std::round(i*scale);
                if(u < 0 || u >= _map.cols || v < 0 || v >= _map.rows)
                {
                    continue;
                }

                if(val < _map.ptr<uchar>(v)[u])
                {
                    _map.ptr<uchar>(v)[u] = val;
                }
            }
        }
    }

    // make open set and close set
    std::vector<NODE*> open_set;    
    cv::Mat close_set(_map.rows, _map.cols, CV_8U, cv::Scalar(0));

    // create start node and end node
    NODE *ed_node = new NODE();
    ed_node->pos = _ed;

    NODE *st_node = new NODE();
    st_node->pos = _st;
    st_node->g = 0;
    st_node->h = cv::norm(ed_node->pos-st_node->pos);
    st_node->f = st_node->g + st_node->h;
    open_set.push_back(st_node);

    // result storage
    while(open_set.size() > 0)
    {
        // get the current node
        int cur_node_idx = 0;
        NODE *cur_node = open_set.front();
        for(size_t p = 0; p < open_set.size(); p++)
        {
            if(open_set[p]->f < cur_node->f)
            {
                cur_node = open_set[p];
                cur_node_idx = p;
            }
        }

        // pop current off open list, add to closed list
        open_set.erase(open_set.begin()+cur_node_idx);        
        close_set.ptr<uchar>(cur_node->pos[1])[cur_node->pos[0]] = 255;

        // found the goal
        if(cur_node->pos == ed_node->pos)
        {
            std::vector<cv::Vec2i> inverse_path;

            NODE* _cur_node = cur_node;
            while(_cur_node != NULL)
            {
                inverse_path.push_back(_cur_node->pos);
                _cur_node = _cur_node->parent;
            }

            std::vector<cv::Vec2i> result_path;
            for(int p = inverse_path.size()-1; p >= 0; p--)
            {
                cv::Vec2i pos;
                pos[0] = inverse_path[p][0] * (1.0/scale);
                pos[1] = inverse_path[p][1] * (1.0/scale);
                result_path.push_back(pos);
            }

            printf("[ASTAR] path found\n");
            return result_path;
        }

        // append child nodes
        std::vector<cv::Vec2i> around_pts;

        int search_range = 2;
        for(int my = -search_range; my <= search_range; my++)
        {
            for(int mx = -search_range; mx <=search_range; mx++)
            {
                if(mx == 0 && my == 0)
                {
                    continue;
                }

                int u = cur_node->pos[0] + mx;
                int v = cur_node->pos[1] + my;
                if(u < 0 || u >= _map.cols || v < 0 || v >= _map.rows)
                {
                    continue;
                }

                if(_map.ptr<uchar>(v)[u] == 255)
                {
                    continue;
                }

                if(close_set.ptr<uchar>(v)[u] == 255)
                {
                    continue;
                }

                around_pts.push_back(cv::Vec2i(u,v));
            }
        }

        for(size_t p = 0; p < around_pts.size(); p++)
        {
            // calc heuristics
            cv::Vec2i child_pos(around_pts[p][0], around_pts[p][1]);
            double child_g = cur_node->g + cv::norm(child_pos - cur_node->pos) + 2.0*_map.ptr<uchar>(child_pos[1])[child_pos[0]];
            double child_h = cv::norm(child_pos - ed_node->pos);
            double child_f = child_g + child_h;

            // check close set
            if(close_set.ptr<uchar>(child_pos[1])[child_pos[0]] == 255)
            {
                continue;
            }

            // check open set
            bool is_open_set = false;
            NODE* open_node = NULL;
            for(size_t p = 0; p < open_set.size(); p++)
            {
                if(open_set[p]->pos == child_pos)
                {
                    is_open_set = true;
                    open_node = open_set[p];
                    break;
                }
            }

            if(is_open_set)
            {
                if(child_g < open_node->g)
                {
                    open_node->parent = cur_node;
                    open_node->g = child_g;
                    open_node->h = child_h;
                    open_node->f = child_f;
                    continue;
                }
                else
                {
                    continue;
                }
            }

            // add new child to open set
            NODE *node = new NODE();
            node->parent = cur_node;
            node->pos = child_pos;
            node->g = child_g;
            node->h = child_h;
            node->f = child_f;
            open_set.push_back(node);
        }
    }

    printf("[ASTAR] path finding failed\n");
    return std::vector<cv::Vec2i>();
}

std::vector<cv::Vec2i> ASTAR::path_simplify(cv::Mat map, std::vector<cv::Vec2i>& raw_path)
{
    int n = raw_path.size();
    if(n < 3)
    {
        return raw_path;
    }

    std::vector<int> mask(n, 1);
    for(int p = 1; p < n-1; p+=2)
    {
        int i0 = p-1;
        int i1 = p+1;

        // check los
        std::vector<cv::Vec2i> line = line_iterator(raw_path[i0], raw_path[i1]);

        bool is_los = true;
        for(size_t q = 0; q < line.size(); q++)
        {
            int u = line[q][0];
            int v = line[q][1];
            if(u < 0 || u >= map.cols || v < 0 || v >= map.rows)
            {
                continue;
            }

            uchar val = map.ptr<uchar>(v)[u];
            if(val == 255)
            {
                is_los = false;
                break;
            }
        }

        if(is_los)
        {
            mask[p] = 0;
        }
    }

    std::vector<cv::Vec2i> res;
    for(size_t p = 0; p < raw_path.size(); p++)
    {
        if(mask[p] == 0)
        {
            continue;
        }
        res.push_back(raw_path[p]);
    }
    return res;
}

std::vector<cv::Vec2i> ASTAR::line_iterator(cv::Vec2i pt0, cv::Vec2i pt1)
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

std::vector<cv::Vec2i> ASTAR::circle_iterator(cv::Vec2i pt, int r)
{
    int x = r;
    int y = 0;
    int error = 3 - 2*r;

    std::vector<cv::Vec2i> res;
    while (x >= y)
    {
        res.push_back(cv::Vec2i(x, y) + pt);
        res.push_back(cv::Vec2i(x, -y) + pt);
        res.push_back(cv::Vec2i(-x, y) + pt);
        res.push_back(cv::Vec2i(-x, -y) + pt);

        res.push_back(cv::Vec2i(y, x) + pt);
        res.push_back(cv::Vec2i(y, -x) + pt);
        res.push_back(cv::Vec2i(-y, x) + pt);
        res.push_back(cv::Vec2i(-y, -x) + pt);

        if (error > 0)
        {
            error -= 4 * (--x);
        }
        error += 4 * (++y) + 2;
    }

    return res;
}

std::vector<cv::Vec2i> ASTAR::filled_circle_iterator(cv::Vec2i pt, int r)
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
