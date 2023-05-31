#include "topomap.h"

TOPOMAP::TOPOMAP(QObject *parent) : QObject(parent)
{

}

TOPOMAP::~TOPOMAP()
{

}

void TOPOMAP::init(UNIMAP *_unimap)
{
    unimap = _unimap;
}

void TOPOMAP::build()
{
    // get data
    cv::Mat cost_map = unimap->get_map_cost0();
    std::vector<cv::Vec2d> locations = unimap->get_locations();

    // make vertex
    double step = (2*robot_config.robot_radius) + 0.1;
    double range = robot_config.robot_map_size * robot_config.robot_grid_size;

    int cnt = 0;
    std::vector<cv::Vec2d> vertices;
    for(double y = -range/2; y <= range/2; y += step)
    {
        for(double x = -range/2; x <= range/2; x += step)
        {
            double offset_x = 0;
            if(cnt % 2 == 0)
            {
                offset_x = step/2;
            }

            cv::Vec2d P(x + offset_x, y);

            bool is_fine = true;
            for(size_t p = 0; p < locations.size(); p++)
            {
                double d = cv::norm(P-locations[p]);
                if(d <= step)
                {
                    is_fine = false;
                    break;
                }
            }

            if(is_fine)
            {
                vertices.push_back(P);
            }
        }
        cnt++;
    }

    //vertices.insert(vertices.end(), locations.begin(), locations.end());

    cv::Mat vertex_map_plot = cost_map.clone();
    std::vector<cv::Vec2i> vertices_uv;
    for(size_t p = 0; p < vertices.size(); p++)
    {
        cv::Vec2i uv = unimap->xy_uv(vertices[p]);

        int u = uv[0];
        int v = uv[1];
        if(u < 0 || u >= cost_map.cols || v < 0 || v >= cost_map.rows)
        {
            continue;
        }

        if(cost_map.ptr<uchar>(v)[u] == 255)
        {
            continue;
        }

        vertices_uv.push_back(uv);

        cv::circle(vertex_map_plot, cv::Point(u,v), robot_config.robot_radius/robot_config.robot_grid_size, cv::Scalar(0), 1);
    }

    for(size_t p = 0; p < locations.size(); p++)
    {
        cv::Vec2i uv = unimap->xy_uv(locations[p]);

        int u = uv[0];
        int v = uv[1];
        if(u < 0 || u >= cost_map.cols || v < 0 || v >= cost_map.rows)
        {
            continue;
        }

        if(cost_map.ptr<uchar>(v)[u] == 255)
        {
            continue;
        }

        locations.push_back(uv);

        cv::circle(vertex_map_plot, cv::Point(u,v), robot_config.robot_radius/robot_config.robot_grid_size, cv::Scalar(0), -1);
    }


    //cv::imshow("vertex_map_plot", vertex_map_plot);


    int aa = 0;

}
