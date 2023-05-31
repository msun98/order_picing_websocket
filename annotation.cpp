#include "annotation.h"

ANNOTATION::ANNOTATION(QObject *parent) : QObject(parent)
{

}

void ANNOTATION::set_map(cv::Mat& map)
{
    raw_map = map.clone();
}
