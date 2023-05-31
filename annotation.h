#ifndef ANNOTATION_H
#define ANNOTATION_H

// defines
#include "global_defines.h"

// qt
#include <QObject>

class ANNOTATION : public QObject
{
    Q_OBJECT
public:
    explicit ANNOTATION(QObject *parent = nullptr);

    void set_map(cv::Mat& map);

private:
    cv::Mat raw_map;

signals:

};

#endif // ANNOTATION_H
