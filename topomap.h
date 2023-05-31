#ifndef TOPOMAP_H
#define TOPOMAP_H

#include "global_defines.h"

#include <QObject>

#include "unimap.h"

class TOPOMAP : public QObject
{
    Q_OBJECT
public:
    explicit TOPOMAP(QObject *parent = nullptr);
    ~TOPOMAP();

    void init(UNIMAP *_unimap);
    void build();


private:
    UNIMAP *unimap = NULL;


signals:

};

#endif // TOPOMAP_H
