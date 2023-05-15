#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "websocket.h"
#include <QMainWindow>
#include "ipc.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    // ipc
//    IPC ipc;
//    QSharedMemory *ipc;

//    uint32_t last_status_tick = 0;

    websocket web;


public slots:
    void showUI_msg(bool msg);

    void showUI_recive_msg(QString message);

    void showUI_send_msg(QString message);

    void check_robot_connected(bool connected);

//    void timerLoop();

//signals:
//    void StatusSignal(float x, float y, float theta,float charge, float power);

private:

    Ui::MainWindow *ui;

};
#endif // MAINWINDOW_H
