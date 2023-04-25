#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "websocket.h"
#include <QMainWindow>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    websocket web;

    QTimer timer;

public slots:
    void showUI_msg(bool msg);

    void showUI_recive_msg(QString message);

    void showUI_send_msg(QString message);

private:

    Ui::MainWindow *ui;

};
#endif // MAINWINDOW_H
