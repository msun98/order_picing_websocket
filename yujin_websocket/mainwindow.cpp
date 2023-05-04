#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <filesystem>


/*
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent)
{
    ui = new Ui::MainWindow();
    ui->setupUi(this);
}
--------------------------------------------
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)//new Ui::MainWindow는 mainwindow 클래스에서 호출되지 않고 Ui::MainWindow에 선언된 setupUI메소드를 사용함.)

같은 내용임.
*/

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    web.open();

//    connect(&web, SIGNAL(onNewConnection()), this, SLOT(showUI_msg()));
    connect(&web, SIGNAL(msgSignal(bool)), this, SLOT(showUI_msg(bool)));
    connect(&web, SIGNAL(msgReciveSignal(QString)), this, SLOT(showUI_recive_msg(QString)));
    connect(&web, SIGNAL(msgSendSignal(QString)), this, SLOT(showUI_send_msg(QString)));

    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::showUI_msg(bool msg)
{
    if (msg==true)
    {
        ui->te_label->setText(QString("New Client Connected.."));
    }

    else
    {
        ui->te_label->setText(QString("New Client Disconnect.."));
    }
}

void MainWindow::showUI_recive_msg(QString message)
{
     ui->te_msg->append(QString(message));
     ui->te_msg->append("");

//     te_send_msg

}

void MainWindow::showUI_send_msg(QString message)
{
    ui->te_send_msg->setText(QString(message));
//     ui->te_msg->setText("");
//     ui->te_send_msg->append(QString(message));
//     ui->te_msg->append("");

}


