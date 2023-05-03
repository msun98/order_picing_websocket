#ifndef WEBSOCKET_H
#define WEBSOCKET_H
#include <QWebSocketServer>
#include <QWebSocket>
#include <QIODevice>

#include <QJsonObject>
#include <QJsonDocument>
#include <QJsonArray>
#include <QTimer>
#include <QtCore/QDebug>
#include <QDir>
#include <QSettings>
#include <cmath>

// opencv
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/types_c.h>

class websocket : public QObject
{
    Q_OBJECT
public:
    ~websocket();
    explicit websocket(QObject *parent = nullptr);

    bool msg = false;

    QString uuid;

    cv::Mat img, src;//디버깅용 이미지 show하기 위함.

    QByteArray fileData_byte;//유진로봇에서 요청한 내용 보내기 위함.

    QString map_id;

    QFile* file;//file read할 때 사용하기 위함.

    QString map_config_path;

    QString config_path;

signals:
    void msgSignal(bool msg);

    void msgReciveSignal(QString message);

    void msgSendSignal(QString message);



public slots:

    void open();

    void onTimeout();

    void onNewConnection();

    void onClosed();

    void onTextMessageReceived(QString message);

    void onBinaryMessageReceived(QByteArray message);

    void MissionCheck(QString uuid);

    void onDisconnected();

    void send_img_package(QString map_config_path,int image_file_size,QString signature,QString fileName);

private:

    QTimer  *timer;

    QWebSocketServer *server;

    QList<QWebSocket *> clients;

    void sendNotice(QWebSocket *client_socket);

    void sendCommandAck(QWebSocket *client_socket, QString result, QJsonObject error_info, QString uuid);

    void sendCommandResult(QWebSocket *client_socket, QString result, QJsonObject error_info, QString uuid);

    QJsonObject data(QString robot_manufacture, QString action,QString robot_type, QString map_id, QString map_alias);

    void sendCommandResult(QWebSocket *client_socket, QString action, QString result, QJsonObject data, QJsonObject error_info, QString uuid);



    void sendAck(QString uuid);
};



#endif // WEBSOCKET_H
