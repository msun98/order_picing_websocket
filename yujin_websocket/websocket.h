#ifndef WEBSOCKET_H
#define WEBSOCKET_H
#include <QWebSocketServer>
#include <QWebSocket>

#include <QJsonObject>
#include <QJsonDocument>
#include <QJsonArray>
#include <QTimer>
#include <QtCore/QDebug>
#include <QDir>

class websocket : public QObject
{
    Q_OBJECT
public:
    ~websocket();
    explicit websocket(QObject *parent = nullptr);

    bool msg = false;

    QString uuid;

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
