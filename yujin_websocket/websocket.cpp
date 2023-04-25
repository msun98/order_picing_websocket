#include "websocket.h"
#include <filesystem>
//little endian
typedef union{
    int     data_i;
    char    data_c[4];
}U_INT_CHAR;

typedef union{
    short   data_s;
    char    data_c[2];
}U_SHORT_CHAR;



websocket::websocket(QObject *parent) : QObject(parent)
{

}

websocket::~websocket()
{
    qDebug()<<"웹소켓 프로그램 종료";
}


void websocket::open()
{
    server = new QWebSocketServer("rb_websocket", QWebSocketServer::NonSecureMode, this);
    if(server->listen(QHostAddress::Any, 38081)) //서버가 들어오는 연결을 수신하기 위해 listen을 호출
    {
        connect(server, SIGNAL(newConnection()), this, SLOT(onNewConnection()));
    }

    timer = new QTimer(this);
//    connect(timer, SIGNAL(timeout()), this, SLOT(onTimeout()));
//    timer->start(100);
}

void websocket::onNewConnection(){
    QWebSocket *pSocket = server->nextPendingConnection();

    connect(pSocket, SIGNAL(textMessageReceived(QString)), this, SLOT(onTextMessageReceived(QString)));
//    connect(pSocket, SIGNAL(binaryMessageReceived(QByteArray)), this, SLOT(onBinaryMessageReceived(QByteArray)));
    connect(pSocket, SIGNAL(disconnected()), this, SLOT(onDisconnected()));
//    connect(pMP, SIGNAL(UpdateUI()), this, SLOT(MissionCheck(QString uuid)));

    clients << pSocket;

    msg = true;
    emit msgSignal(msg);

    qDebug()<<"상대방이 접속하였습니다.";
}

void websocket::onDisconnected(){
    qDebug()<<"상대방의 접속이 끊겼습니다.";
    msg = false;
    emit msgSignal(msg);

    QWebSocket *pClient = qobject_cast<QWebSocket *>(sender());
    if(pClient){
        clients.removeAll(pClient);
        pClient->deleteLater();
    }
}

void websocket::onClosed()
{

}


void websocket::onTimeout()
{
    static int cnt = 0;
    cnt++;

    if(clients.size() > 0)
    {
        if(cnt%5 == 0)
        {
            for(int i=0; i<clients.size(); i++)
            {
                QWebSocket *pSocket = clients[i];
                sendNotice(pSocket);
            }
        }
    }
}

void websocket::onBinaryMessageReceived(QByteArray message){
    QWebSocket *pClient = qobject_cast<QWebSocket *>(sender());
    if(pClient){
        pClient->sendBinaryMessage(message);
    }
}

void websocket::onTextMessageReceived(QString message) //comand msg
{
    QWebSocket *pClient = qobject_cast<QWebSocket *>(sender());
    emit msgReciveSignal(message);// ui를 통해 메시지를 확인하기 위함.

    if(pClient){
        QJsonObject json;
        QJsonDocument doc_json = QJsonDocument::fromJson(message.toUtf8());
        json = doc_json.object();

        if(json["msg_type"] == "command"){
            // Command Msg
            QString entry = json["entry"].toString();
            QString action = json["do"].toString();
            QJsonObject params = json["params"].toObject();
            uuid = json["uuid"].toString();
            QJsonObject error_info;
            sendAck(uuid);


            if(action == "move")
            {
                //4.1 Move
                QJsonObject tempDest = params["dest"].toObject();
//                QString mm = QJsonDocument(tempDest).toJson(QJsonDocument::Compact);
//                qDebug()<<mm;

                double x = tempDest["x"].toDouble();
                double y = tempDest["y"].toDouble();
                double theta = tempDest["theta"].toDouble();

                qDebug()<<"x :"<<x<<",y :"<<y<<",theta :"<<theta;
//                pMP->MoveOmron(x,y,theta);
            }

            else if(action == "dock")
            {
                //4.2 Dock

                QString marker_id = params["marker_id"].toString();
                QString direction = params["direction"].toString();

                qDebug()<<"marker_id : "<<marker_id<<"direction : "<<direction;

//                pMP->MoveOmron(x,y,theta);
            }

            else if(action == "pause")
            {
                //4.4 Pause
                //robot에 Pause 명령줘야함.
                qDebug()<<"Pause";

//                qDebug()<<"stop";
//                pClient->sendTextMessage("stop");
//                pMP->StopOmron();
            }

            else if(action == "resume")
            {
                //4.5 Resume
                qDebug()<<"Resume";
                //robot에 stop 명령줘야함.
//                pClient->sendTextMessage("stop");
//                pMP->StopOmron();
            }

            else if(action == "stop")
            {
                //4.6 Stop
                qDebug()<<"stop";
                //robot에 stop 명령줘야함.
//                pClient->sendTextMessage("stop");
//                pMP->StopOmron();
            }

            else if(action == "get_map_info_list")
            {
                //4.8 Get Map Info List
////                QJsonObject json_data;//map 의 개수가 증가함에 따라 json object 의 개수가 늘어야함.
                QJsonObject json_out;
                QJsonArray json_arr;
//                QJsonObject json_out;
                json_out["msg_type"] = "cmd_result";
                json_out["result"] = "success";
                json_out["error_info"] = "null";

                ////////////map directory 내부에 있는 파일 list 반환////////////
                /// \brief dir
//                QDir::homePath()+"maps/map_4/";
                QDir dir("/home/rainbow/maps/map_4");
//                QDir dir("maps/map_4/");
                QFileInfoList list = dir.entryInfoList(QDir::Files, QDir::NoSort);

                for (int i = 0; i < list.size(); ++i)
                {
                   QJsonObject json_data;
                   QFileInfo fInfo = list.at(i);
                   QString fPath = fInfo.absoluteFilePath();

                   QString filelist = fPath.section("/home/rainbow/maps/map_4", 1);

//                   bool ret = filelist.contains(".png", Qt::CaseInsensitive);

//                   qDebug()<<ret;

                   if (filelist.contains(".png", Qt::CaseInsensitive)==true)
                   {

//                       qDebug()<<"yes";
                       json_data["data"] = data("OMRON","get_map_info_list","LD250", "MIR", filelist);
//                       QJsonObject data = json_data["data"];
                       QJsonObject data = json_data["data"].toObject();
                       json_arr.insert(i,data);//디렉토리 내부에 있는 파일 중 1번 out(omron에 들어가는 파일의 개수가 많아지면 0이 다른 숫자로 바뀌어야함.)
                       json_out["data"] = json_arr;
                   }

                   else{
                       qDebug()<<"no";
                   }



                }

                 if(!error_info.empty())
                 {
                     json_out["error_info"] = error_info;
                 }
                 QJsonDocument doc_json(json_out);
                 QString str_json(doc_json.toJson(QJsonDocument::Indented));
//                QJsonObject data = json_data["data"].toObject();
                pClient->sendTextMessage(str_json);
            }


            else if(action == "get_map_info")
            {
                //4.9 Get Map Info
                QJsonObject json;
                QJsonObject json_data;
                json["msg_type"] = "cmd_result"; //만약 명령 수행시 성공했다면 result -> success (omron 이동 하면 success!)
                json["result"] = "success";

                if(!error_info.empty())
                {
                    json["error_info"] = error_info;
                }
                json["uuid"] = uuid;


            //    QJsonArray json_arr;

                json_data["origin_px"] = 5;
                json_data["origin_py"] = 5;
                json_data["scale_m2px"] = 20.0;
                json_data["filpped"] = "false";
//                qDebug()<<params["map_id"].toString();

                if (params["map_id"].toString() == "")
                {
                    json["error_info"] = "null";
                    json_data["map_id"] = "original.png";
                    json_data["map_alias"] = 20.0;
                    json_data["width_gm"] = 500;
                    json_data["height_gm"] = 500;
                }

                else
                {
                    //나중에 유진에서 요청하는 맵 이름으로 변경하여 내보내기
                    json_data["map_id"] = params["map_id"].toString();;
                    json_data["map_alias"] = 20.0;
                    json_data["width_gm"] = 500;
                    json_data["height_gm"] = 500;
                }
                json["data"] = json_data;


                QJsonDocument doc_json(json);
                QString str_json(doc_json.toJson(QJsonDocument::Indented));
                pClient->sendTextMessage(str_json);

            }

            else if(action == "get_map_data")
            {
                //4.10 Get Map Data
//                qDebug()<<params["map_id"];
                QString id = params["map_id"].toString();//server 에서 요청한 map 의 id 파싱
                QJsonObject json;
                QJsonObject json_data;
                QString fileName;

                int image_file_size;
                json_data["data_type"] = params["data_type"];
                json["msg_type"] = "cmd_result";
                json["result"] = "success";
                json["error_info"] = "null";
//                json_data["map_id"] = params["map_id"];

                json["uuid"] = uuid;
                /*cv::Mat img;
                QByteArray fileData_byte ;

                if (id == "")
                {

                    id = "original.png";//현재 사용하고 있는 map
                    fileName = id;

                    QFile* file = new QFile("map/"+id);
                    file->open(QFile::ReadOnly);
                    fileData_byte=file->readAll();
//                    fileData_byte = file->open(QFile::ReadOnly | QFile::Text);
                    image_file_size = fileData_byte.size();


//                    QString imageFileSize = QString::number(image_file_size);
//                    reverse(imageFileSize.begin(), imageFileSize.end()); //글자 위치 반전시킬 때

//                    json_data["map_id"] = map_id;
                    json_data["filename"] = id;
                    json_data["filesize"] = image_file_size;
                    json_data["flipped"] = false;
                    img = cv::imread("map/"+id.toStdString(),cv::IMREAD_GRAYSCALE); // 이미지 읽기

//                    mapImg=img;
//                    imshow("지금 사용하고 있는 맵",img);*/
                    //바이너리로 변경하여 유진로봇 api에 쏴주어야함.
                }

                else
                {
                    qDebug()<<"hi";
                    //나중에 유진에서 요청하는 맵 이름으로 변경하여 내보내기
                    /*fileName=id;
                    id = "map/"+id;
                    QFile* file = new QFile(id);
                    file->open(QFile::ReadOnly);
                    fileData_byte=file->readAll();
//                    fileData_byte = file.toUtf8();
//                    fileData_byte = file->open(QFile::ReadOnly | QFile::Text);
                    image_file_size = fileData_byte.size();
//                    QString imageFileSize = QString::number(image_file_size);
//                    reverse(imageFileSize.begin(), imageFileSize.end());

//                    cout<<size1<<endl;


                    json_data["map_id"] = fileName;
                    json_data["filename"] = fileName;
                    json_data["filesize"] = image_file_size;
                    json_data["flipped"] = false;
                    img = cv::imread(id.toStdString(),cv::IMREAD_GRAYSCALE); // 이미지 읽기－￣

                }

                imshow("맵",img);
                json["data"] = json_data;

                QJsonDocument doc_json(json);
                QString str_json(doc_json.toJson(QJsonDocument::Indented));
//                cout<<img<<endl;
///////////////////////////////////////////////////cmd/////////////////////////////////////////////////////
                QString prifix = "FILE_DATA";
                QByteArray prifix_byte = prifix.toUtf8();

                QString signature = json_data["data_type"].toString();
                if (signature == "map_image_png")
                {
                    signature = "MAP_IMAGE_PNG___";
                }
                else
                {
                    signature = "MAP_PACKAGE_COMP";
                }



                int header_size = signature.size()+fileName.size();
                U_SHORT_CHAR u_headersize;
                u_headersize.data_s = header_size;
                prifix_byte.append(u_headersize.data_c[0]);
                prifix_byte.append(u_headersize.data_c[1]);

                U_INT_CHAR u_filesize;
                u_filesize.data_i = image_file_size;

                QByteArray signature_byte = signature.toUtf8();
                prifix_byte.append(signature_byte);

                QByteArray fileName_byte = fileName.toUtf8();
                prifix_byte.append(fileName_byte);
                prifix_byte.append(u_filesize.data_c[0]);
                prifix_byte.append(u_filesize.data_c[1]);
                prifix_byte.append(u_filesize.data_c[2]);
                prifix_byte.append(u_filesize.data_c[3]);

                prifix_byte.append(fileData_byte);

                QFile file("prifix_byte.bin");

                file.open(QIODevice::WriteOnly);
                file.write(fileData_byte);
                file.close();

                pClient->sendTextMessage(str_json);
                pClient->sendBinaryMessage(prifix_byte);*/
            }

//            else if(action == "set_map_data")
//            {
//                //4.11 Set Map Data
//            }


//            QString uuid = json["uuid"].toString();￣

//            QJsonObject error_info;
//            sendCommandAck(pClient, "success", error_info, uuid);

//            pClient->sendTextMessage("end");

        }
    }
}

QJsonObject websocket::data(QString robot_manufacture, QString action,QString robot_type, QString map_id, QString map_alias){

    QJsonObject json_data_map;
//    QJsonArray json_arr;
    QJsonObject json_out;

    if (action == "get_map_info_list")
    {
        json_data_map["robot_manufacturer"] = robot_manufacture;
        json_data_map["robot_type"] = robot_type;
        json_data_map["map_id"] = map_id;
        json_data_map["map_alias"] = map_alias;

//        QJsonArray json_arr;
//        json_arr.insert(0,json_data_map);//디렉토리 내부에 있는 파일 중 1번 out(omron에 들어가는 파일의 개수가 많아지면 0이 다른 숫자로 바뀌어야함.)
//        //    json_out["data"] = json_arr;
        json_out = json_data_map;
    }


    return json_out;
}

void websocket::MissionCheck(QString uuid){
//    QJsonObject json;
    QJsonObject error_info;
//    QString uuid = uuid;
    for(int i=0; i<clients.size(); i++)
    {
        QWebSocket *pSocket = clients[i];

//        if (pMP->check == 1)
//        {
//            sendCommandResult(pSocket, "success", error_info, uuid);
//            qDebug()<<"success";
//        }

//        else if (pMP->check == 0)
//        {
//            sendCommandResult(pSocket, "Failed", error_info, uuid);
//            qDebug()<<"Failed";
//        }
    }
}

void websocket::sendNotice(QWebSocket *client_socket){
    QJsonObject json;
    json["msg_type"] = "notice";
    json["robot_state"] = "ready";
    json["navi_mode"] = "navigate";

//    QJsonObject json_robot;
//    json_robot["x"] = QString::number(networkThread->x);
//    json_robot["y"] = QString::number(networkThread->y);
//    json_robot["theta"] = QString::number(networkThread->heading);
//    json["robot_pose"] = json_robot;

    QJsonObject json_map;
    json_map["map_id"] = "test";
    json_map["map_alias"] = "1F";
    json["map"] = json_map;

//    QJsonObject json_battery;
//    json_battery["level"] = QString::number(networkThread->baterry);
//    json_battery["in_charging"] = false;
//    json["battery"] = json_battery;

    QJsonDocument doc_json(json);
    //QString str_json(doc_json.toJson(QJsonDocument::Compact));
    QString str_json(doc_json.toJson(QJsonDocument::Indented));
//    client_socket->sendTextMessage("notice");
    client_socket->sendTextMessage(str_json);
}

void websocket::sendAck(QString uuid)
{

    QWebSocket *pClient = qobject_cast<QWebSocket *>(sender());

    QJsonObject json;
    json["msg_type"] = "ack";
    json["result"] = "success";
    json["uuid"] = uuid;

    QJsonDocument doc_json(json);
    QString str_json(doc_json.toJson(QJsonDocument::Indented)); //QJsonDocument to QString

    emit msgSendSignal(str_json);
    pClient->sendTextMessage(str_json);

//    client_socket->sendTextMessage(json);
}
