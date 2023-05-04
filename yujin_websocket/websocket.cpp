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
    if(server->listen(QHostAddress::Any, 1111)) //서버가 들어오는 연결을 수신하기 위해 listen을 호출
    {
        connect(server, SIGNAL(newConnection()), this, SLOT(onNewConnection()));
    }

    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(onTimeout()));
    timer->start(100);
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

    if(pClient)
    {
        QJsonObject json;
        QJsonDocument doc_json = QJsonDocument::fromJson(message.toUtf8());
        json = doc_json.object();

        if(json["msg_type"] == "command")
        {
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

                QJsonObject json_out;
                QJsonArray json_arr;
                //                QJsonObject json_out;
                json_out["msg_type"] = "cmd_result";
                json_out["result"] = "success";
                json_out["error_info"] = QJsonValue::Null;

                ////////////map directory 내부에 있는 폴더 list 반환////////////

                QDir dir("/home/rainbow/maps/");
                foreach(QFileInfo item, dir.entryInfoList(QDir::NoDotAndDotDot | QDir::AllEntries) )
                {
                    if(item.isDir())
                    {

                        QJsonObject json_data;
                        QString filelist = item.baseName();
                        qDebug()<<filelist;
                        //                        qDebug() << "Dir: " << filelist; // 폴더 출력
                        json_data["data"] = data("Rainbow","get_map_info_list","RB", "RB", filelist);
                        QJsonObject data = json_data["data"].toObject();

                        //                        qDebug()<<"data : "<<data;
                        json_arr.insert(0,data);//대괄호 조건에 맞추기 위함.
                        json_out["data"] = json_arr;
                        QString map_info = QJsonDocument(json_out).toJson(QJsonDocument::Indented);//보낸 내용 확인용
                        emit msgSendSignal(map_info);
                        pClient->sendTextMessage(map_info);
                    }
                    //                    else if(item.isFile())
                    //                    {
                    //                        qDebug() << "File: " << item.absoluteFilePath(); // 파일 출력
                    //                    }


                    /*              파일 열어서 확인할 때 사용.
//                QDir dir("/home/rainbow/maps/map_4");
//                QFileInfoList list = dir.entryInfoList(QDir::Files, QDir::NoSort);

//                qDebug()<<dir;

//                for (int i = 0; i < list.size(); ++i)
//                {
//                   QJsonObject json_data;
//                   QFileInfo fInfo = list.at(i);
//                   QString fPath = fInfo.absoluteFilePath();

//                   QString filelist = fPath.section("/home/rainbow/maps", 0);

//                   qDebug()<<"filelist : "<<filelist;
//                   bool ret = filelist.contains(".png", Qt::CaseInsensitive); //png 인 것들만 이름보냄.
////                   if (ret==true)
////                   {
////                       qDebug()<<"filelist : "<<filelist;
//                       json_data["data"] = data("Rainbow","get_map_info_list","RB", "RB", filelist);
//                       QJsonObject data = json_data["data"].toObject();

//                        qDebug()<<"data : "<<data;
//                       json_arr.insert(0,data);//대괄호 조건에 맞추기 위함.
//                       json_out["data"] = json_arr;
//                       QString map_info = QJsonDocument(json_out).toJson(QJsonDocument::Indented);//보낸 내용 확인용
//                       emit msgSendSignal(map_info);
////                   }
/// */
                }
            }

            else if(action == "get_map_info")
            {
                //4.9 Get Map Info
                //                QJsonObject json;
                QJsonObject json_data;
                QJsonObject json_out;
                QString map_id;

                json_out["msg_type"] = "cmd_result";
                json_out["result"] = "success";
                json_out["error_info"] = QJsonValue::Null;

                ////////////map directory 내부에 있는 폴더 list 반환////////////

                if (params["map_id"].toString() == "")
                {
                    //for get map name
                    QString config_path = QDir::homePath()+"/robot_config.ini";
                    QFileInfo config_info(config_path);
                    if(config_info.exists() && config_info.isFile())
                    {
                        QSettings settings(config_path, QSettings::IniFormat);
                        map_id = settings.value("FLOOR/map_name").toString();
                        json_data["map_id"] = map_id;
                    }

                    //for get map info
                    map_config_path = QDir::homePath()+"/maps/"+map_id+"/map_meta.ini";
                    qDebug()<<map_config_path;

                    QFileInfo map_config_info(map_config_path);
                    if(map_config_info.exists() && map_config_info.isFile())
                    {
                        QSettings map_settings(map_config_path, QSettings::IniFormat);
                        //                        map_id = settings.value("FLOOR/map_name").toString();

                        json_data["map_alias"] = "rainbow";
                        json_data["origin_px"] = map_settings.value("map_metadata/map_w").toDouble();
                        json_data["origin_py"] = map_settings.value("map_metadata/map_origin_v").toDouble();
                        json_data["width_gm"] = map_settings.value("map_metadata/map_w").toDouble();
                        json_data["height_gm"] = map_settings.value("map_metadata/map_h").toDouble();
                        double map_val = 1/map_settings.value("map_metadata/map_grid_width").toDouble();
                        double map_val_round = round(map_val);
                        //                        int val = map_val.toInt();
                        //                        qDebug()<<val;
                        json_data["scale_m2px"] = map_val_round;
                        json_data["flipped"] = "false";
                    }
                }
                else
                {
                    //for get map name
                    map_id = params["map_id"].toString();
                    json_data["map_id"] = map_id;

                    //for get map info
                    map_config_path = QDir::homePath()+"/maps/"+map_id+"/map_meta.ini";
                    qDebug()<< map_config_path;

                    QFileInfo map_config_info(map_config_path);
                    if(map_config_info.exists() && map_config_info.isFile())
                    {
                        QSettings map_settings(map_config_path, QSettings::IniFormat);
                        //                        map_id = settings.value("FLOOR/map_name").toString();

                        json_data["map_alias"] = "rainbow";
                        json_data["origin_px"] = map_settings.value("map_metadata/map_origin_u").toDouble();
                        json_data["origin_py"] = map_settings.value("map_metadata/map_origin_v").toDouble();
                        json_data["width_gm"] = map_settings.value("map_metadata/map_w").toDouble();
                        json_data["height_gm"] = map_settings.value("map_metadata/map_h").toDouble();
                        double map_val = 1/map_settings.value("map_metadata/map_grid_width").toDouble();
                        double map_val_round = round(map_val);
                        //                        int val = map_val.toInt();
                        //                        qDebug()<<val;
                        json_data["scale_m2px"] = map_val_round;
                        json_data["flipped"] = "false";
                    }
                }

                json_out["data"] = json_data;

                QJsonDocument doc_json(json_out);
                QString str_json(doc_json.toJson(QJsonDocument::Indented));
                //                    qDebug()<<str_json;

                emit msgSendSignal(str_json); //for debuging
                pClient->sendTextMessage(str_json);
            }

            else if(action == "get_map_data")
            {
                //4.10 Get Map Data
                //                qDebug()<<params["map_id"];
                QString id = params["map_id"].toString();//server 에서 요청한 map 의 id 파싱
                QJsonObject json;
                QJsonObject json_data;
                QString fileName = params["filename"].toString();
                //                QString filename;

                //                qDebug()<<"yujin file name : "<<fileName;

                int image_file_size;
                json_data["data_type"] = params["data_type"];
                json["msg_type"] = "cmd_result";
                json["result"] = "success";
                json["error_info"] = QJsonValue::Null;
                //                json_data["map_id"] = params["map_id"];
                //                qDebug()<<id;

                json["uuid"] = uuid;
                //                map_config_path = QDir::homePath()+"/maps/map_4/map_edited.png";

                if (id == "") //현재 slam에서 사용중인 map
                {
                    config_path = QDir::homePath()+"/robot_config.ini";
                    QFileInfo config_info(config_path);
                    if(config_info.exists() && config_info.isFile())
                    {
                        QSettings settings(config_path, QSettings::IniFormat);
                        map_id = settings.value("FLOOR/map_name").toString();
                        fileName = map_id;
                        json_data["map_id"] = map_id;
                        json_data["filename"] = map_id;
                        json_data["data_type"] = params["data_type"].toString();
                    }

                    if (params["data_type"].toString() == "map_image_png")
                    {
                        //for get map name
                        map_config_path = QDir::homePath()+"/maps/"+map_id+"/map_edited.png";
                        //                         }
                    }
                    else if(params["data_type"].toString() == "map_package")
                    {
                        //for get map_package
                        map_config_path = QDir::homePath()+"/maps/"+map_id+".tar.xz";
                        //                         }
                    }
                    json_data["filesize"] = image_file_size;
                    qDebug()<<"image : "<<image_file_size;

                    json["data"] = json_data;

                    QJsonDocument doc_json(json);
                    QString str_json(doc_json.toJson(QJsonDocument::Indented));
                    //                    qDebug()<<str_json;

                    emit msgSendSignal(str_json); //for debuging
                    pClient->sendTextMessage(str_json);
                    send_img_package(map_config_path,image_file_size,params["data_type"].toString(),fileName);
                }

                else
                {
                    //만약 리스트안에 없는 파일을 달라고 하면 터지지 않도록 예외처리 해주어야함.
                    map_id = id;
                    qDebug()<<"map id : "<<map_id;

                    QString map_path = QDir::homePath()+"/maps/";
                    QDir dir(map_path);
                    qDebug()<<"map_path id : "<<map_path;

                    QStringList itemlist; //임시적으로 데이터 길이 늘려놓음.
                    //                    QString filelist;

                    foreach(QFileInfo item, dir.entryInfoList(QDir::NoDotAndDotDot | QDir::AllEntries) )
                    {
                        QString filelist;
                        if(item.isDir())
                        {
                            filelist = item.baseName();
                            itemlist += filelist;
                        }
                    }
                    qDebug()<<"itemlist : "<<itemlist;

                    if(itemlist.filter(id).count() != 0)
                    {
                        qDebug()<<"파일이 있습니다.";
                        //                        qDebug()<<"map_id : " <<params["data_type"].toString();

                        if (params["data_type"].toString() == "map_image_png")
                        {
                            //for get map name

                            qDebug()<<"map_id : "<<map_id;
                            map_config_path = QDir::homePath()+"/maps/"+map_id+"/map_edited.png";
                            qDebug()<<"map_config_path :"<<map_config_path;
                            fileName = map_id;
                            json_data["map_id"] = map_id;
                            json_data["filename"] = map_id;
                            json_data["data_type"] = params["data_type"].toString();

                        }
                        else if (params["data_type"].toString() == "map_package")
                        {
                            //for get map name
                            map_config_path = QDir::homePath()+"/maps/"+map_id+".tar.xz";
                            //                        map_id = id;
                            fileName = map_id+".tar.xz";
                            json_data["map_id"] = map_id;
                            json_data["filename"] = map_id;
                            json_data["data_type"] = params["data_type"].toString();
                        }
                        json_data["filesize"] = image_file_size;
                        qDebug()<<"image : "<<image_file_size;

                        json["data"] = json_data;

                        QJsonDocument doc_json(json);
                        QString str_json(doc_json.toJson(QJsonDocument::Indented));
                        //                    qDebug()<<str_json;

                        emit msgSendSignal(str_json); //for debuging
                        pClient->sendTextMessage(str_json);
                        send_img_package(map_config_path,image_file_size,params["data_type"].toString(),fileName);
                    }

                    else
                    {
                        qDebug()<<"파일이 없습니다.";
                    }

                }

            }

            else if(action == "set_map_data")
            {
                //4.11 Set Map Data
            }

            else if(action == "get_robot_info")
            {
                //4.12 Get robot info

                QJsonObject json_out;
                QString robot_id,map_id;
                QJsonObject json_data;

                json_out["msg_type"] = "cmd_result";
                json_out["result"] = "success";
                json_out["error_info"] = QJsonValue::Null;

                QString config_path = QDir::homePath()+"/robot_config.ini";
                QFileInfo config_info(config_path);
                if(config_info.exists() && config_info.isFile())
                {
                    QSettings settings(config_path, QSettings::IniFormat);
                    robot_id = settings.value("ROBOT_SW/robot_id").toString();
                    map_id = settings.value("FLOOR/map_name").toString();
                    QJsonObject json_map;

                    json_map["map_id"] = map_id;
                    json_map["map_alias"] = "1F";
                    json_data["map"] = json_map;
                    //                    json_data =

                    //                    QJsonObject json_robot_id;
                    json_data["robot_id"] = robot_id;
                    json_data["robot_alias"] = robot_id;
                    //                    json_data["map"]=json_robot_id;

                }

                QJsonObject json_pose;
                json_pose["x"] = 5;
                json_pose["y"] = 5;
                json_pose["th"] = 10;
                json_data["robot_pose"] = json_pose;

                QJsonObject json_battery;
                json_battery["level"] = 90;
                json_battery["in_charging"] = false;
                json_data["battery"] = json_battery;

                json_data["docking_direction"] = "foward";
                json_out["data"]=json_data;

                json_out["uuid"]=uuid;

                QJsonDocument doc_json(json_out);
                QString str_json(doc_json.toJson(QJsonDocument::Indented));

                emit msgSendSignal(str_json); //for debuging
                pClient->sendTextMessage(str_json);


            }

            else if(action == "pick")
            {
                //4.13 pick item

                QJsonObject json_out;
                QString map_id;

                //                QJsonObject tempDest = params["item_id"].toObject();
                //                QString mm = QJsonDocument(tempDest).toJson(QJsonDocument::Compact);
                //                qDebug()<<mm;

                //                params
                QString _id = params["item_id"].toString();
                int item_id = params["item_id"].toInt();
                int item_count = params["item_count"].toInt();
                double shelve_height = params["shelve_height"].toDouble();
                double shelve_degree = params["shelve_degree"].toDouble();
                qDebug()<<_id;
                std::cout<<item_id<<item_count<<shelve_height<<shelve_degree<<std::endl;

                json_out["msg_type"] = "cmd_result";
                json_out["result"] = "success";
                json_out["error_info"] = QJsonValue::Null;

                map_id = params["map_id"].toString();

                QJsonObject json_data;
                json_data["success_count"]=3;
                json_data["failure_count"]=0;
                json_out["data"]=json_data;

                json_out["uuid"]=uuid;

                QJsonDocument doc_json(json_out);
                QString str_json(doc_json.toJson(QJsonDocument::Indented));

                emit msgSendSignal(str_json); //for debuging
                pClient->sendTextMessage(str_json);
            }

            else if(action == "set_position")
            {
                //4.11 Set position (ini file 변경.)

                QJsonObject json_out;
                QString map_id;

                json_out["msg_type"] = "cmd_result";
                json_out["result"] = "success";
                json_out["error_info"] = QJsonValue::Null;

                map_id = params["map_id"].toString();

                QString config_path = QDir::homePath()+"/robot_config.ini";
                QFileInfo config_info(config_path);
                if(config_info.exists() && config_info.isFile())
                {
                    QSettings settings(config_path, QSettings::IniFormat);
                    settings.setValue("FLOOR/map_name",map_id);
                    settings.setValue("FLOOR/map_path",QDir::homePath()+"/maps/"+map_id);
                    //                    json_data["map_id"] = map_id;


                }

                QJsonDocument doc_json(json_out);
                QString str_json(doc_json.toJson(QJsonDocument::Indented));

                emit msgSendSignal(str_json); //for debuging
                pClient->sendTextMessage(str_json);


            }
        }
    }
}

QJsonObject websocket::data(QString robot_manufacture, QString action,QString robot_type, QString map_id, QString map_alias)
{

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

void websocket::send_img_package(QString map_config_path,int image_file_size,QString signature,QString fileName)
{
    QWebSocket *pClient = qobject_cast<QWebSocket *>(sender());
    //for get map info

    file = new QFile(map_config_path);
    qDebug()<<"path : "<<map_config_path;
    file->open(QFile::ReadOnly);
    fileData_byte = file->readAll();

    image_file_size = fileData_byte.size();

    QString prifix = "FILE_DATA";
    QByteArray prifix_byte = prifix.toUtf8();

    //    QString signature = json_data["data_type"].toString();
    //    QString signature = signature;
    if (signature == "map_image_png")
    {
        signature = "MAP_IMAGE_PNG___";
    }
    else
    {
        signature = "MAP_PACKAGE_COMP";
    }

    int header_size = signature.size()+fileName.size();
    qDebug()<<"fileName :"<<fileName;
    qDebug()<<"fileName size :"<<fileName.size();
    qDebug()<<"header_size :"<<header_size;
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

    //파일 잘 갔는지 디버깅용



    pClient->sendBinaryMessage(prifix_byte);
    qDebug()<<"signature : "<<signature;

    if (signature == "MAP_IMAGE_PNG___")
    {
        //이미지 잘 전송되었는지 디버깅용

        QFile file("send_IMG.bin");// bin file 생성

        file.open(QIODevice::WriteOnly);
        file.write(fileData_byte);
        file.close();
        img = cv::imread(map_config_path.toStdString(),cv::IMREAD_GRAYSCALE); // 이미지 읽기
        cv::resize(img, src, cv::Size(img.cols/3, img.rows/3 ), 0, 0, CV_INTER_NN );
        imshow("지금 사용하고 있는 맵",src);
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
