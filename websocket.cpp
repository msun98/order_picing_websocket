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
    //    connect(&MainWindow, SIGNAL(msgSignal(bool)), this, SLOT(GetSignal(float, float,float,float,float,float)));
}

websocket::~websocket()
{
    qDebug()<<"웹소켓 프로그램 종료";
}


void websocket::open()
{
    server = new QWebSocketServer("rb_websocket", QWebSocketServer::NonSecureMode, this);
    if(server->listen(QHostAddress::Any, 28081)) //서버가 들어오는 연결을 수신하기 위해 listen을 호출(yujin -> client, rainbow -> server)
    {
        connect(server, SIGNAL(newConnection()), this, SLOT(onNewConnection()));
    }

    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(onTimeout()));
    //    timer->start(100);

    // for ipc (robot status check)
    connect(timer, SIGNAL(timeout()), this, SLOT(timerLoop()));
//    timer.start(100);
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

void websocket::CMD_RESULT(QString result)
{
    for(int i=0; i<clients.size(); i++)
    {
        QWebSocket *pSocket = clients[i];
        //        sendNotice(pSocket);
        QJsonObject json;
        json["msg_type"] = "cmd_result";
        json["result"] = result;
        json["data"] = "dddd";

        qDebug()<<"result : "<<result;

        if (result == "failure")
        {
            QJsonObject json_error_info;
            json_error_info["error_code"] = "dddd";
            json_error_info["description"] = "dddd";
            json_error_info["debug_msg"] = "dddd";
            json["error"] = json_error_info;
        }

        json["uuid"] = uuid;

        QJsonDocument doc_json(json);
        QString str_json(doc_json.toJson(QJsonDocument::Indented));
        emit msgSendSignal(str_json);
        pSocket->sendTextMessage(str_json);

        std::string rainbow_cmd(str_json.toStdString());
        IPC::WEB_commend RB_CMD;
        memcpy((uint8_t*)RB_CMD.json_cmd, rainbow_cmd.data(), 1000);
        RB_CMD.json_cmd_size = rainbow_cmd.size();
        std::cout<<RB_CMD.json_cmd<<std::endl;

        std::string uuid_send(uuid.toStdString());
//            std::cout<<uuid_send<<std::endl;
//            yj_CMD.uuid=uuid_send;
        memcpy((uint8_t*)RB_CMD.json_uuid, uuid_send.data(), 30);

        RB_CMD.json_uuid_size = uuid_send.size();
//            std::cout<<RB_CMD.json_uuid_size<<std::endl;

        ipc.set_Rainbow_CMD(RB_CMD); //통합 ui에 유진로봇 명령을 넘기기 위한 코드.

    }

    //    QWebSocket *pClient = qobject_cast<QWebSocket *>(sender());

}

void websocket::timerLoop()
{
   //    QWebSocket *pClient = qobject_cast<QWebSocket *>(sender());
    //    sendNotice(pSocket);

    // status
    IPC::STATUS status = ipc.get_status();
    IPC::PATH path = ipc.get_path();
    IPC::POSE pose;

    IPC::SUCCESS_CHECK sucess = ipc.get_mobile_success_check();
    if(sucess.tick != last_sucess_tick) //틱으로 로봇 상태 갱신되는지 확인.
    {
        //        QString check;
        // 로봇 이동 성공 여부 체크.
        if(sucess.check == 1)
        {
            //            check = "success";
            qDebug()<<"success";
            qDebug()<<"dddddd"<<sucess.check;
            CMD_RESULT("success");
        }
        else if(sucess.check == 0)
        {
            //            check = "fail";
            qDebug()<<"fail";
            CMD_RESULT("failure");
        }

    }
    last_sucess_tick = sucess.tick;


    if(status.tick != last_status_tick) //틱으로 로봇 상태 갱신되는지 확인.
    {
        connected = true;

        //        ui->la_connection_check->setText("로봇과 연결되었습니다.");
        QString mobile_status_str;

        //get robot pose
        //        robot_purpose_x=status.robot_pose[0];
        //        robot_purpose_y=status.robot_pose[1];
        //        robot_purpose_theta=status.robot_pose[2];

        //get robot battery
        //        IPC::STATUS status = ipc.get_status();
        //        status_charge = status.status_charge; //배터리 충전중인지 판단.
        //        status_power = status.status_power; //  배터리 잔량 체크(로봇에 넣어 확인.)

        //get robot path
        //        memcpy(goal_x, path.x, sizeof(float)*512);
        //        memcpy(goal_y, path.y, sizeof(float)*512);

        //        //get robot pose 유진로봇으로부터 받은 이동해야할 곳
        //        pose.x=x;
        //        pose.y=y;
        //        pose.theta=theta;


        ipc.set_move_where(pose);
        //        qDebug()<<"pose.x"<<x;

        //        goal_x = path.x;
        //        goal_y = path.y;


        //        emit StatusSignal(mobile_pose.x,mobile_pose.y,mobile_pose.theta,mobile_status.status_charge,mobile_status.status_power);



        mobile_status_str.sprintf("tick:%d, connection(m0, m1): %d, %d\nstatus(m0, m1): %d, %d\ntemperature(m0, m1): %d, %d,"
                                  " cur(m0, m1):%.2f, %.2f\ncharge, power, emo, remote state: %d, %d, %d, %d\nBAT(in, out, cur):%.3f, %.3f, %.3f"
                                  "\npower: %.3f\ntotal power: %.3f,\nrobot pose: %.3f,%.3f,%.3f",
                                  status.tick,
                                  status.connection_m0, status.connection_m1, status.status_m0, status.status_m1, status.temp_m0, status.temp_m1,
                                  (double)status.cur_m0/10.0, (double)status.cur_m1/10.0,
                                  status.status_charge, status.status_power, status.status_emo, status.status_remote,
                                  status.bat_in, status.bat_out, status.bat_cur,
                                  status.power, status.total_power,x,y,theta);

        //status 확인용
        //        ui->te_send_msg->setText(mobile_status_str);
        //        qDebug()<<mobile_status_str;
        //        emit msgSendSignal(mobile_status_str);
    }
    else
    {
        connected = false;
        //        emit msgSendSignal(mobile_status_str);
        //       ui->la_connection_check->setText("로봇과 연결이 끊겼습니다.");
    }
    emit check_robot_connected(connected);
    last_status_tick = status.tick;



    //    // map
    //    IPC::MAP map = ipc.get_map();
    //    if(map.tick != last_map_tick)
    //    {
    //        cv::Mat map_img(1000, 1000, CV_8U, cv::Scalar(0));
    //        memcpy((uint8_t*)map_img.data, map.buf, 1000*1000);

    //        ui->lb_Screen1->setPixmap(QPixmap::fromImage(mat_to_qimage_cpy(map_img)));
    //        ui->lb_Screen1->setScaledContents(true);
    //        ui->lb_Screen1->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    //    }
    //    last_map_tick = map.tick;

    //    // obstacle map
    //    IPC::MAP obs = ipc.get_obs();
    //    if(obs.tick != last_obs_tick)
    //    {
    //        cv::Mat obs_img(1000, 1000, CV_8U, cv::Scalar(0));
    //        memcpy((uint8_t*)obs_img.data, obs.buf, 1000*1000);

    //        ui->lb_Screen2->setPixmap(QPixmap::fromImage(mat_to_qimage_cpy(obs_img)));
    //        ui->lb_Screen2->setScaledContents(true);
    //        ui->lb_Screen2->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    //    }
    //    last_obs_tick = obs.tick;

    //    // cam0
    //    IPC::IMG cam0 = ipc.get_cam0();
    //    if(cam0.tick != last_cam0_tick)
    //    {
    //        cv::Mat cam0_img(270, 480, CV_8U, cv::Scalar(0));
    //        memcpy((uint8_t*)cam0_img.data, cam0.buf, 270*480);

    //        ui->lb_Screen3->setPixmap(QPixmap::fromImage(mat_to_qimage_cpy(cam0_img)));
    //        ui->lb_Screen3->setScaledContents(true);
    //        ui->lb_Screen3->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    //    }
    //    last_cam0_tick = cam0.tick;

    //    // cam1
    //    IPC::IMG cam1 = ipc.get_cam1();
    //    if(cam1.tick != last_cam1_tick)
    //    {
    //        cv::Mat cam1_img(270, 480, CV_8U, cv::Scalar(0));
    //        memcpy((uint8_t*)cam1_img.data, cam1.buf, 270*480);

    //        ui->lb_Screen4->setPixmap(QPixmap::fromImage(mat_to_qimage_cpy(cam1_img)));
    //        ui->lb_Screen4->setScaledContents(true);
    //        ui->lb_Screen4->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    //    }
    //    last_cam1_tick = cam1.tick;
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
    qDebug()<<message;

    //for toss massage integrated UI
    std::string yujin_cmd(message.toStdString());
    IPC::WEB_commend yj_CMD;
    memcpy((uint8_t*)yj_CMD.json_cmd, yujin_cmd.data(), 1000);
    yj_CMD.json_cmd_size = yujin_cmd.size();
    std::cout<<yj_CMD.json_cmd_size<<std::endl;


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
            std::string uuid_send(uuid.toStdString());
//            std::cout<<uuid_send<<std::endl;
//            yj_CMD.uuid=uuid_send;
            memcpy((uint8_t*)yj_CMD.json_uuid, uuid_send.data(), 30);

            yj_CMD.json_uuid_size = uuid_send.size();
//            std::cout<<yj_CMD.json_uuid_size<<std::endl;

            ipc.set_Yujin_CMD(yj_CMD); //통합 ui에 유진로봇 명령을 넘기기 위한 코드.

            if(action == "move")
            {
                //4.1 Move
                IPC::POSE pose;
                IPC::ROBOT_COMMAND robot_command;
                robot_command.robot_status = 0;
                ipc.set_mobile_status(robot_command);
                QJsonObject tempDest = params["dest"].toObject();
                //                QString mm = QJsonDocument(tempDest).toJson(QJsonDocument::Compact);
                //                qDebug()<<mm;

                x = tempDest["x"].toDouble();
                y = tempDest["y"].toDouble();
                theta = tempDest["theta"].toDouble();

                pose.x=x;
                pose.y=y;
                pose.theta=theta*180/3.14; //유진로봇에서는 라디안으로 정보를 넘김.

                ipc.set_move_where(pose);



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
                IPC::ROBOT_COMMAND robot_command;
                robot_command.robot_status = 1;
                qDebug()<<robot_command.robot_status;
                ipc.set_mobile_status(robot_command);

                qDebug()<<"Pause";

                //                qDebug()<<"stop";
                //                pClient->sendTextMessage("stop");
                //                pMP->StopOmron();
            }

            else if(action == "resume")
            {
                //4.5 Resume
                IPC::ROBOT_COMMAND robot_command;
                robot_command.robot_status = 2;
                qDebug()<<robot_command.robot_status;
                ipc.set_mobile_status(robot_command);

                qDebug()<<"Resume";
                //robot에 stop 명령줘야함.
                //                pClient->sendTextMessage("stop");
                //                pMP->StopOmron();
            }

            else if(action == "stop")
            {
                //4.6 Stop
                IPC::ROBOT_COMMAND robot_command;
                robot_command.robot_status = 3;
                qDebug()<<robot_command.robot_status;
                ipc.set_mobile_status(robot_command);

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
                        map_config_path = QDir::homePath()+"/maps/"+map_id+"/changed_map.png";
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
                            map_config_path = QDir::homePath()+"/maps/"+map_id+"/changed_map.png";
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
           ￣ }

            else if(action == "get_robot_info")
            {
                //4.12 Get robot info

                // status
                IPC::MOBILE_POSE mobile_pose = ipc.get_mobile_pos();
                IPC::STATUS status = ipc.get_status();
                //                status_charge = status.status_charge; //배터리 충전중인지 판단.
                //                status_power = status.status_power; //  배터리 잔량 체크(로봇에 넣어 확인.)

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


                // check motor init
                // 모터 상태 입력받을 구조체 (global_defines에 있음.)
                //                MOBILE_STATUS mobile_status ;
                //                        '= ipc.get_status();


                QJsonObject json_pose;
                json_pose["x"] = mobile_pose.pose[0];
                json_pose["y"] = mobile_pose.pose[1];
                double mobile_th =double(mobile_pose.pose[2]);
                json_pose["th"] = mobile_th/180*3.14;
                json_data["robot_pose"] = json_pose;

                QJsonObject json_battery;
                //                json_battery["level"] = 90;

                json_battery["level"] = status.status_power;

                if(status.status_charge == 0)
                {
                    charging_status = false;
                }
                else
                {
                    charging_status = true;
                }
                json_battery["in_charging"] = charging_status;
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
//                QString str_json(doc_json.toJson(QJsonDocument::Indented));
                QString str_json(doc_json.toJson(QJsonDocument::Indented));

                std::string rainbow_cmd(str_json.toStdString());
                IPC::WEB_commend RB_CMD;
                memcpy((uint8_t*)RB_CMD.json_cmd, rainbow_cmd.data(), 1000);
                RB_CMD.json_cmd_size = rainbow_cmd.size();
                std::cout<<RB_CMD.json_cmd_size<<std::endl;

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

    QString config_path = QDir::homePath()+"/robot_config.ini";
    QFileInfo config_info(config_path);
    if(config_info.exists() && config_info.isFile())
    {
        QSettings settings(config_path, QSettings::IniFormat);
        QString robot_id = settings.value("ROBOT_SW/robot_id").toString();
        map_id = settings.value("FLOOR/map_name").toString();
    }

    //    QJsonObject json_robot;
    //    json_robot["x"] = QString::number(networkThread->x);
    //    json_robot["y"] = QString::number(networkThread->y);
    //    json_robot["theta"] = QString::number(networkThread->heading);
    //    json["robot_pose"] = json_robot;

    QJsonObject json_map;
    json_map["map_id"] = map_id;
    json_map["map_alias"] = map_id;
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
