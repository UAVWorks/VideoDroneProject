/* ********************************
 *  general Socket information
 *  Flight Controller socket ( 192.168.1.121 , 10000)
 *  Video Socket (192.168.1.121, 12000)
 *
 *
 *
 *
 *
 *
 *
 *
 * *********************************/


#include "touchwidget.h"
#include "ui_touchwidget.h"

template <typename T>

std::string to_string(T value)
{
    std::ostringstream os ;
    os << value ;
    return os.str() ;
}

TouchWidget::TouchWidget(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::TouchWidget)
{
    ui->setupUi(this);
    ui->centralWidget->setAttribute(Qt::WA_AcceptTouchEvents, true);

    // Set Flight Controller UI parameters
    ui->Thrust->setMinimum(0);
    ui->Thrust->setMaximum(100);
    ui->Roll->setMinimum(0);
    ui->Roll->setMaximum(100);
    ui->Pitch->setMinimum(0);
    ui->Pitch->setMaximum(100);
    ui->Yaw->setMinimum(0);
    ui->Yaw->setMaximum(100);


    //serverIP   = "192.168.1.121";
    //serverPort = 12000;
    //addrLen = sizeof(struct sockaddr_in);

    string_now = new QString("");
    socket_fc = new QTcpSocket(this); // socket to pass fc data
    socket_vid = new QTcpSocket(this); //socket to pass video data

    timer_fc = new QTimer(this);
    connect(timer_fc, SIGNAL(timeout()), this, SLOT(getFlightControlVal())); // Get data from UserInterface
    timer_fc->start(10);

    connect(ui->ConnectButton, SIGNAL(pressed()), this , SLOT(on_ConnectButton_clicked())); // Signal to connect to UAV to start FC
    connect(ui->DisconnectButton, SIGNAL(pressed()), this, SLOT(on_DisconnectButton_clicked())); // signal to disconnect UAV FC

}

void TouchWidget::resetVal(){

    if(!ui->Thrust->isSliderDown()){
         ui->Thrust->setValue((ui->Thrust->maximum() + ui->Thrust->minimum())/2.0);
    }
    if(!ui->Roll->isSliderDown()){
        ui->Roll->setValue((ui->Roll->maximum() + ui->Roll->minimum())/2.0);
    }
    if(!ui->Pitch->isSliderDown()){
        ui->Pitch->setValue((ui->Pitch->maximum() + ui->Pitch->minimum())/2.0);
    }
    if(!ui->Yaw->isSliderDown()){
        ui->Yaw->setValue((ui->Yaw->maximum() + ui->Yaw->minimum())/2.0);
    }

}

void TouchWidget::getFlightControlVal(){
    float thrust, roll, pitch, yaw;
    std::string string_data, sock_data;

    thrust = ui->Thrust->value();
    roll = ui->Roll->value();
    pitch = ui->Pitch->value();
    yaw = ui->Yaw->value();

    sock_data = to_string(thrust)+"\t" + to_string(roll) \
            + "\t"+ to_string(pitch)+"\t"+ to_string(yaw)+"\n";
    const char *cstr = sock_data.c_str();

    if( socket_fc->isOpen() ){
        socket_fc->write(cstr);
    }
    string_data = "Thrust = " + to_string(thrust)+" Roll = " + to_string(roll) \
            + " Pitch = "+ to_string(pitch)+" Yaw = "+ to_string(yaw)+"\n";

    *string_now = QString::fromStdString(string_data);
    //ui->FFlabel->setText(*string_now);

    resetVal();
}

TouchWidget::~TouchWidget()
{
    delete ui;
}

void TouchWidget::on_ConnectButton_clicked()
{
    ui->FFlabel2->setText("Trying to establish connection\n");
    socket_fc->connectToHost("192.168.1.121", 10000);

    if(socket_fc->waitForConnected(30000)){
        ui->FFlabel2->append("Connected to UAV!");
    }else{
        ui->FFlabel2->append("Connection to UAV failed!");
    }

}

void TouchWidget::on_DisconnectButton_clicked()
{
    ui->FFlabel2->setText("Disconnected from UAV\n");
    socket_fc->disconnectFromHost();
}

void TouchWidget::on_StartVid_clicked()
{
    sourceAddress = "192.168.1.121";
    sourcePort = 12000;

    char* testdata = "Hi";
    sock.sendTo(testdata, sizeof(testdata), sourceAddress, sourcePort);

    timer_vid = new QTimer(this);
    connect(timer_vid, SIGNAL(timeout()), this, SLOT(VidRun())); // Get data from UserInterface
    timer_vid->start(200);

}

void TouchWidget::VidRun(){

    VID_RUNNING = true;
    do {
        recvMsgSize = sock.recvFrom(buffer, BUF_LEN, sourceAddress, sourcePort);
       } while (recvMsgSize > sizeof(int));
       total_pack = ((int * ) buffer)[0];

       //cout << "expecting length of packs:" << total_pack << endl;
       longbuf = new char[PACK_SIZE * total_pack];
       for (int i = 0; i < total_pack; i++) {
            recvMsgSize = sock.recvFrom(buffer, BUF_LEN, sourceAddress, sourcePort);
            if (recvMsgSize != PACK_SIZE) {
                cerr << "Received unexpected size pack:" << recvMsgSize << endl;
                continue;
             }
             memcpy( & longbuf[i * PACK_SIZE], buffer, PACK_SIZE);
       }

       //  cout << "Received packet from " << sourceAddress << ":" << sourcePort << endl;

       rawData = Mat(1, PACK_SIZE * total_pack, CV_8UC1, longbuf);
       frame = imdecode(rawData, CV_LOAD_IMAGE_COLOR);
       //CV_LOAD_IMAGE_COLOR
       if (frame.size().width == 0) {
           cerr << "decode failure!" << endl;
           // continue;
       }
       //imshow("recv", frame);
       free(longbuf);
       QImage qimg((const uchar *) frame.data, frame.cols, frame.rows, frame.step, QImage::Format_RGB888);
       int w = ui->Vid->width();
       int h = ui->Vid->height();
       ui->Vid->setPixmap(QPixmap::fromImage(qimg).scaled(w,h,Qt::KeepAspectRatio));

}

void TouchWidget::on_StopVid_clicked()
{
    disconnect(timer_vid);
    VID_RUNNING = false;
}
