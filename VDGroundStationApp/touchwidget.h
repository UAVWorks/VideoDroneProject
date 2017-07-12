#ifndef TOUCHWIDGET_H
#define TOUCHWIDGET_H

#include <QMainWindow>
#include <QTouchEvent>
#include <QTouchDevice>
#include <QTimer>
#include <QObject>
#include <QDebug>
#include <QTcpSocket>
#include <QAbstractSocket>
#include <iostream>
#include <cstdlib>                // For atoi()
#include <string>
#include <sstream>
#include <vector>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

#include "PracticalSocket.h"      // For UDPSocket and SocketException
#include "config.h"

//#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect/objdetect.hpp>

using namespace cv;

namespace Ui {
class TouchWidget;
}

class TouchWidget : public QMainWindow
{
    Q_OBJECT

public:
    explicit TouchWidget(QWidget *parent = 0);
    ~TouchWidget();

    QTimer *timer_fc, *timer_vid;
    QString *string_now;


private slots:
    void getFlightControlVal();

    void on_ConnectButton_clicked();

    void on_DisconnectButton_clicked();

    void on_StartVid_clicked();

    void on_StopVid_clicked();

    void VidRun();


private:
    QTcpSocket *socket_fc, *socket_vid;

    // Video variable
    bool VID_RUNNING;
    Mat frame;
    string sourceAddress;// = argv[1];
    unsigned short sourcePort;// = Socket::resolveService(argv[2],"udp");
    UDPSocket sock;
    char buffer[BUF_LEN]; // Buffer for echo string
    int recvMsgSize; // Size of received message
    int total_pack;
    char * longbuf;
    Mat rawData;


    socklen_t addrLen;
    int sokt;
    char* serverIP;
    int serverPort;

    Ui::TouchWidget *ui;

    //VideoCapture cap;
    void resetVal();


};
QString toDebug(const QByteArray &line);
#endif // TOUCHWIDGET_H
