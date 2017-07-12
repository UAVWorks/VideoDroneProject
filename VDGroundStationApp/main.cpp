#include "touchwidget.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    TouchWidget w;
    w.show();

    return a.exec();
}
