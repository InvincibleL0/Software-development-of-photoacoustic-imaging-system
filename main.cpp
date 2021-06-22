#include "widget.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    vtkOutputWindow::SetGlobalWarningDisplay(0);
    QApplication a(argc, argv);
    Widget w;
    w.setWindowTitle(" 光声显微成像采集重建 ");
    w.show();

    return a.exec();
}
