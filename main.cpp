#include "widget.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    vtkOutputWindow::SetGlobalWarningDisplay(0);
    QApplication a(argc, argv);
    Widget w;
    w.show();

    return a.exec();
}
