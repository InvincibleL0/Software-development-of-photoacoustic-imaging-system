#ifndef WIDGET_H
#define WIDGET_H

#include<opencv2/opencv.hpp>
#include<QWidget>
#include<QTimer>
#include<QString>
#include<QComboBox>
#include<QImage>
#include<QPixmap>
#include<QFileDialog>
#include<QThread>
#include<QThreadPool>
#include<QtConcurrent/QtConcurrent>
#include<QtSerialPort/QSerialPort>
#include<QtSerialPort/QSerialPortInfo>

#include<QPointF>
#include<QtCharts/QChartView>
#include<QtCharts/QLineSeries>
#include<QtCharts/QBarSeries>
#include<QtCharts/QBarSet>
#include<QtCharts/QChart>
#include<QtCharts/QValueAxis>
#include<QtCharts/QChartGlobal>
#include<QtCharts/QBarCategoryAxis>
#include<QtWidgets/QVBoxLayout>
#include<QStandardItem>
#include<QStandardItemModel>
#include<QByteArray>
#include<QPainter>
#include<QtPrintSupport/QPrinter>
#include<QScreen>

#include<dlltyp.h>
#include<regs.h>
#include<spcerr.h>
#include<spcm_drv.h>
#include<spcm_ostools.h>

#include<concurrentqueue/concurrentqueue.h>

#include<QVTKWidget.h>
#include<vtkAutoInit.h>
#include<vtkSmartPointer.h>

#include<vtkPNGReader.h>
#include<vtkGPUVolumeRayCastMapper.h>
#include<vtkVolume.h>
#include<vtkColorTransferFunction.h>
#include<vtkVolumeProperty.h>
#include<vtkPiecewiseFunction.h>
#include<vtkRenderer.h>
#include<vtkRenderWindow.h>
#include<vtkRenderWindowInteractor.h>
#include<vtkImageData.h>
#include<vtkCubeAxesActor.h>

#include<vtkOutputWindow.h>
#include<vtkActor.h>
#include<vtkSmartVolumeMapper.h>
#include<vtkStringArray.h>

#include<algorithm>
#include<iostream>
#include<fstream>
#include<stdio.h>
#include<stdlib.h>

using namespace cv;
using namespace std;
using namespace QtCharts;

VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingVolumeOpenGL2);
#define vtkRenderingCore_AUTOINIT 4(vtkInteractionStyle,vtkRenderingFreeType,vtkRenderingFreeTypeOpenGL2,vtkRenderingOpenGL2)
#define vtkRenderingVolume_AUTOINIT 1(vtkRenderingVolumeOpenGL2)

namespace Ui {class Widget;}

class Widget : public QWidget
{
    Q_OBJECT

public:
    explicit Widget(QWidget *parent = nullptr);
    ~Widget();

private slots:
    void alineDebug();
    void drawCharts();
    void acquisitionStart();
    void dataProcess();
    void bscanDisplay();
    void mapDisplay();
    void savePathSelect();
    void volumeRender();

    void readDat();
    void savePathSelect2();
    void drawBarGraph();
    static void onMouse1(int event, int x, int y, int flags, void *ustc);
    static void onMouse2(int event, int x, int y, int flags, void *ustc);
    void readLog(vector<double> &vec);
    void depthChange();
    void contrastChange();
    void saveImage();
    void lookUpTable();
    void saveVTKwidget();

private:
    Ui::Widget *ui;

    QChart *m_chart1;
    QLineSeries *m_series1;
    QList<QPointF> m_data1;

    QSerialPort serialPort1, serialPort2, serialPort3;
    void initPort();
    void serialPortOpen1();
    void serialPortOpen2();
    void serialPortOpen3();

    void laserPumpSwitch();
    void laserTriggerMode();
    void laserEnergySet();
    void laserFrequencySet();

    void startAlineDebug();
    void startFullScan();
    void endScan();
    void backToZero();
    short fixAlineCount();
    short fixBscanCount();
    short midValue(short *inArray, int arrayLength);
    QByteArray stringToHex(QString str);
    char convertHexChar(char ch);
    void writeLog();
    void sleep(int msec);

    QChart *m_chart;
    QBarSeries *m_series;
    QBarSet *set0 = new QBarSet(" 直径(mm) ");
    QBarSet *set1 = new QBarSet(" 弯曲程度 ");
};

#endif // WIDGET_H
