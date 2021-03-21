#ifndef WIDGET_H
#define WIDGET_H

#include<opencv2/opencv.hpp>
#include<QWidget>
#include<QTimer>
#include<QString>
#include<QImage>
#include<QPixmap>
#include<QFileDialog>
#include<QThread>
#include<QThreadPool>
#include<QtConcurrent/QtConcurrent>
#include<QtSerialPort/QSerialPort>
#include<QtSerialPort/QSerialPortInfo>

#include<dlltyp.h>
#include<regs.h>
#include<spcerr.h>
#include<spcm_drv.h>
#include<spcm_ostools.h>
//#include<spcm_oswrap.h>
//#include<spcm_lib_card.h>

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

#include<vtkOutputWindow.h>
#include<vtkActor.h>
#include<vtkSmartVolumeMapper.h>
#include<vtkStringArray.h>

#include<algorithm>
#include<iostream>
#include<fstream>

using namespace cv;
using namespace std;

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
    //void GetParameter();
    void AcquisitionStart();
    void DataProcess();
    void BscanDisplay();
    void MapDisplay();
    void VolumeRender();
    void CCD();
    void StartTrigger();
    void SavePathSelect();
private:
    Ui::Widget *ui;
    QSerialPort SerialPort;
    void InitPort();
    void SerialPortOpen();
    void BandPass(unsigned int64 n, double SampleRate, double HignFre, double LowFre, short *InArray, unsigned int64 length);
    QByteArray StringToHex(QString str);
    char ConvertHexChar(char ch);
};

#endif // WIDGET_H
