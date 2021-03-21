#include "widget.h"
#include "ui_widget.h"

#define BSCANCOUNT 200
#define ALINECOUNT 300
#define ALINEDEPTH 2048
#define BSCAN ALINECOUNT*ALINEDEPTH

static int CutPoint = 0;
static int CutCount = 400;
static int SampleRate = 250;
static int SectionOri = 0;
static int SectionEnd = 0;

static bool ImagingFlag = false;
static bool CCDflag = false;
static bool SerialPortFlag = false;
static moodycamel::ConcurrentQueue<int16> Q1(BSCAN);
static moodycamel::ConcurrentQueue<int16> Q2(BSCAN);
static moodycamel::ConcurrentQueue<int16> Q3(5*ALINECOUNT);
static int16 data1[BSCAN];
static int16 data2[BSCAN];
static int16 data3[5*ALINECOUNT];

Widget::Widget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Widget)
{
    ui->setupUi(this);

    InitPort();

    connect(ui->pushButton, &QPushButton::clicked,
            [=]()
            {
                CutPoint = ui->spinBox_2->value();
                CutCount = ui->spinBox_3->value();
                SampleRate = ui->spinBox_4->value();
                SectionOri = ui->spinBox_5->value();
                SectionEnd = ui->spinBox_6->value();

                ImagingFlag = true;
                QtConcurrent::run(this, &Widget::AcquisitionStart);
                QtConcurrent::run(this, &Widget::DataProcess);
                QtConcurrent::run(this, &Widget::BscanDisplay);
                QtConcurrent::run(this, &Widget::MapDisplay);
            });
    connect(ui->pushButton_2, &QPushButton::clicked,
            [=]()
            {
                VolumeRender();
            });
    connect(ui->pushButton_3, &QPushButton::clicked,
            [=]()
            {
                SerialPortFlag = !SerialPortFlag;
                SerialPortOpen();
                StartTrigger();
            });
    connect(ui->pushButton_4, &QPushButton::clicked,
            [=]()
            {
                CCDflag = !CCDflag;
                CCD();
            });
    connect(ui->pushButton_5, &QPushButton::clicked,
            [=]()
            {
                SavePathSelect();
            });
}

Widget::~Widget()
{
    delete ui;
}

void Widget::BandPass(unsigned int64 n, double SampleRate, double HignFre, double LowFre, short *InArray, unsigned int64 length)
{
    double a = cos(CV_PI*(HignFre + LowFre) / SampleRate) / cos(CV_PI*(HignFre - LowFre) / SampleRate);
    double a2 = a * a;
    double b = tan(CV_PI*(HignFre - LowFre) / SampleRate);
    double b2 = b * b;
    double r;
    n = n / 4;
    double *A  = static_cast<double *>(malloc(n * sizeof(double)));
    double *d1 = static_cast<double *>(malloc(n * sizeof(double)));
    double *d2 = static_cast<double *>(malloc(n * sizeof(double)));
    double *d3 = static_cast<double *>(malloc(n * sizeof(double)));
    double *d4 = static_cast<double *>(malloc(n * sizeof(double)));
    double *w0 = static_cast<double *>(calloc(n,  sizeof(double)));
    double *w1 = static_cast<double *>(calloc(n,  sizeof(double)));
    double *w2 = static_cast<double *>(calloc(n,  sizeof(double)));
    double *w3 = static_cast<double *>(calloc(n,  sizeof(double)));
    double *w4 = static_cast<double *>(calloc(n,  sizeof(double)));

    for (unsigned int64 i = 0; i < n; ++i)
    {
        r = sin(CV_PI*(2.0*i + 1.0) / (4.0*n));
        SampleRate = b2 + 2.0*b*r + 1.0;
        A[i] = b2 / SampleRate;
        d1[i] = 4.0*a*(1.0 + b * r) / SampleRate;
        d2[i] = 2.0*(b2 - 2.0*a2 - 1.0) / SampleRate;
        d3[i] = 4.0*a*(1.0 - b * r) / SampleRate;
        d4[i] = -(b2 - 2.0*b*r + 1.0) / SampleRate;
    }

    for (unsigned int64 j = 1; j < length; ++j)
    {
        for (unsigned int64 i = 0; i < n; ++i)
        {
            w0[i] = d1[i] * w1[i] + d2[i] * w2[i] + d3[i] * w3[i] + d4[i] * w4[i] + InArray[j];
            InArray[j] = static_cast<short>(A[i] * (w0[i] - 2.0*w2[i] + w4[i]));
            w4[i] = w3[i];
            w3[i] = w2[i];
            w2[i] = w1[i];
            w1[i] = w0[i];
        }
    }
}

void Widget::CCD()
{
    VideoCapture Capture(0);
    while(CCDflag)
    {
        Mat CCDframe;
        Capture >> CCDframe;
        cvtColor(CCDframe, CCDframe, COLOR_BGR2RGB);

        if(ui->checkBox_2->isChecked())
        {
            vector<Mat> Channels;
            Mat GreenChannel;
            split(CCDframe, Channels);
            GreenChannel = Channels[1];
            for (int i = 0; i < GreenChannel.rows; ++i)
            {
                for (int j = 0; j < GreenChannel.cols; ++j) GreenChannel.at<uchar>(i, j) = 0;
            }
            merge(Channels, CCDframe);
        }

        QImage CCDDisplay1 = QImage(CCDframe.data, CCDframe.cols, CCDframe.rows, QImage::Format_RGB888);
        QImage CCDDisplay2 = CCDDisplay1.scaled(QSize(600, 600), Qt::IgnoreAspectRatio);
        ui->label_1->setPixmap(QPixmap::fromImage(CCDDisplay2));
        waitKey(40);
    }
}

void Widget::InitPort()
{
    foreach(const QSerialPortInfo &info,QSerialPortInfo::availablePorts())
    {
        SerialPort.setPort(info);
        if(SerialPort.open(QIODevice::ReadWrite))
        {
            ui->comboBox->addItem(SerialPort.portName());
            SerialPort.close();
        }
    }
}

void Widget::SerialPortOpen()
{
    //SerialPort.setPortName("COM3");
    SerialPort.setPortName(ui->comboBox->currentText());
    if(SerialPort.open(QIODevice::ReadWrite))
    {
        SerialPort.setBaudRate(QSerialPort::Baud115200);
        SerialPort.setDataBits(QSerialPort::Data8);
        SerialPort.setParity(QSerialPort::NoParity);
        SerialPort.setStopBits(QSerialPort::OneStop);
        SerialPort.setFlowControl(QSerialPort::NoFlowControl);
    }
    else return;
}

void Widget::StartTrigger()
{
    if(SerialPortFlag) SerialPort.write(StringToHex("F1"));
    else SerialPort.write(StringToHex("F0"));
}

void Widget::SavePathSelect()
{
    ui->textEdit->clear();
    QString SavePath = QFileDialog::getExistingDirectory(this, tr("Open Directory"), "/home", QFileDialog::ShowDirsOnly);
    ui->textEdit->append(SavePath);
}

void Widget::AcquisitionStart()
{
    int32        lSegmentsize, lPretrigger, lPosttrigger;
    uint64		 lDataBufLen;
    int16        *pvBuffer;

    int32        lStatus;
    int32        lNotifySize;
    uint         dwErr;

    lSegmentsize = ALINEDEPTH;
    lPosttrigger = ALINEDEPTH - 16;
    lPretrigger  = 16;
    lDataBufLen  = ALINEDEPTH * ALINECOUNT * 2;
    lNotifySize  = ALINEDEPTH * ALINECOUNT * 2 / 5;

    drv_handle hDrv = spcm_hOpen ("/dev/spcm0");
    spcm_dwSetParam_i32 (hDrv, SPC_PATH0,            0);
    spcm_dwSetParam_i32 (hDrv, SPC_AMP0,             ui->comboBox_2->currentText().toInt());
    spcm_dwSetParam_i32 (hDrv, SPC_OFFS0,            0);

    spcm_dwSetParam_i32 (hDrv, SPC_CARDMODE,         SPC_REC_FIFO_MULTI);
    spcm_dwSetParam_i32 (hDrv, SPC_CHENABLE,         CHANNEL0);

    spcm_dwSetParam_i64 (hDrv, SPC_SEGMENTSIZE,      lSegmentsize);
    spcm_dwSetParam_i64 (hDrv, SPC_POSTTRIGGER,      lPosttrigger);
    spcm_dwSetParam_i64 (hDrv, SPC_PRETRIGGER,       lPretrigger);
    spcm_dwSetParam_i64 (hDrv, SPC_LOOPS,            0);

    spcm_dwSetParam_i32 (hDrv, SPC_CLOCKMODE,        SPC_CM_INTPLL);
    spcm_dwSetParam_i32 (hDrv, SPC_CLOCKOUT,         0);
    spcm_dwSetParam_i64 (hDrv, SPC_SAMPLERATE,       SampleRate * 1000000);
    spcm_dwSetParam_i64 (hDrv, SPC_REFERENCECLOCK,   10000000);

    spcm_dwSetParam_i32 (hDrv, SPC_TRIG_TERM,        0);
    spcm_dwSetParam_i32 (hDrv, SPC_TRIG_EXT0_ACDC,   0);
    spcm_dwSetParam_i32 (hDrv, SPC_TRIG_EXT0_MODE,   1);
    spcm_dwSetParam_i32 (hDrv, SPC_TRIG_EXT0_LEVEL0, 2000);
    spcm_dwSetParam_i32 (hDrv, SPC_TRIG_EXT0_LEVEL1, 0);
    spcm_dwSetParam_i32 (hDrv, SPC_TRIG_ORMASK,      2);
    spcm_dwSetParam_i32 (hDrv, SPC_TRIG_ANDMASK,     0);
    spcm_dwSetParam_i32 (hDrv, SPC_TRIG_CH_ORMASK0,  0);
    spcm_dwSetParam_i32 (hDrv, SPC_TRIG_CH_ANDMASK0, 0);
    spcm_dwSetParam_i32 (hDrv, SPC_TRIG_DELAY,       0);

    pvBuffer = static_cast<int16*>(pvAllocMemPageAligned(lDataBufLen));

    spcm_dwDefTransfer_i64 (hDrv, SPCM_BUF_DATA, SPCM_DIR_CARDTOPC, static_cast<uint32>(lNotifySize), pvBuffer, 0, lDataBufLen);
    dwErr = spcm_dwSetParam_i32 (hDrv, SPC_M2CMD, M2CMD_CARD_START | M2CMD_CARD_ENABLETRIGGER | M2CMD_DATA_STARTDMA);

    if(dwErr != ERR_OK)
    {
        vFreeMemPageAligned (pvBuffer, lDataBufLen);
        spcm_vClose(hDrv);
    }
    else
    {
        int i = 0;
        while(ImagingFlag)
        {
            if((dwErr = spcm_dwSetParam_i32 (hDrv, SPC_M2CMD, M2CMD_DATA_WAITDMA)) != ERR_OK) break;
            else
            {
                spcm_dwGetParam_i32 (hDrv, SPC_M2STATUS, &lStatus);
                //spcm_dwGetParam_i64 (hDrv, SPC_DATA_AVAIL_USER_LEN, &lDataAvailBytes);
                //spcm_dwGetParam_i64 (hDrv, SPC_DATA_AVAIL_USER_POS, &lAvailpos);

                if(lStatus & M2STAT_DATA_BLOCKREADY)
                {
                    if((++i % 5) == 0) Q1.enqueue_bulk(pvBuffer, BSCAN);
                    spcm_dwSetParam_i32 (hDrv, SPC_DATA_AVAIL_CARD_LEN, lNotifySize);
                }
            }
            if(i == BSCANCOUNT*5) break;
        }
    }
    spcm_dwSetParam_i32 (hDrv, SPC_M2CMD, M2CMD_CARD_STOP | M2CMD_DATA_STOPDMA);
    vFreeMemPageAligned (pvBuffer, lDataBufLen);
    spcm_vClose(hDrv);
}

void Widget::DataProcess()
{
    int r = 0;
    while(ImagingFlag)
    {
        if(Q1.try_dequeue_bulk(data1, BSCAN))
        {
            int16 *AlineMax = new int16[5*ALINECOUNT]();
            for(int m = 0; m < ALINECOUNT; ++m)
            {
                int16 *AlineCut = new int16[static_cast<unsigned int64>(CutCount)]();
                BandPass(4, SampleRate, 20, 1, data1+m*ALINEDEPTH+CutPoint, static_cast<unsigned int64>(CutCount));
                memcpy(AlineCut, data1+m*ALINEDEPTH+CutPoint, static_cast<unsigned int64>(CutCount)*sizeof(int16));

                AlineMax[5*m]   = AlineCut[0];
                AlineMax[5*m+1] = AlineCut[SectionOri];
                AlineMax[5*m+2] = AlineCut[SectionOri+  (SectionEnd-SectionOri+1)/4];
                AlineMax[5*m+3] = AlineCut[SectionOri+2*(SectionEnd-SectionOri+1)/4];
                AlineMax[5*m+4] = AlineCut[SectionOri+3*(SectionEnd-SectionOri+1)/4];

                for(int n = 0; n < CutCount; ++n) if(AlineMax[5*m] < AlineCut[n]) AlineMax[5*m] = AlineCut[n];
                for(int n = 0; n < (SectionEnd-SectionOri+1)/4; ++n) if(AlineMax[5*m+1] < AlineCut[SectionOri+n]) AlineMax[5*m+1] = AlineCut[SectionOri+n];
                for(int n = 0; n < (SectionEnd-SectionOri+1)/4; ++n) if(AlineMax[5*m+2] < AlineCut[SectionOri+  (SectionEnd-SectionOri+1)/4+n]) AlineMax[5*m+2] = AlineCut[SectionOri+(SectionEnd-SectionOri+1)/4+n];
                for(int n = 0; n < (SectionEnd-SectionOri+1)/4; ++n) if(AlineMax[5*m+3] < AlineCut[SectionOri+2*(SectionEnd-SectionOri+1)/4+n]) AlineMax[5*m+3] = AlineCut[SectionOri+2*(SectionEnd-SectionOri+1)/4+n];
                for(int n = 0; n < (SectionEnd-SectionOri+1)/4; ++n) if(AlineMax[5*m+4] < AlineCut[SectionOri+3*(SectionEnd-SectionOri+1)/4+n]) AlineMax[5*m+4] = AlineCut[SectionOri+3*(SectionEnd-SectionOri+1)/4+n];

                delete [] AlineCut;
            }
            Q2.enqueue_bulk(data1, BSCAN);
            Q3.enqueue_bulk(AlineMax, 5*ALINECOUNT);
            delete [] AlineMax;
            ++r;
        }
        if(r == BSCANCOUNT) break;
    }
}

void Widget::BscanDisplay()
{
    int r = 0;
    QVector<QRgb> grayTable;
    for(int i = 0; i < 256; ++i) grayTable.push_back(qRgb(i, 0, 0));

    Mat Bscan(ALINECOUNT, CutCount, CV_8UC1, Scalar::all(0));

    while(ImagingFlag)
    {
        if(Q2.try_dequeue_bulk(data2, BSCAN))
        {
            for(int t = 0; t < ALINECOUNT; ++t)
            {
                uchar *BscanRow = Bscan.ptr<uchar>(t);
                for(int c = 0; c < CutCount; ++c) BscanRow[c] = saturate_cast<uchar>(data2[t*ALINEDEPTH+CutPoint+c]/64);
            }
//            if((r%2)==1) flip(Bscan, Bscan, 0);
            if((++r%100)==1)
            {
                Mat BscanDisplay = Bscan;
                rotate(BscanDisplay, BscanDisplay, ROTATE_90_CLOCKWISE);
                QImage BscanDisplay1 = QImage(BscanDisplay.data, BscanDisplay.cols, BscanDisplay.rows, QImage::Format_Indexed8);
                BscanDisplay1.setColorTable(grayTable);
                ui->label_7->setPixmap(QPixmap::fromImage(BscanDisplay1));
            }
            if(ui->checkBox->isChecked())
            {
                QString DataSavePath = ui->textEdit->toPlainText() + "/Bscan_" + QString::number(r) + ".dat";
                FILE* OutFile;
                fopen_s(&OutFile, DataSavePath.toLatin1(), "wb");
                fwrite(data2, sizeof(int16), BSCAN, OutFile);
                fclose(OutFile);

                QString BscanSavePath = ui->textEdit->toPlainText() + "/Bscan_" + QString::number(r) + ".png";
                Mat BscanSave = Bscan;
                rotate(BscanSave, BscanSave, ROTATE_90_CLOCKWISE);
                imwrite(BscanSavePath.toStdString(), BscanSave);
            }
        }
        if(r == BSCANCOUNT) break;
    }
}

void Widget::MapDisplay()
{
    int r1 = 0;

    QVector<QRgb> grayTable;
    for(int i = 0; i < 256; ++i) grayTable.push_back(qRgb(i, 0, 0));

    while(ImagingFlag)
    {
        if(Q3.try_dequeue_bulk(data3, 5*ALINECOUNT))
        {
            static Mat XYMAP(  BSCANCOUNT, ALINECOUNT, CV_8UC1, Scalar::all(0));
            static Mat XYMAP01(BSCANCOUNT, ALINECOUNT, CV_8UC1, Scalar::all(0));
            static Mat XYMAP12(BSCANCOUNT, ALINECOUNT, CV_8UC1, Scalar::all(0));
            static Mat XYMAP23(BSCANCOUNT, ALINECOUNT, CV_8UC1, Scalar::all(0));
            static Mat XYMAP34(BSCANCOUNT, ALINECOUNT, CV_8UC1, Scalar::all(0));

            uchar *MAPRow   = XYMAP.ptr<uchar>(r1);
            uchar *MAPRow01 = XYMAP01.ptr<uchar>(r1);
            uchar *MAPRow12 = XYMAP12.ptr<uchar>(r1);
            uchar *MAPRow23 = XYMAP23.ptr<uchar>(r1);
            uchar *MAPRow34 = XYMAP34.ptr<uchar>(r1);
            for(int c = 0; c < ALINECOUNT; ++c)
            {
                MAPRow[c]   = saturate_cast<uchar>(data3[5*c]/64);
                MAPRow01[c] = saturate_cast<uchar>(data3[5*c+1]/64);
                MAPRow12[c] = saturate_cast<uchar>(data3[5*c+2]/64);
                MAPRow23[c] = saturate_cast<uchar>(data3[5*c+3]/64);
                MAPRow34[c] = saturate_cast<uchar>(data3[5*c+4]/64);
            }
            if((++r1%10)==0)
            {
//                cv::resize(XYMAP, XYMAP, Size(600,600), INTER_CUBIC);
                QImage MAPDisplay1 = QImage(XYMAP.data, XYMAP.cols, XYMAP.rows, QImage::Format_Indexed8);
                MAPDisplay1.setColorTable(grayTable);
                ui->label_2->setPixmap(QPixmap::fromImage(MAPDisplay1));
                QImage MAPDisplay01 = QImage(XYMAP01.data, XYMAP01.cols, XYMAP01.rows, QImage::Format_Indexed8);
                MAPDisplay01.setColorTable(grayTable);
                ui->label_3->setPixmap(QPixmap::fromImage(MAPDisplay01));
                QImage MAPDisplay12 = QImage(XYMAP12.data, XYMAP12.cols, XYMAP12.rows, QImage::Format_Indexed8);
                MAPDisplay12.setColorTable(grayTable);
                ui->label_4->setPixmap(QPixmap::fromImage(MAPDisplay12));
                QImage MAPDisplay23 = QImage(XYMAP23.data, XYMAP23.cols, XYMAP23.rows, QImage::Format_Indexed8);
                MAPDisplay23.setColorTable(grayTable);
                ui->label_5->setPixmap(QPixmap::fromImage(MAPDisplay23));
                QImage MAPDisplay34 = QImage(XYMAP34.data, XYMAP34.cols, XYMAP34.rows, QImage::Format_Indexed8);
                MAPDisplay34.setColorTable(grayTable);
                ui->label_6->setPixmap(QPixmap::fromImage(MAPDisplay34));
            }
        }
        if(r1 == BSCANCOUNT) break;
    }
}

void Widget::VolumeRender()
{
    QString FileNamePrefix = "/Bscan_";
    QString FilePath = ui->textEdit->toPlainText() + FileNamePrefix;

    vtkPNGReader *reader = vtkPNGReader::New();
    reader->SetFilePrefix(FilePath.toStdString().c_str());
    reader->SetFilePattern("%s%d.png");
    reader->SetDataExtent(0,299,0,CutCount-1,1,200);
    reader->Update();

    vtkGPUVolumeRayCastMapper *mapper = vtkGPUVolumeRayCastMapper::New();
    mapper->SetInputData(reader->GetOutput());

    vtkVolume *volume = vtkVolume::New();
    volume->SetMapper(mapper);

    vtkVolumeProperty *property = vtkVolumeProperty::New();

    vtkPiecewiseFunction *popacity = vtkPiecewiseFunction::New();
    popacity->AddPoint(ui->verticalSlider_2->value(), 0.0);
    popacity->AddPoint(255, 1);

    vtkColorTransferFunction *color = vtkColorTransferFunction::New();
    bool Flag = false;
    if(Flag)
    {
        color->AddHSVPoint(10, 1/5, 0.73,  0.55);
        color->AddHSVPoint(25, 1/4, 0.73,  0.55, 0.5, 0.92);
        color->AddHSVPoint(40, 1/3, 0.67,  0.88);
        color->AddHSVPoint(55, 1/2, 0.67,  0.88, 0.33, 0.45);
        color->AddHSVPoint(70,   1, 0.063, 1.0);
    }
    else
    {
        color->AddHSVPoint(255/5, 1/5, 0.73,  0.55);
        color->AddHSVPoint(255/4, 1/4, 0.73,  0.55, 0.5, 0.92);
        color->AddHSVPoint(255/3, 1/3, 0.67,  0.88);
        color->AddHSVPoint(255/2, 1/2, 0.67,  0.88, 0.33, 0.45);
        color->AddHSVPoint(255,   1,   0.063, 1.0);
    }

    property->SetColor(color);
    property->SetScalarOpacity(popacity);
    property->ShadeOn();
    property->SetInterpolationTypeToLinear();
    property->SetShade(0, 1);
    property->SetDiffuse(1);
    property->SetAmbient(0.5);
    property->SetSpecular(0.2);
    property->SetSpecularPower(10);
    property->SetComponentWeight(0, 1);
    property->SetDisableGradientOpacity(1);
    property->DisableGradientOpacityOn();
    property->SetScalarOpacityUnitDistance(1);


    volume->SetProperty(property);

    vtkRenderer *render = vtkRenderer::New();
    render->AddActor(volume);
    render->SetBackground(0, 0, 0);

    vtkRenderWindow *renWin = vtkRenderWindow::New();
    renWin->AddRenderer(render);

    ui->qvtkWidget->SetRenderWindow(renWin);
}

QByteArray Widget::StringToHex(QString str)
{
    QByteArray send_data;
    int hex_data, low_hex_data;
    int hex_data_len = 0;
    int len = str.length();
    send_data.resize(len / 2);
    char l_str, h_str;

    for(int i = 0; i < len; )
    {
        h_str=str[i].toLatin1();
        if(h_str == ' ')
        {
            i++;
            continue;
        }
        i++;
        if(i >= len)
            break;
        l_str = str[i].toLatin1();
        hex_data = ConvertHexChar(h_str);
        low_hex_data = ConvertHexChar(l_str);
        if((hex_data == 16) || (low_hex_data == 16))
            break;
        else
            hex_data = hex_data*16+low_hex_data;
        i++;
        send_data[hex_data_len] = static_cast<char>(hex_data);
        hex_data_len++;
    }

    send_data.resize(hex_data_len);
    return send_data;
}

char Widget::ConvertHexChar(char ch)
{
    if((ch >= '0') && (ch <= '9'))
            return ch-0x30;
        else if((ch >= 'A') && (ch <= 'F'))
            return ch-'A'+10;
        else if((ch >= 'a') && (ch <= 'f'))
            return ch-'a'+10;
        else return (-1);
}
