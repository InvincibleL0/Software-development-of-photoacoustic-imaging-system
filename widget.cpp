#include "widget.h"
#include "ui_widget.h"

#define LASER "COM2"
#define FPGA "COM3"
#define MOTOR 23

static const int trigFre = 10000;
static const int alineDepth = 4096;

static const int cutPoint = 900;
static const int cutCount = 500;
static const int sampleRate = 200;
static int bscanCount = 0;
static int alineCount = 0;
static int meanTimes = 0;

static bool alineDebugFlag = false;
static bool imageingFlag = false;

static moodycamel::ConcurrentQueue<short> Q1(1);
static moodycamel::ConcurrentQueue<short> Q2(1);
static moodycamel::ConcurrentQueue<short> Q3(1);
static moodycamel::ConcurrentQueue<short> Q4(1);

static vector<vector<vector<short>>> xyzData;

static Mat org, img, tmp, dst;//MVD
static vector<double> diamRes, tortRes;
void diamTort(vector<double>& diamRes, vector<double>& tortRes, Mat& src);
static vector<double> paraLog;

typedef int (*FUN1)(int,int);//DLL_OpenCom
typedef void (*FUN2)(unsigned int,unsigned char);//ControlStatusSet
typedef char (*FUN3)(unsigned int,int);//PositionAbsoluteMove
typedef void (*FUN4)(unsigned int);//PaceSet
typedef char (*FUN5)(unsigned int,unsigned int);//VelocitySet
typedef void (*FUN6)(int,int,int,int);//ScanRangeSet
typedef void (*FUN7)(void);//BeginScan
typedef void (*FUN8)(void);//StopScan

/*
函数功能：widget类构造函数
参数说明：初始化列表
返回值：
*/
Widget::Widget(QWidget *parent) : QWidget(parent), ui(new Ui::Widget)
{
    ui->setupUi(this);
//////////////////////////////////////////////////////////////////////////////////////////////////////
    //Aline波形显示初始化
    m_chart1 = new QChart;
    QChartView *chartView1 = new QChartView(m_chart1);
    chartView1->setRenderHint(QPainter::Antialiasing);
    m_series1 = new QLineSeries;

    QValueAxis *axisX1 = new QValueAxis;
    axisX1->setLabelFormat("%d");
    axisX1->setRange(0, 400);
    axisX1->setTickCount(6);
    axisX1->setTitleText("Samples");
    axisX1->setTitleFont(QFont("Arial Rounded MT Bold", 8));

    QValueAxis *axisY = new QValueAxis;
    axisY->setLabelFormat("%.1f");
    axisY->setRange(-1, 1);
    axisY->setTickCount(5);
    axisY->setTitleText("Amplitude / V");
    axisY->setTitleFont(QFont("Arial Rounded MT Bold", 8));

    m_chart1->addAxis(axisX1, Qt::AlignBottom);
    m_chart1->addAxis(axisY, Qt::AlignLeft);
    m_chart1->addSeries(m_series1);

    m_series1->attachAxis(axisX1);
    m_series1->attachAxis(axisY);
    m_series1->setPen(QPen(Qt::red, 1.5, Qt::SolidLine));

    m_chart1->legend()->hide();
    ui->gridLayout->addWidget(chartView1);
    //血管参数统计结果显示初始化
    m_chart = new QChart;
    QChartView *chartView = new QChartView(m_chart);
    chartView->setRenderHint(QPainter::Antialiasing);
    m_series = new QBarSeries;
    m_chart->addSeries(m_series);
    m_chart->setAnimationOptions(QChart::SeriesAnimations);

    m_chart->legend()->setVisible(true);
    m_chart->legend()->setAlignment(Qt::AlignTop);

    QValueAxis *axisY2 = new QValueAxis;
    axisY2->setLabelFormat("%.1f");
    axisY2->setRange(0, 1);
    axisY2->setTickCount(6);
    axisY2->setTitleText("比例");
    m_chart->setAxisY(axisY2, m_series);

    ui->gridLayout_2->addWidget(chartView);
    m_series->append(set0);
    m_series->append(set1);
    m_series->setLabelsPosition(QAbstractBarSeries::LabelsOutsideEnd);
    m_series->setLabelsFormat("%.3f");
    m_series->setLabelsAngle(90);
    m_series->setLabelsVisible(false);
//////////////////////////////////////////////////////////////////////////////////////////////////////
    initPort();
    //serialPortOpen1();
    serialPortOpen2();
    serialPortOpen3();
    lookUpTable();

    connect(ui->pushButton, &QPushButton::clicked, [=]{savePathSelect();});
    connect(ui->pushButton_2, &QPushButton::clicked, [=]
    {
        alineDebugFlag = !alineDebugFlag;
        startAlineDebug();
        QtConcurrent::run(this, &Widget::alineDebug);
        QtConcurrent::run(this, &Widget::drawCharts);
    });
    connect(ui->pushButton_3, &QPushButton::clicked, [=]
    {
        imageingFlag = !imageingFlag;
        alineCount = fixAlineCount();
        bscanCount = fixBscanCount();
        meanTimes = ui->comboBox_7->currentText().toInt();;
        writeLog();
        startFullScan();
        QtConcurrent::run(this, &Widget::acquisitionStart);
        QtConcurrent::run(this, &Widget::dataProcess);
        QtConcurrent::run(this, &Widget::bscanDisplay);
        QtConcurrent::run(this, &Widget::mapDisplay);
    });
    connect(ui->pushButton_4, &QPushButton::clicked, [=]{backToZero();});
    connect(ui->pushButton_5, &QPushButton::clicked, [=]{volumeRender();});
    connect(ui->pushButton_6, &QPushButton::clicked, [=]{saveVTKwidget();});
    connect(ui->pushButton_11, &QPushButton::clicked, [=]{savePathSelect2();});
    connect(ui->pushButton_13, &QPushButton::clicked, [=]
    {
        QtConcurrent::run(this, &Widget::readDat);
    });
    connect(ui->pushButton_14, &QPushButton::clicked, [=]
    {
        namedWindow("Select Rectangular Area");
        setMouseCallback("Select Rectangular Area", onMouse1, nullptr);
        imshow("Select Rectangular Area", org);
    });
    connect(ui->pushButton_10, &QPushButton::clicked, [=]
    {
        namedWindow("Select Area");
        setMouseCallback("Select Area", onMouse2, nullptr);
        imshow("Select Area", org);
    });
    connect(ui->pushButton_9, &QPushButton::clicked, [=]{drawBarGraph();});
    connect(ui->verticalSlider_DS_1, &QSlider::valueChanged, [=]{depthChange();});
    connect(ui->verticalSlider_DS_2, &QSlider::valueChanged, [=]{depthChange();});
    connect(ui->verticalSlider_Con, &QSlider::valueChanged, [=]{contrastChange();});
    connect(ui->verticalSlider_Lum, &QSlider::valueChanged, [=]{contrastChange();});
    connect(ui->pushButton_save, &QPushButton::clicked, [=]{saveImage();});
    connect(ui->comboBox_colormap, &QComboBox::currentTextChanged, [=]{lookUpTable();});
}

/*
函数功能：widget类析构函数
参数说明：
返回值：
*/
Widget::~Widget()
{
    delete ui;
}

/****************************************************************扫描采集相关函数****************************************************************/
/*
函数功能：采集Aline，入队列4
参数说明：
返回值：
*/
void Widget::alineDebug()
{
    int        lSegmentsize, lPretrigger, lPosttrigger;
    uint64		 lDataBufLen;
    short        *pvBuffer;

    int32        lStatus;
    int32        lNotifySize;
    uint         dwErr;

    lSegmentsize = alineDepth;
    lPosttrigger = alineDepth - 16;
    lPretrigger  = 16;
    lDataBufLen  = static_cast<uint64>(alineDepth*500*2);
    lNotifySize  = alineDepth*500*2;

    drv_handle hDrv = spcm_hOpen ("/dev/spcm0");
    spcm_dwSetParam_i32 (hDrv, SPC_PATH0,            0);
    spcm_dwSetParam_i32 (hDrv, SPC_AMP0,             2000);
    spcm_dwSetParam_i32 (hDrv, SPC_OFFS0,            0);

    spcm_dwSetParam_i32 (hDrv, SPC_CARDMODE,         SPC_REC_FIFO_MULTI);
    spcm_dwSetParam_i32 (hDrv, SPC_CHENABLE,         CHANNEL0);

    spcm_dwSetParam_i64 (hDrv, SPC_SEGMENTSIZE,      lSegmentsize);
    spcm_dwSetParam_i64 (hDrv, SPC_POSTTRIGGER,      lPosttrigger);
    spcm_dwSetParam_i64 (hDrv, SPC_PRETRIGGER,       lPretrigger);
    spcm_dwSetParam_i64 (hDrv, SPC_LOOPS,            0);

    spcm_dwSetParam_i32 (hDrv, SPC_CLOCKMODE,        SPC_CM_INTPLL);
    spcm_dwSetParam_i32 (hDrv, SPC_CLOCKOUT,         0);
    spcm_dwSetParam_i64 (hDrv, SPC_SAMPLERATE,       sampleRate * 1000000);
    spcm_dwSetParam_i64 (hDrv, SPC_REFERENCECLOCK,   10000000);
/*
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
*/
    //m3i
    spcm_dwSetParam_i32 (hDrv, 47200,                SPCM_XMODE_TRIGIN);
    spcm_dwSetParam_i32 (hDrv, SPC_TRIG_ORMASK,      SPC_TMASK_EXT1);
    spcm_dwSetParam_i32 (hDrv, SPC_TRIG_EXT1_MODE,   SPC_TM_POS);
    spcm_dwSetParam_i32 (hDrv, SPC_TRIG_ANDMASK,     0);

    pvBuffer = static_cast<short*>(pvAllocMemPageAligned(lDataBufLen));

    spcm_dwDefTransfer_i64 (hDrv, SPCM_BUF_DATA, SPCM_DIR_CARDTOPC, static_cast<uint32>(lNotifySize), pvBuffer, 0, lDataBufLen);
    dwErr = spcm_dwSetParam_i32 (hDrv, SPC_M2CMD, M2CMD_CARD_START | M2CMD_CARD_ENABLETRIGGER | M2CMD_DATA_STARTDMA);

    if(dwErr != ERR_OK)
    {
        vFreeMemPageAligned (pvBuffer, lDataBufLen);
        spcm_vClose(hDrv);
    }
    else
    {
        while(alineDebugFlag)
        {
            if((dwErr = spcm_dwSetParam_i32 (hDrv, SPC_M2CMD, M2CMD_DATA_WAITDMA)) != ERR_OK) break;
            else
            {
                spcm_dwGetParam_i32 (hDrv, SPC_M2STATUS, &lStatus);

                if(lStatus & M2STAT_DATA_BLOCKREADY)
                {
                    Q4.enqueue_bulk(pvBuffer, alineDepth*500);
                    spcm_dwSetParam_i32 (hDrv, SPC_DATA_AVAIL_CARD_LEN, lNotifySize);
                }
            }
        }
    }
    spcm_dwSetParam_i32 (hDrv, SPC_M2CMD, M2CMD_CARD_STOP | M2CMD_DATA_STOPDMA);
    vFreeMemPageAligned (pvBuffer, lDataBufLen);
    spcm_vClose(hDrv);
}

/*
函数功能：出队列4，投影Aline波形
参数说明：
返回值：
*/
void Widget::drawCharts()
{
    short *data = new short[alineDepth*500]();
    int Point = 900;
    int alineDebugCount = 1024;
    while(alineDebugFlag)
    {
        if(Q4.try_dequeue_bulk(data, alineDepth*500))
        {
            float *xreal = new float[alineDebugCount]();
            for(int i = 0; i < alineDebugCount; ++i)
            {
                xreal[i] = static_cast<float>(data[Point+i]) / 8192;
                m_data1.append(QPointF(i, xreal[i]));
            }
            m_series1->replace(m_data1);
            m_data1.clear();
            delete [] xreal;
        }
    }
    delete [] data;
}

/*
函数功能：采集Bscan，入队列1
参数说明：
返回值：
*/
void Widget::acquisitionStart()
{
    int32        lSegmentsize, lPretrigger, lPosttrigger;
    uint64		 lDataBufLen;
    short        *pvBuffer;

    int32        lStatus;
    int32        lNotifySize;
    uint         dwErr;

    lSegmentsize = alineDepth;
    lPosttrigger = alineDepth - 16;
    lPretrigger  = 16;
    lDataBufLen  = static_cast<uint64>(alineDepth*alineCount*2);
    lNotifySize  = alineDepth*alineCount*2;

    drv_handle hDrv = spcm_hOpen ("/dev/spcm0");
    spcm_dwSetParam_i32 (hDrv, SPC_PATH0,            0);
    spcm_dwSetParam_i32 (hDrv, SPC_AMP0,             2000);
    spcm_dwSetParam_i32 (hDrv, SPC_OFFS0,            0);

    spcm_dwSetParam_i32 (hDrv, SPC_CARDMODE,         SPC_REC_FIFO_MULTI);
    spcm_dwSetParam_i32 (hDrv, SPC_CHENABLE,         CHANNEL0);

    spcm_dwSetParam_i64 (hDrv, SPC_SEGMENTSIZE,      lSegmentsize);
    spcm_dwSetParam_i64 (hDrv, SPC_POSTTRIGGER,      lPosttrigger);
    spcm_dwSetParam_i64 (hDrv, SPC_PRETRIGGER,       lPretrigger);
    spcm_dwSetParam_i64 (hDrv, SPC_LOOPS,            0);

    spcm_dwSetParam_i32 (hDrv, SPC_CLOCKMODE,        SPC_CM_INTPLL);
    spcm_dwSetParam_i32 (hDrv, SPC_CLOCKOUT,         0);
    spcm_dwSetParam_i64 (hDrv, SPC_SAMPLERATE,       sampleRate * 1000000);
    spcm_dwSetParam_i64 (hDrv, SPC_REFERENCECLOCK,   10000000);
/*
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
*/
    //m3i
    spcm_dwSetParam_i32 (hDrv, 47200,                SPCM_XMODE_TRIGIN);
    spcm_dwSetParam_i32 (hDrv, SPC_TRIG_ORMASK,      SPC_TMASK_EXT1);
    spcm_dwSetParam_i32 (hDrv, SPC_TRIG_EXT1_MODE,   SPC_TM_POS);
    spcm_dwSetParam_i32 (hDrv, SPC_TRIG_ANDMASK,     0);

    pvBuffer = static_cast<short*>(pvAllocMemPageAligned(lDataBufLen));

    dwErr = spcm_dwDefTransfer_i64 (hDrv, SPCM_BUF_DATA, SPCM_DIR_CARDTOPC, static_cast<uint32>(lNotifySize), pvBuffer, 0, lDataBufLen);
    dwErr = spcm_dwSetParam_i32 (hDrv, SPC_M2CMD, M2CMD_CARD_START | M2CMD_CARD_ENABLETRIGGER | M2CMD_DATA_STARTDMA);

    if(dwErr != ERR_OK)
    {
        vFreeMemPageAligned (pvBuffer, lDataBufLen);
        spcm_vClose(hDrv);
    }
    else
    {
        int i = 0;
        while(imageingFlag)
        {
            if((dwErr = spcm_dwSetParam_i32 (hDrv, SPC_M2CMD, M2CMD_DATA_WAITDMA)) != ERR_OK) break;
            else
            {
                spcm_dwGetParam_i32 (hDrv, SPC_M2STATUS, &lStatus);

                if(lStatus & M2STAT_DATA_BLOCKREADY)
                {
                    if(++i) Q1.enqueue_bulk(pvBuffer, alineDepth*alineCount);
                    spcm_dwSetParam_i32 (hDrv, SPC_DATA_AVAIL_CARD_LEN, lNotifySize);
                }
            }
            if(i == bscanCount) break;
        }
    }
    HINSTANCE hdll;
    hdll = LoadLibraryA("SerialCom.dll");
    FUN8 StopScan;
    StopScan = (FUN8)GetProcAddress(hdll, "StopScan");
    StopScan();
    spcm_dwSetParam_i32 (hDrv, SPC_M2CMD, M2CMD_CARD_STOP | M2CMD_DATA_STOPDMA);
    vFreeMemPageAligned (pvBuffer, lDataBufLen);
    spcm_vClose(hDrv);
}

/*
函数功能：出队列1，取Cscan中值入队列2，取Aline最大值入队列3
参数说明：
返回值：
*/
void Widget::dataProcess()
{
    int row = 0;
    int meanTimes = ui->comboBox_7->currentText().toInt();
    int alineCountMean = alineCount/meanTimes;
    short *data1 = new short[alineDepth*alineCount]();
    short *data1cut = new short[cutCount*alineCount]();
    short *AlineCut = new short[cutCount*meanTimes]();
    short *BscanMean = new short[cutCount*alineCountMean]();
    short *Cline = new short[meanTimes]();
    short *AlineMax = new short[alineCountMean]();
    while(imageingFlag)
    {
        if(Q1.try_dequeue_bulk(data1, alineDepth*alineCount))
        {
            for(int i = 0; i < alineCount; ++i)
            {
                memcpy(data1cut+i*cutCount, data1+i*alineDepth+cutPoint, cutCount*sizeof(short));
            }
            for(int i = 0; i < alineCountMean; ++i)
            {
                memcpy(AlineCut, data1cut+cutCount*meanTimes*i, cutCount*meanTimes*sizeof(short));
                for(int j = 0; j < cutCount; ++j)
                {
                    for(int k = 0; k < meanTimes; ++k)
                    {
                        Cline[k] = AlineCut[cutCount*k+j];
                    }
                    BscanMean[cutCount*i+j] = midValue(Cline, meanTimes);
                }
                AlineMax[i] = BscanMean[cutCount*i+cutPoint];
                for(int j = 0; j < cutCount; ++j) if(AlineMax[i] < BscanMean[cutCount*i+j]) AlineMax[i] = BscanMean[cutCount*i+j];
            }
            Q2.enqueue_bulk(BscanMean, cutCount*alineCountMean);
            Q3.enqueue_bulk(AlineMax, alineCountMean);
            ++row;
        }
        if(row == bscanCount) break;
    }
    delete [] data1;
    delete [] data1cut;
    delete [] AlineCut;
    delete [] BscanMean;
    delete [] Cline;
    delete [] AlineMax;
}

/*
函数功能：出队列2，Bscan投影、存dat & png
参数说明：
返回值：
*/
void Widget::bscanDisplay()
{
    int r1 = 0, r2 = 0;

    int alineCountMean = alineCount/meanTimes;
    short *data2 = new short[cutCount*alineCountMean]();

    while(imageingFlag)
    {
        if(Q2.try_dequeue_bulk(data2, cutCount*alineCountMean))
        {
            Mat Bscan(alineCountMean, cutCount, CV_8UC1, Scalar::all(0));
            Mat BscanDisplay, BscanSave;
            for(int i = 0; i < Bscan.rows; ++i)
            {
                uchar *BscanRow = Bscan.ptr<uchar>(i);
                for(int j = 0; j < Bscan.cols; ++j) BscanRow[j] = saturate_cast<uchar>(ui->verticalSlider_2->value()*(data2[cutCount*i+j]/64)+ui->verticalSlider_3->value());
            }
            if(r1%2) flip(Bscan, Bscan, 0);
            rotate(Bscan, BscanDisplay, ROTATE_90_CLOCKWISE);
            cv::resize(BscanDisplay, BscanDisplay, Size(850, 500), INTER_CUBIC);
            cv::resize(BscanDisplay, BscanSave, Size(bscanCount/2, cutCount), INTER_LINEAR);

            applyColorMap(BscanDisplay, BscanDisplay, ui->comboBox_colormap->currentIndex());
            cvtColor(BscanDisplay, BscanDisplay, COLOR_BGR2RGB);
            QImage BscanDisplay1 = QImage(BscanDisplay.data, BscanDisplay.cols, BscanDisplay.rows, QImage::Format_RGB888);
            ui->label->setPixmap(QPixmap::fromImage(BscanDisplay1));
            if((r1%2)==0)
            {
                QString PNGSavePath = ui->textEdit->toPlainText() + "/Bscan_" + QString::number(r2+1) + ".png";
                imwrite(PNGSavePath.toStdString(), BscanSave);

                QString DATSavePath = ui->textEdit->toPlainText() + "/Bscan_" + QString::number(r2+1) + ".dat";
                FILE* OutFile;
                fopen_s(&OutFile, DATSavePath.toLatin1(), "wb");
                fwrite(data2, sizeof(short), cutCount*alineCountMean, OutFile);
                fclose(OutFile);

                ++r2;
            }
            ++r1;
        }
        if(r1 == bscanCount) break;
    }
    delete [] data2;
}

/*
函数功能：出队列3，最大值投影，存png
参数说明：
返回值：
*/
void Widget::mapDisplay()
{
    int r1 = 0, r2 = 0;

    int alineCountMean = alineCount/meanTimes;
    short *data3 = new short[alineCountMean]();
    QString MAPSavePath = ui->textEdit->toPlainText() + "/xyMap" + ".png";
    Mat XYMAP(bscanCount/2, alineCountMean, CV_8UC1, Scalar::all(0));
    Mat XYMAPDisplay;
    while(imageingFlag)
    {
        if(Q3.try_dequeue_bulk(data3, alineCountMean))
        {
            if(r1%2 == 0)
            {
                uchar *XYMAPRow = XYMAP.ptr<uchar>(r2++);
                for(int i = 0; i < alineCountMean; ++i) XYMAPRow[i] = saturate_cast<uchar>(ui->verticalSlider_2->value()*(data3[alineCountMean-1-i]/64)+ui->verticalSlider_3->value());
                cv::resize(XYMAP, XYMAPDisplay, Size(980,980), INTER_CUBIC);
                applyColorMap(XYMAPDisplay, XYMAPDisplay, ui->comboBox_colormap->currentIndex());
                cvtColor(XYMAPDisplay, XYMAPDisplay, COLOR_BGR2RGB);
                QImage MAPDisplay = QImage(XYMAPDisplay.data, XYMAPDisplay.cols, XYMAPDisplay.rows, QImage::Format_RGB888);
                ui->label_2->setPixmap(QPixmap::fromImage(MAPDisplay));

                imwrite(MAPSavePath.toStdString(), XYMAP);
            }
            ++r1;
        }
        if(r1 == bscanCount)
        {
            applyColorMap(XYMAP, XYMAP, ui->comboBox_colormap->currentIndex());
            imwrite(MAPSavePath.toStdString(), XYMAP);
            break;
        }
    }
}

/*
函数功能：用Bscan的png进行三维体重建
参数说明：
返回值：
*/
void Widget::volumeRender()
{
    QString FileNamePrefix = "/Bscan_";
    QString FilePath = ui->textEdit->toPlainText() + FileNamePrefix;

    vtkPNGReader *reader = vtkPNGReader::New();
    reader->SetFilePrefix(FilePath.toStdString().c_str());
    reader->SetFilePattern("%s%d.png");
    reader->SetDataExtent(0, alineCount/ui->comboBox_7->currentText().toInt(), 0, cutCount, 1, bscanCount/2);
    reader->Update();

    vtkGPUVolumeRayCastMapper *mapper = vtkGPUVolumeRayCastMapper::New();
    mapper->SetInputData(reader->GetOutput());

    vtkVolume *volume = vtkVolume::New();
    volume->SetMapper(mapper);

    vtkVolumeProperty *property = vtkVolumeProperty::New();

    vtkPiecewiseFunction *popacity = vtkPiecewiseFunction::New();
    popacity->AddPoint(ui->verticalSlider->value(), 0.0);
    popacity->AddPoint(255, 1);

    vtkColorTransferFunction *color = vtkColorTransferFunction::New();
    bool flag = false;
    if(flag)
    {
        color->AddHSVPoint(10, 1/5, 0.73,  0.55);
        color->AddHSVPoint(25, 1/4, 0.73,  0.55, 0.5, 0.92);
        color->AddHSVPoint(40, 1/3, 0.67,  0.88);
        color->AddHSVPoint(55, 1/2, 0.67,  0.88, 0.33, 0.45);
        color->AddHSVPoint(70,   1, 0.063, 1.0);
    }
    else
    {
        color->AddHSVPoint(255/5, 1/5, 0.8,  0.55);
        color->AddHSVPoint(255/4, 1/4, 0.8,  0.55);
        color->AddHSVPoint(255/3, 1/3, 0.8,  0.88);
        color->AddHSVPoint(255/2, 1/2, 0.8,  0.88);
        color->AddHSVPoint(255,   1,   0.8,  1.0);
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

    vtkCubeAxesActor *cube = vtkCubeAxesActor::New();
    cube->SetCamera(render->GetActiveCamera());
    cube->SetBounds(reader->GetOutput()->GetBounds());
    cube->SetXTitle("X");
    cube->SetYTitle("Z");
    cube->SetZTitle("Y");
    cube->XAxisTickVisibilityOff();
    cube->YAxisTickVisibilityOff();
    cube->ZAxisTickVisibilityOff();
    cube->SetXAxisRange(0,0);
    cube->SetYAxisRange(0,0);
    cube->SetZAxisRange(0,0);
    cube->SetFlyModeToStaticEdges();
    render->AddActor(cube);

    vtkRenderWindow *renWin = vtkRenderWindow::New();
    renWin->AddRenderer(render);

    ui->qvtkWidget->SetRenderWindow(renWin);
}

/*
函数功能：选择数据存储路径
参数说明：
返回值：
*/
void Widget::savePathSelect()
{
    ui->textEdit->clear();
    ui->textEdit_2->clear();
    QString SavePath = QFileDialog::getExistingDirectory(this, tr("Open Directory"), "/home", QFileDialog::ShowDirsOnly);
    ui->textEdit->append(SavePath);
    ui->textEdit_2->append(SavePath);
    ui->pushButton_13->setEnabled(true);
}

/*
函数功能：初始化电机、FPGA串口
参数说明：
返回值：
*/
void Widget::initPort()
{
    foreach(const QSerialPortInfo &info,QSerialPortInfo::availablePorts())
    {
        serialPort2.setPort(info);
        if(serialPort2.open(QIODevice::ReadWrite))
        {
            //ui->comboBox->addItem(serialPort1.portName());
            serialPort2.close();
        }
    }
}

/*
函数功能：打开激光器控制串口
参数说明：
返回值：
*/
void Widget::serialPortOpen1()
{
//    serialPort1.setPortName(LASER);
//    //serialPort1.setPortName(ui->comboBox->currentText());
//    if(serialPort1.open(QIODevice::ReadWrite))
//    {
//        serialPort1.setBaudRate(QSerialPort::Baud4800);
//        serialPort1.setDataBits(QSerialPort::Data8);
//        serialPort1.setParity(QSerialPort::NoParity);
//        serialPort1.setStopBits(QSerialPort::OneStop);
//        serialPort1.setFlowControl(QSerialPort::NoFlowControl);
//    }
//    else ui->textEdit_2->append(" Warnning 激光器连接失败 ");
}

/*
函数功能：激光器开关
参数说明：
返回值：
*/
void Widget::laserPumpSwitch()
{
//    if(ui->checkBox->isChecked())
//    {
//        LaserEnergySet();
//        serialPort1.write(stringToHex("8301"));
//        //ui->checkBox_2->setEnabled(0);
//        //ui->comboBox->setEnabled(0);
//        //ui->comboBox_2->setEnabled(0);
//    }
//    else
//    {
//        serialPort1.write(stringToHex("8300"));
//        //ui->checkBox_2->setEnabled(1);
//        //ui->comboBox->setEnabled(1);
//        //if(ui->checkBox_2->isChecked()) ui->comboBox_2->setEnabled(1);
//        //else ui->comboBox_2->setEnabled(0);
//    }
}

/*
函数功能：激光器触发模式设置
参数说明：
返回值：
*/
void Widget::laserTriggerMode()
{
//    if(ui->checkBox_2->isChecked())
//    {
//        LaserFrequencySet();
//        serialPort1.write(stringToHex("9E00"));
//        waitKey(50);
//        ui->comboBox_2->setEnabled(1);
//    }
//    else
//    {
//        serialPort1.write(stringToHex("9E01"));
//        waitKey(50);
//        ui->comboBox_2->setEnabled(0);
//    }
}

/*
函数功能：激光器能量设置
参数说明：
返回值：
*/
void Widget::laserEnergySet()
{
//    switch (ui->horizontalSlider->value())
//    {
//        case 100: serialPort1.write(stringToHex("80FF")); waitKey(50); break;
//        case 95 : serialPort1.write(stringToHex("80F2")); waitKey(50); break;
//        case 90 : serialPort1.write(stringToHex("80F6")); waitKey(50); break;
//        case 85 : serialPort1.write(stringToHex("80D9")); waitKey(50); break;
//        case 80 : serialPort1.write(stringToHex("80CC")); waitKey(50); break;
//        case 75 : serialPort1.write(stringToHex("80BF")); waitKey(50); break;
//        case 70 : serialPort1.write(stringToHex("80B3")); waitKey(50); break;
//        case 65 : serialPort1.write(stringToHex("80A6")); waitKey(50); break;
//        case 60 : serialPort1.write(stringToHex("8099")); waitKey(50); break;
//        case 55 : serialPort1.write(stringToHex("808C")); waitKey(50); break;
//        case 50 : serialPort1.write(stringToHex("8080")); waitKey(50); break;
//        case 45 : serialPort1.write(stringToHex("8073")); waitKey(50); break;
//        case 40 : serialPort1.write(stringToHex("8066")); waitKey(50); break;
//        case 30 : serialPort1.write(stringToHex("804D")); waitKey(50); break;
//        case 20 : serialPort1.write(stringToHex("8033")); waitKey(50); break;
//        case 10 : serialPort1.write(stringToHex("801A")); waitKey(50); break;
//    }
}

/*
函数功能：激光器频率设置
参数说明：
返回值：
*/
void Widget::laserFrequencySet()
{
//    switch (ui->comboBox_2->currentIndex())
//    {
//        case 0 :  serialPort1.write(stringToHex("97E8")); waitKey(50); serialPort1.write(stringToHex("9803")); waitKey(50); break;
//        case 1 :  serialPort1.write(stringToHex("9784")); waitKey(50); serialPort1.write(stringToHex("9803")); waitKey(50); break;
//        case 2 :  serialPort1.write(stringToHex("9720")); waitKey(50); serialPort1.write(stringToHex("9803")); waitKey(50); break;
//        case 3 :  serialPort1.write(stringToHex("97BC")); waitKey(50); serialPort1.write(stringToHex("9802")); waitKey(50); break;
//        case 4 :  serialPort1.write(stringToHex("9758")); waitKey(50); serialPort1.write(stringToHex("9802")); waitKey(50); break;
//        case 5 :  serialPort1.write(stringToHex("97F4")); waitKey(50); serialPort1.write(stringToHex("9801")); waitKey(50); break;
//        case 6 :  serialPort1.write(stringToHex("9790")); waitKey(50); serialPort1.write(stringToHex("9801")); waitKey(50); break;
//        case 7 :  serialPort1.write(stringToHex("972C")); waitKey(50); serialPort1.write(stringToHex("9801")); waitKey(50); break;
//        case 8 :  serialPort1.write(stringToHex("97FA")); waitKey(50); serialPort1.write(stringToHex("9800")); waitKey(50); break;
//        case 9 :  serialPort1.write(stringToHex("97C8")); waitKey(50); serialPort1.write(stringToHex("9800")); waitKey(50); break;
//        case 10 : serialPort1.write(stringToHex("9796")); waitKey(50); serialPort1.write(stringToHex("9800")); waitKey(50); break;
//        case 11 : serialPort1.write(stringToHex("9764")); waitKey(50); serialPort1.write(stringToHex("9800")); waitKey(50); break;
//        case 12 : serialPort1.write(stringToHex("9732")); waitKey(50); serialPort1.write(stringToHex("9800")); waitKey(50); break;
//        case 13 : serialPort1.write(stringToHex("9719")); waitKey(50); serialPort1.write(stringToHex("9800")); waitKey(50); break;
//        case 14 : serialPort1.write(stringToHex("9714")); waitKey(50); serialPort1.write(stringToHex("9800")); waitKey(50); break;
//        case 15 : serialPort1.write(stringToHex("970A")); waitKey(50); serialPort1.write(stringToHex("9800")); waitKey(50); break;
//    }
}

/*
函数功能：打开FPGA控制串口
参数说明：
返回值：
*/
void Widget::serialPortOpen2()
{
    serialPort2.setPortName(FPGA);
    //serialPort1.setPortName(ui->comboBox->currentText());
    if(serialPort2.open(QIODevice::ReadWrite))
    {
        serialPort2.setBaudRate(QSerialPort::Baud115200);
        serialPort2.setDataBits(QSerialPort::Data8);
        serialPort2.setParity(QSerialPort::NoParity);
        serialPort2.setStopBits(QSerialPort::OneStop);
        serialPort2.setFlowControl(QSerialPort::NoFlowControl);
    }
    //else ui->textEdit_2->append(" Warnning FPGA连接失败 ");
}

/*
函数功能：向FPGA发送串口信号，开始给采集卡触发
参数说明：
返回值：
*/
void Widget::startAlineDebug()
{
    serialPort2.write(stringToHex("F0"));
}

/*
函数功能：加载电机DLL
参数说明：
返回值：
*/
void Widget::serialPortOpen3()
{
    HINSTANCE hdll;
    hdll = LoadLibraryA("SerialCom.dll");
    FUN1 DLL_OpenCom;
    FUN2 ControlStatusSet;
    DLL_OpenCom = (FUN1)GetProcAddress(hdll,"DLL_OpenCom");
    ControlStatusSet = (FUN2)GetProcAddress(hdll,"ControlStatusSet");

    ControlStatusSet(1, 1);
    ControlStatusSet(2, 1);
}

/*
函数功能：位移平台回零
参数说明：
返回值：
*/
void Widget::backToZero()
{
    HINSTANCE hdll;
    hdll = LoadLibraryA("SerialCom.dll");
    FUN3 PositionAbsoluteMove;
    PositionAbsoluteMove = (FUN3)GetProcAddress(hdll,"PositionAbsoluteMove");
    FUN8 StopScan;
    StopScan = (FUN8)GetProcAddress(hdll,"StopScan");
    PositionAbsoluteMove(2, 1);
    //delay
    sleep(10000);
    StopScan();
}

/*
函数功能：开始全扫，FPGA开始发送触发，设置扫描速度、范围
参数说明：
返回值：
*/
void Widget::startFullScan()
{
    HINSTANCE hdll;
    hdll = LoadLibraryA("SerialCom.dll");
    FUN3 PositionAbsoluteMove;
    FUN4 PaceSet;
    FUN5 VelocitySet;
    FUN6 ScanRangeSet;
    FUN7 BeginScan;
    PositionAbsoluteMove = (FUN3)GetProcAddress(hdll,"PositionAbsoluteMove");
    PaceSet = (FUN4)GetProcAddress(hdll,"PaceSet");
    VelocitySet = (FUN5)GetProcAddress(hdll,"VelocitySet");
    ScanRangeSet = (FUN6)GetProcAddress(hdll,"ScanRangeSet");
    BeginScan = (FUN7)GetProcAddress(hdll,"BeginScan");

    switch (ui->comboBox_3->currentIndex())
    {
        case 0 :switch (ui->comboBox_4->currentIndex())//范围 3 mm
                {//速度 5 10 20 1
                    case 0 :{serialPort2.write(stringToHex("F1"));
                            PositionAbsoluteMove(2, -1);
                            PositionAbsoluteMove(1, -2);
                            waitKey(500);
                            ScanRangeSet(40000, -40000, 30000, -30000);
                            PaceSet(ui->spinBox_4->value()*20);
                            VelocitySet(1, 5);
                            VelocitySet(2, 2);
                            BeginScan();
                            break;}
                    case 1 :{serialPort2.write(stringToHex("F2"));
                            PositionAbsoluteMove(2, -1);
                            PositionAbsoluteMove(1, -2);
                            waitKey(500);
                            ScanRangeSet(40000, -40000, 30000, -30000);
                            PaceSet(ui->spinBox_4->value()*20);
                            VelocitySet(1, 10);
                            VelocitySet(2, 2);
                            BeginScan();
                            break;}
                    case 2 :{serialPort2.write(stringToHex("F4"));
                            PositionAbsoluteMove(2, -1);
                            PositionAbsoluteMove(1, -2);
                            waitKey(500);
                            ScanRangeSet(40000, -40000, 30000, -30000);
                            PaceSet(ui->spinBox_4->value()*20);
                            VelocitySet(1, 20);
                            VelocitySet(2, 2);
                            BeginScan();
                            break;}
                } break;
        case 1 :switch (ui->comboBox_4->currentIndex())//范围 5 mm
                {//速度 5 10 20 1
                    case 0 :{serialPort2.write(stringToHex("F5"));
                            PositionAbsoluteMove(2, -2);
                            PositionAbsoluteMove(1, -3);
                            waitKey(500);
                            ScanRangeSet(60000, -60000, 50000, -50000);
                            PaceSet(ui->spinBox_4->value()*20);
                            VelocitySet(1, 5);
                            VelocitySet(2, 2);
                            BeginScan();
                            break;}
                    case 1 :{serialPort2.write(stringToHex("F6"));
                            PositionAbsoluteMove(2, -2);
                            PositionAbsoluteMove(1, -3);
                            waitKey(500);
                            ScanRangeSet(60000, -60000, 50000, -50000);
                            PaceSet(ui->spinBox_4->value()*20);
                            VelocitySet(1, 10);
                            VelocitySet(2, 2);
                            BeginScan();
                            break;}
                    case 2 :{serialPort2.write(stringToHex("F8"));
                            PositionAbsoluteMove(2, -2);
                            PositionAbsoluteMove(1, -3);
                            waitKey(500);
                            ScanRangeSet(60000, -60000, 50000, -50000);
                            PaceSet(ui->spinBox_4->value()*20);
                            VelocitySet(1, 20);
                            VelocitySet(2, 2);
                            BeginScan();
                            break;}
                } break;
        case 2 :switch (ui->comboBox_4->currentIndex())//范围 8 mm
                {//速度 5 10 20 1
                    case 0 :{serialPort2.write(stringToHex("F9"));
                            PositionAbsoluteMove(2, -4);
                            PositionAbsoluteMove(1, -4);
                            waitKey(500);
                            ScanRangeSet(90000, -90000, 80000, -80000);
                            PaceSet(ui->spinBox_4->value()*20);
                            VelocitySet(1, 5);
                            VelocitySet(2, 2);
                            BeginScan();
                            break;}
                    case 1 :{serialPort2.write(stringToHex("FA"));
                            PositionAbsoluteMove(2, -4);
                            PositionAbsoluteMove(1, -4);
                            waitKey(500);
                            ScanRangeSet(90000, -90000, 80000, -80000);
                            PaceSet(ui->spinBox_4->value()*20);
                            VelocitySet(1, 10);
                            VelocitySet(2, 2);
                            BeginScan();
                            break;}
                    case 2 :{serialPort2.write(stringToHex("FC"));
                            PositionAbsoluteMove(2, -4);
                            PositionAbsoluteMove(1, -4);
                            waitKey(500);
                            ScanRangeSet(90000, -90000, 80000, -80000);
                            PaceSet(ui->spinBox_4->value()*20);
                            VelocitySet(1, 20);
                            VelocitySet(2, 2);
                            BeginScan();
                            break;}
                } break;
        case 3 :switch (ui->comboBox_4->currentIndex())//范围 10 mm
                {//速度 5 10 20 1
                    case 0 :{serialPort2.write(stringToHex("FD"));
                            PositionAbsoluteMove(2, -5);
                            PositionAbsoluteMove(1, -5);
                            waitKey(500);
                            ScanRangeSet(110000, -110000, 100000, -100000);
                            PaceSet(ui->spinBox_4->value()*20);
                            VelocitySet(1, 5);
                            VelocitySet(2, 2);
                            BeginScan();
                            break;}
                    case 1 :{serialPort2.write(stringToHex("FE"));
                            PositionAbsoluteMove(2, -5);
                            PositionAbsoluteMove(1, -5);
                            waitKey(500);
                            ScanRangeSet(110000, -110000, 100000, -100000);
                            PaceSet(ui->spinBox_4->value()*20);
                            VelocitySet(1, 10);
                            VelocitySet(2, 2);
                            BeginScan();
                            break;}
                    case 2 :{serialPort2.write(stringToHex("0F"));
                            PositionAbsoluteMove(2, -5);
                            PositionAbsoluteMove(1, -5);
                            waitKey(500);
                            ScanRangeSet(110000, -110000, 100000, -100000);
                            PaceSet(ui->spinBox_4->value()*20);
                            VelocitySet(1, 20);
                            VelocitySet(2, 2);
                            BeginScan();
                            break;}
                } break;
        case 4 :switch (ui->comboBox_4->currentIndex())//范围 15 mm
                {//速度 5 10 20 1
                    case 0 :{serialPort2.write(stringToHex("1F"));
                            PositionAbsoluteMove(2, -7);
                            PositionAbsoluteMove(1, -8);
                            waitKey(500);
                            ScanRangeSet(160000, -160000, 150000, -150000);
                            PaceSet(ui->spinBox_4->value()*20);
                            VelocitySet(1, 5);
                            VelocitySet(2, 2);
                            BeginScan();
                            break;}
                    case 1 :{serialPort2.write(stringToHex("2F"));
                            PositionAbsoluteMove(2, -7);
                            PositionAbsoluteMove(1, -8);
                            waitKey(500);
                            ScanRangeSet(160000, -160000, 150000, -150000);
                            PaceSet(ui->spinBox_4->value()*20);
                            VelocitySet(1, 10);
                            VelocitySet(2, 2);
                            BeginScan();
                            break;}
                    case 2 :{serialPort2.write(stringToHex("4F"));
                            PositionAbsoluteMove(2, -7);
                            PositionAbsoluteMove(1, -8);
                            waitKey(500);
                            ScanRangeSet(160000, -160000, 150000, -150000);
                            PaceSet(ui->spinBox_4->value()*20);
                            VelocitySet(1, 20);
                            VelocitySet(2, 2);
                            BeginScan();
                            break;}
                } break;
        case 5 :switch (ui->comboBox_4->currentIndex())//范围 20 mm
                {//速度 5 10 20 1
                    case 0 :{serialPort2.write(stringToHex("5F"));
                            PositionAbsoluteMove(2, -10);
                            PositionAbsoluteMove(1, -10);
                            waitKey(500);
                            ScanRangeSet(210000, -210000, 200000, -200000);
                            PaceSet(ui->spinBox_4->value()*20);
                            VelocitySet(1, 5);
                            VelocitySet(2, 2);
                            BeginScan();
                            break;}
                    case 1 :{serialPort2.write(stringToHex("6F"));
                            PositionAbsoluteMove(2, -10);
                            PositionAbsoluteMove(1, -10);
                            waitKey(500);
                            ScanRangeSet(210000, -210000, 200000, -200000);
                            PaceSet(ui->spinBox_4->value()*20);
                            VelocitySet(1, 10);
                            VelocitySet(2, 2);
                            BeginScan();
                            break;}
                    case 2 :{serialPort2.write(stringToHex("8F"));
                            PositionAbsoluteMove(2, -10);
                            PositionAbsoluteMove(1, -10);
                            waitKey(500);
                            ScanRangeSet(210000, -210000, 200000, -200000);
                            PaceSet(ui->spinBox_4->value()*20);
                            VelocitySet(1, 20);
                            VelocitySet(2, 2);
                            BeginScan();
                            break;}
                } break;
    }
}

/*
函数功能：电机停止运动
参数说明：
返回值：
*/
void Widget::endScan()
{
//    HINSTANCE hdll;
//    hdll = LoadLibraryA("SerialCom.dll");
//    FUN8 StopScan;
//    StopScan = (FUN8)GetProcAddress(hdll,"StopScan");
//    StopScan();
}

/*
函数功能：由扫描范围和速度确定每个Bscan包含的Aline数
参数说明：
返回值：每个Bscan包含的Aline数
*/
short Widget::fixAlineCount()
{
    short length = 0;
    switch (ui->comboBox_3->currentIndex())//范围、速度
    {
        case 0:length = 3;  break;
        case 1:length = 5;  break;
        case 2:length = 8;  break;
        case 3:length = 10; break;
        case 4:length = 15; break;
        case 5:length = 20; break;
    }
    return length*trigFre/ui->comboBox_4->currentText().toInt();
}

/*
函数功能：由扫描范围和步距确定Bscan数
参数说明：
返回值：Bscan数
*/
short Widget::fixBscanCount()
{
    short length = 0;
    switch (ui->comboBox_3->currentIndex())//范围、步长
    {
        case 0:length = 3;  break;
        case 1:length = 5;  break;
        case 2:length = 8;  break;
        case 3:length = 10; break;
        case 4:length = 15; break;
        case 5:length = 20; break;
    }
    return length*1000/ui->spinBox_4->value();
}

/*
函数功能：Cline中值处理
参数说明：inArray为原始Cline，arrayLength为原始Cline长度
返回值：Cline中值结果
*/
short Widget::midValue(short *inArray, int arrayLength)
{
    sort(inArray, inArray+arrayLength);
    if(arrayLength % 2 == 0)
        return (inArray[arrayLength/2-1] + inArray[arrayLength/2])/2;
    else
        return inArray[(arrayLength-1)/2];
}

/*
函数功能：字符串转十六进制，用于发送串口
参数说明：
返回值：
*/
QByteArray Widget::stringToHex(QString str)
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
        hex_data = convertHexChar(h_str);
        low_hex_data = convertHexChar(l_str);
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

/*
函数功能：字符串转十六进制，用于发送串口
参数说明：
返回值：
*/
char Widget::convertHexChar(char ch)
{
    if((ch >= '0') && (ch <= '9'))
            return ch-0x30;
        else if((ch >= 'A') && (ch <= 'F'))
            return ch-'A'+10;
        else if((ch >= 'a') && (ch <= 'f'))
            return ch-'a'+10;
        else return (-1);
}

/*
函数功能：延时
参数说明：msec为毫秒
返回值：
*/
void Widget::sleep(int msec)
{
    QTime dieTime = QTime::currentTime().addMSecs(msec);
    while( QTime::currentTime() < dieTime) QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
}

/*
函数功能：写入log.txt，记录触发频率、采样率、平均次数、截取长度、Aline总数、Bscan总数、X轴扫描速度、Y轴步距
参数说明：
返回值：
*/
void Widget::writeLog()
{
    QString fileName = ui->textEdit->toPlainText() + "/log.txt";
    ofstream outFile;
    outFile.open(fileName.toStdString().c_str(), ios::out);

    outFile << trigFre << "\n";
    outFile << sampleRate << "\n";
    outFile << meanTimes << "\n";
    outFile << cutCount << "\n";
    outFile << alineCount/meanTimes << "\n";
    outFile << bscanCount/2 << "\n";
    outFile << ui->comboBox_4->currentText().toInt() << "\n";
    outFile << ui->spinBox_4->value() << "\n";

    outFile.close();
}

/****************************************************************重建处理相关函数****************************************************************/
/*
函数功能：选择数据存储路径
参数说明：
返回值：
*/
void Widget::savePathSelect2()
{
    ui->textEdit_2->clear();
    QString SavePath = QFileDialog::getExistingDirectory(this, tr("Open Directory"), "/home", QFileDialog::ShowDirsOnly);
    ui->textEdit_2->append(SavePath);
    ui->pushButton_13->setEnabled(true);
}

/*
函数功能：读取log.txt，参数写入到vector<double> paraLog中
参数说明：
返回值：
*/
void Widget::readLog(vector<double> &vec)
{
    vec.clear();
    QString fileName = ui->textEdit_2->toPlainText() + "/log.txt";
    ifstream inFile;
    inFile.open(fileName.toStdString().c_str(), ios::in);

    double tmp = 0;
    while (inFile >> tmp) vec.push_back(tmp);
    //trigFre    1sampleRate  2meanTimes   3cutCount    4alineCount  5bscanCount  6Xspeed  7Ystep
    inFile.close();
}

/*
函数功能：读取dat文件，将所有数据copy到vector<vector<vector<short>>> xyzData中
参数说明：
返回值：
*/
static Mat xzMap(1, 1, CV_8UC1, Scalar::all(0));
static Mat xyMap(1, 1, CV_8UC1, Scalar::all(0));
static Mat xyMap2(1, 1, CV_8UC1, Scalar::all(0));
static Mat xyDS1(1, 1, CV_8UC1, Scalar::all(0));
static Mat xyDS2(1, 1, CV_8UC1, Scalar::all(0));
static Mat xyDS3(1, 1, CV_8UC1, Scalar::all(0));
static Mat xyDS33(1, 1, CV_16UC1, Scalar::all(0));
void Widget::readDat()
{
    ui->verticalSlider_DS_1->setEnabled(false);
    ui->verticalSlider_DS_2->setEnabled(false);
    ui->verticalSlider_Con->setEnabled(false);
    ui->verticalSlider_Lum->setEnabled(false);
    ui->pushButton_9->setEnabled(false);
    ui->pushButton_10->setEnabled(false);
    ui->pushButton_11->setEnabled(false);
    ui->pushButton_13->setEnabled(false);
    ui->pushButton_14->setEnabled(false);
    ui->pushButton_save->setEnabled(false);
    ui->spinBox_Threshold->setEnabled(false);
    ui->spinBox_L1->setEnabled(false);
    ui->spinBox_L2->setEnabled(false);
    ui->spinBox_L3->setEnabled(false);
    ui->comboBox_colormap->setEnabled(false);

    readLog(paraLog);
    int alineCount2 = paraLog[4];
    int alineDepth2 = paraLog[3];
    size_t pointCount = static_cast<size_t>(alineCount2 * alineDepth2);
    int bscanCount2 = paraLog[5];
    short *data2D = new short[pointCount];

    vector<string> fileNames;
    vector<string> fileNames2;
    glob(ui->textEdit_2->toPlainText().toStdString(), fileNames);
    for(unsigned long long i = 0; i < fileNames.size(); ++i)
    {
        if(fileNames[i].find(".dat") != string::npos) fileNames2.push_back(fileNames[i]);//只保留后缀.dat的路径
    }
    sort(fileNames2.begin(), fileNames2.end(), [](string a, string b)
    {
        unsigned long long aBegin = a.find('_'), aEnd = a.find(".dat"), bBegin = b.find('_'), bEnd = b.find(".dat");
        int aNum = stoi(a.substr(aBegin+1, aEnd-aBegin));
        int bNum = stoi(b.substr(bBegin+1, bEnd-bBegin));
        return aNum < bNum;
    });

    ui->verticalSlider_DS_1->setMaximum(alineDepth2-1);
    ui->verticalSlider_DS_1->setValue(alineDepth2-1);
    ui->verticalSlider_DS_2->setMaximum(alineDepth2-1);
    ui->verticalSlider_DS_2->setValue(0);

    cv::resize(xzMap, xzMap, Size(alineDepth2, alineCount2));
    cv::resize(xyMap, xyMap, Size(alineCount2, bscanCount2));
    cv::resize(xyMap2, xyMap2, Size(alineCount2, bscanCount2));
    cv::resize(xyDS1, xyDS1, Size(alineCount2, bscanCount2));
    cv::resize(xyDS2, xyDS2, Size(alineCount2, bscanCount2));
    cv::resize(xyDS3, xyDS3, Size(alineCount2, bscanCount2));
    cv::resize(xyDS33, xyDS33, Size(alineCount2, bscanCount2));

    Mat xzMapResize, xyMapResize, xyMap2Resize, xyDS1Resize, xyDS2Resize, xyDS3Resize, xyDS33Resize;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    xyzData.resize(bscanCount2);
    for(unsigned int i = 0; i < fileNames2.size(); ++i)
    {
        FILE* InFile;
        fopen_s(&InFile, fileNames2[i].c_str(), "rb");
        fread(data2D, sizeof(short), pointCount, InFile);
        fclose(InFile);

        xyzData[i].resize(alineCount2);
        for(int r = 0; r < alineCount2; ++r)
        {
            xyzData[i][r].resize(alineDepth2);
            for(int c = 0; c < alineDepth2; ++c) xyzData[i][r][c] = data2D[alineDepth2*r+c];//第i个bscan的第r个aline的第c个点
        }

        uchar *xyMapRow = xyMap.ptr<uchar>(static_cast<int>(i));
        uchar *xyMapRow2 = xyMap2.ptr<uchar>(static_cast<int>(i));
        uchar *xyDSRow1 = xyDS1.ptr<uchar>(static_cast<int>(i));
        uchar *xyDSRow2 = xyDS2.ptr<uchar>(static_cast<int>(i));
        uchar *xyDSRow3 = xyDS3.ptr<uchar>(static_cast<int>(i));
        ushort *xyDSRow33 = xyDS33.ptr<ushort>(static_cast<int>(i));
        for(int x = 0; x < alineCount2; ++x)
        {
            short xyMax = SHRT_MIN, xyMax2 = SHRT_MIN;
            int startDepth = ui->verticalSlider_DS_1->maximum() - ui->verticalSlider_DS_1->value(), endDepth = ui->verticalSlider_DS_2->maximum() - ui->verticalSlider_DS_2->value();
            for(int z = startDepth; z < endDepth; ++z) if(xyMax < xyzData[i][x][z]) xyMax = xyzData[i][x][z];
            for(int z = startDepth; z < alineDepth2; ++z) if(xyMax2 < xyzData[i][x][z]) xyMax2 = xyzData[i][x][z];
            xyMapRow[x] = saturate_cast<uchar>(xyMax/64);
            xyMapRow2[x] = saturate_cast<uchar>(xyMax2/64);

            short maxLayer1 = SHRT_MIN, maxLayer2 = SHRT_MIN, maxLayer3 = SHRT_MIN, maxLayer33 = SHRT_MIN;
            for(int z = 1; z < alineDepth2-1; ++z)
            {
                if((xyzData[i][x][z-1] < xyzData[i][x][z]) && (xyzData[i][x][z] < xyzData[i][x][z+1]) && (xyzData[i][x][z] > ui->spinBox_Threshold->value() * 64))
                {
                    for(unsigned int n = z + ui->spinBox_L1->value(); n < z + ui->spinBox_L2->value(); ++n)
                    {
                        if(maxLayer1 < xyzData[i][x][n]) maxLayer1 = xyzData[i][x][n];
                    }
                    for(unsigned int n = z + ui->spinBox_L2->value(); n < z + ui->spinBox_L3->value() + ui->spinBox_L2->value(); ++n)
                    {
                        if(maxLayer2 < xyzData[i][x][n]) maxLayer2 = xyzData[i][x][n];
                    }
                    for(unsigned int n = z + ui->spinBox_L3->value() + ui->spinBox_L2->value(); n < alineDepth2; ++n)
                    {
                        if(maxLayer3 < xyzData[i][x][n]) maxLayer3 = xyzData[i][x][n];
                        if(maxLayer33 < xyzData[i][x][n]) maxLayer33 = xyzData[i][x][n];
                    }
                    break;
                }
            }
            xyDSRow1[x] = saturate_cast<uchar>(maxLayer1/64);
            xyDSRow2[x] = saturate_cast<uchar>(maxLayer2/64);
            xyDSRow3[x] = saturate_cast<uchar>(maxLayer3/64);
            xyDSRow33[x] = saturate_cast<ushort>(maxLayer33);
        }

        Mat xyDS338 = Mat::zeros(xyDS33.size(), CV_8UC1);
        double min16, max16;
        minMaxIdx(xyDS33, &min16, &max16);
        double delt = 255.0 / (max16 - min16);
        for(int i = 0; i < xyDS33.rows; ++i)
        {
            for(int j=0; j < xyDS33.cols; ++j)
            {
                ushort s = xyDS33.at<ushort>(i, j);
                uchar d = saturate_cast<uchar>((s - min16) * delt);
                xyDS338.at<uchar>(i, j) = d;
            }
        }

        for(int x = 0; x < alineCount2; ++x)
        {
            uchar *xzMapRow = xzMap.ptr<uchar>(static_cast<int>(x));
            for(int z = 0; z < alineDepth2; ++z)
            {
                short xzMax = SHRT_MIN;
                for(unsigned int ii = 0; ii < i; ++ii) if(xzMax < xyzData[ii][x][z]) xzMax = xyzData[ii][x][z];
                xzMapRow[z] = saturate_cast<uchar>(xzMax/64);
            }
        }
        rotate(xzMap, xzMapResize, ROTATE_90_CLOCKWISE);
        cv::resize(xzMapResize, xzMapResize, Size(500,500), INTER_CUBIC);
        cv::resize(xyMap, xyMapResize, Size(500,500), INTER_CUBIC);
        cv::resize(xyMap2, xyMap2Resize, Size(500,500), INTER_CUBIC);
        cv::resize(xyDS1, xyDS1Resize, Size(500,500), INTER_CUBIC);
        cv::resize(xyDS2, xyDS2Resize, Size(500,500), INTER_CUBIC);
        cv::resize(xyDS3, xyDS3Resize, Size(500,500), INTER_CUBIC);
        cv::resize(xyDS338, xyDS33Resize, Size(500,500), INTER_CUBIC);
        applyColorMap(xzMapResize, xzMapResize, ui->comboBox_colormap->currentIndex());
        applyColorMap(xyMapResize, xyMapResize, ui->comboBox_colormap->currentIndex());
        applyColorMap(xyMap2Resize, xyMap2Resize, ui->comboBox_colormap->currentIndex());
        applyColorMap(xyDS1Resize, xyDS1Resize, ui->comboBox_colormap->currentIndex());
        applyColorMap(xyDS2Resize, xyDS2Resize, ui->comboBox_colormap->currentIndex());
        applyColorMap(xyDS3Resize, xyDS3Resize, ui->comboBox_colormap->currentIndex());
        applyColorMap(xyDS33Resize, xyDS33Resize, ui->comboBox_colormap->currentIndex());
        cvtColor(xzMapResize, xzMapResize, COLOR_BGR2RGB);
        cvtColor(xyMapResize, xyMapResize, COLOR_BGR2RGB);
        cvtColor(xyMap2Resize, xyMap2Resize, COLOR_BGR2RGB);
        cvtColor(xyDS1Resize, xyDS1Resize, COLOR_BGR2RGB);
        cvtColor(xyDS2Resize, xyDS2Resize, COLOR_BGR2RGB);
        cvtColor(xyDS3Resize, xyDS3Resize, COLOR_BGR2RGB);
        cvtColor(xyDS33Resize, xyDS33Resize, COLOR_BGR2RGB);
        QImage xyMapDisplay = QImage(xyMapResize.data, xyMapResize.cols, xyMapResize.rows, QImage::Format_RGB888);
        QImage xyMapDisplay2 = QImage(xyMap2Resize.data, xyMap2Resize.cols, xyMap2Resize.rows, QImage::Format_RGB888);
        QImage xzMapDisplay = QImage(xzMapResize.data, xzMapResize.cols, xzMapResize.rows, QImage::Format_RGB888);
        QImage xyDS1Display = QImage(xyDS1Resize.data, xyDS1Resize.cols, xyDS1Resize.rows, QImage::Format_RGB888);
        QImage xyDS2Display = QImage(xyDS2Resize.data, xyDS2Resize.cols, xyDS2Resize.rows, QImage::Format_RGB888);
        QImage xyDS3Display = QImage(xyDS33Resize.data, xyDS3Resize.cols, xyDS3Resize.rows, QImage::Format_RGB888);

        if(i%2)
        {
            ui->label_DS_1->setPixmap(QPixmap::fromImage(xyMapDisplay));
            ui->label_DS_2->setPixmap(QPixmap::fromImage(xyMapDisplay2));
            ui->label_XZMAP->setPixmap(QPixmap::fromImage(xzMapDisplay));
            ui->label_L1->setPixmap(QPixmap::fromImage(xyDS1Display));
            ui->label_L2->setPixmap(QPixmap::fromImage(xyDS2Display));
            ui->label_L3->setPixmap(QPixmap::fromImage(xyDS3Display));
        }
        xyMap.copyTo(org); xyMap.copyTo(img);//MVD
        applyColorMap(org, org, ui->comboBox_colormap->currentIndex());
        applyColorMap(img, img, ui->comboBox_colormap->currentIndex());
    }
    cv::resize(org, org, Size(500, 500));
    cv::resize(img, img, Size(500, 500));
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    delete [] data2D;
    ui->verticalSlider_DS_1->setEnabled(true);
    ui->verticalSlider_DS_2->setEnabled(true);
    ui->verticalSlider_Con->setEnabled(true);
    ui->verticalSlider_Lum->setEnabled(true);
    ui->pushButton_9->setEnabled(true);
    ui->pushButton_10->setEnabled(true);
    ui->pushButton_11->setEnabled(true);
    ui->pushButton_13->setEnabled(true);
    ui->pushButton_14->setEnabled(true);
    ui->pushButton_save->setEnabled(true);
    ui->spinBox_Threshold->setEnabled(true);
    ui->spinBox_L1->setEnabled(true);
    ui->spinBox_L2->setEnabled(true);
    ui->spinBox_L3->setEnabled(true);
    ui->comboBox_colormap->setEnabled(true);
}

/*
函数功能：xy面最大值投影 深度切片
参数说明：
返回值：
*/
void Widget::depthChange()
{
    int rows = paraLog[5], cols = paraLog[4], depth = paraLog[3];
    Mat xyMapResize, xyMap2Resize, xzMapResize;

    Mat top(depth, 500, CV_8UC1, Scalar::all(0));
    for(int i = 0; i < 500; ++i) top.at<uchar>(ui->verticalSlider_DS_1->maximum() - ui->verticalSlider_DS_1->value(),i) = 255;
    Mat buttom(depth, 500, CV_8UC1, Scalar::all(0));
    for(int i = 0; i < 500; ++i) top.at<uchar>(ui->verticalSlider_DS_2->maximum() - ui->verticalSlider_DS_2->value(),i) = 255;

    for(int x = 0; x < cols; ++x)
    {
        uchar *xzMapRow = xzMap.ptr<uchar>(static_cast<int>(x));
        for(int z = 0; z < depth; ++z)
        {
            short xzMax = SHRT_MIN;
            for(unsigned int ii = 0; ii < rows; ++ii) if(xzMax < xyzData[ii][x][z]) xzMax = xyzData[ii][x][z];
            xzMapRow[z] = saturate_cast<uchar>(xzMax/64);
        }
    }
    rotate(xzMap, xzMapResize, ROTATE_90_CLOCKWISE);
    cv::resize(xzMapResize, xzMapResize, Size(500,500), INTER_CUBIC);
    xzMapResize = xzMapResize + top + buttom;

    for(int i = 0; i < rows; ++i)
    {
        uchar *xyMapRow = xyMap.ptr<uchar>(static_cast<int>(i));
        uchar *xyMapRow2 = xyMap2.ptr<uchar>(static_cast<int>(i));
        for(int x = 0; x < cols; ++x)
        {
            short xyMax = 0, xyMax2 = 0;
            int startDepth = ui->verticalSlider_DS_1->maximum() - ui->verticalSlider_DS_1->value(), endDepth = ui->verticalSlider_DS_2->maximum() - ui->verticalSlider_DS_2->value();
            for(int z = startDepth; z < endDepth; ++z) if(xyMax < xyzData[i][x][z]) xyMax = xyzData[i][x][z];
            for(int z = startDepth; z < depth; ++z) if(xyMax2 < xyzData[i][x][z]) xyMax2 = xyzData[i][x][z];
            xyMapRow[x] = saturate_cast<uchar>(xyMax/64);
            xyMapRow2[x] = saturate_cast<uchar>(xyMax2/64);
        }
    }
    cv::resize(xyMap, xyMapResize, Size(500,500), INTER_CUBIC);
    cv::resize(xyMap2, xyMap2Resize, Size(500,500), INTER_CUBIC);
    applyColorMap(xzMapResize, xzMapResize, ui->comboBox_colormap->currentIndex());
    applyColorMap(xyMapResize, xyMapResize, ui->comboBox_colormap->currentIndex());
    applyColorMap(xyMap2Resize, xyMap2Resize, ui->comboBox_colormap->currentIndex());
    cvtColor(xzMapResize, xzMapResize, COLOR_BGR2RGB);
    cvtColor(xyMapResize, xyMapResize, COLOR_BGR2RGB);
    cvtColor(xyMap2Resize, xyMap2Resize, COLOR_BGR2RGB);
    QImage xzMapDisplay = QImage(xzMapResize.data, xzMapResize.rows, xzMapResize.cols, QImage::Format_RGB888);
    QImage xyMapDisplay = QImage(xyMapResize.data, xyMapResize.rows, xyMapResize.cols, QImage::Format_RGB888);
    QImage xyMapDisplay2 = QImage(xyMap2Resize.data, xyMap2Resize.rows, xyMap2Resize.cols, QImage::Format_RGB888);
    ui->label_XZMAP->setPixmap(QPixmap::fromImage(xzMapDisplay));
    ui->label_DS_1->setPixmap(QPixmap::fromImage(xyMapDisplay));
    ui->label_DS_2->setPixmap(QPixmap::fromImage(xyMapDisplay2));

    xyMapResize.copyTo(org); xyMapResize.copyTo(img);
    applyColorMap(org, org, ui->comboBox_colormap->currentIndex());
    applyColorMap(img, img, ui->comboBox_colormap->currentIndex());
}

/*
函数功能：对比度亮度调整
参数说明：
返回值：
*/
void Widget::contrastChange()
{
    int rows = paraLog[5], cols = paraLog[4], depth = paraLog[3];
    Mat xyMap3 = xyMap.clone();
    Mat xyMap23 = xyMap2.clone();
    Mat xzMap3 = xzMap.clone();
    Mat xyDS13 = xyDS1.clone();
    Mat xyDS23 = xyDS2.clone();
    Mat xyDS33 = xyDS3.clone();
    Mat xzMapResize, xyMapResize, xyMap2Resize, xyDS1Resize, xyDS2Resize, xyDS3Resize;
    for(int i = 0; i < rows; ++i)
    {
        uchar *xyMapRow3 = xyMap3.ptr<uchar>(static_cast<int>(i)); uchar *xyMapRow = xyMap.ptr<uchar>(static_cast<int>(i));
        uchar *xyMapRow23 = xyMap23.ptr<uchar>(static_cast<int>(i)); uchar *xyMapRow2 = xyMap2.ptr<uchar>(static_cast<int>(i));
        uchar *xyDSRow13 = xyDS13.ptr<uchar>(static_cast<int>(i)); uchar *xyDSRow1 = xyDS1.ptr<uchar>(static_cast<int>(i));
        uchar *xyDSRow23 = xyDS23.ptr<uchar>(static_cast<int>(i)); uchar *xyDSRow2 = xyDS2.ptr<uchar>(static_cast<int>(i));
        uchar *xyDSRow33 = xyDS33.ptr<uchar>(static_cast<int>(i)); uchar *xyDSRow3 = xyDS3.ptr<uchar>(static_cast<int>(i));
        for(int x = 0; x < cols; ++x)
        {
            xyMapRow3[x] = saturate_cast<uchar>(ui->verticalSlider_Con->value() * saturate_cast<uchar>(xyMapRow[x]) + ui->verticalSlider_Lum->value());
            xyMapRow23[x] = saturate_cast<uchar>(ui->verticalSlider_Con->value() * saturate_cast<uchar>(xyMapRow2[x]) + ui->verticalSlider_Lum->value());
            xyDSRow13[x] = saturate_cast<uchar>(ui->verticalSlider_Con->value() * saturate_cast<uchar>(xyDSRow1[x]) + ui->verticalSlider_Lum->value());
            xyDSRow23[x] = saturate_cast<uchar>(ui->verticalSlider_Con->value() * saturate_cast<uchar>(xyDSRow2[x]) + ui->verticalSlider_Lum->value());
            xyDSRow33[x] = saturate_cast<uchar>(ui->verticalSlider_Con->value() * saturate_cast<uchar>(xyDSRow3[x]) + ui->verticalSlider_Lum->value());
        }
    }
    for(int x = 0; x < cols; ++x)
    {
        uchar *xzMapRow3 = xzMap3.ptr<uchar>(static_cast<int>(x)); uchar *xzMapRow = xzMap.ptr<uchar>(static_cast<int>(x));
        for(int z = 0; z < depth; ++z)
        {
            xzMapRow3[z] = saturate_cast<uchar>(ui->verticalSlider_Con->value() * saturate_cast<uchar>(xzMapRow[z]) + ui->verticalSlider_Lum->value());
        }
    }
    rotate(xzMap3, xzMapResize, ROTATE_90_CLOCKWISE);
    cv::resize(xzMapResize, xzMapResize, Size(500,500), INTER_CUBIC);
    cv::resize(xyMap3, xyMapResize, Size(500,500), INTER_CUBIC);
    cv::resize(xyMap23, xyMap2Resize, Size(500,500), INTER_CUBIC);
    cv::resize(xyDS13, xyDS1Resize, Size(500,500), INTER_CUBIC);
    cv::resize(xyDS23, xyDS2Resize, Size(500,500), INTER_CUBIC);
    cv::resize(xyDS33, xyDS3Resize, Size(500,500), INTER_CUBIC);
    applyColorMap(xzMapResize, xzMapResize, ui->comboBox_colormap->currentIndex());
    applyColorMap(xyMapResize, xyMapResize, ui->comboBox_colormap->currentIndex());
    applyColorMap(xyMap2Resize, xyMap2Resize, ui->comboBox_colormap->currentIndex());
    applyColorMap(xyDS1Resize, xyDS1Resize, ui->comboBox_colormap->currentIndex());
    applyColorMap(xyDS2Resize, xyDS2Resize, ui->comboBox_colormap->currentIndex());
    applyColorMap(xyDS3Resize, xyDS3Resize, ui->comboBox_colormap->currentIndex());
    cvtColor(xzMapResize, xzMapResize, COLOR_BGR2RGB);
    cvtColor(xyMapResize, xyMapResize, COLOR_BGR2RGB);
    cvtColor(xyMap2Resize, xyMap2Resize, COLOR_BGR2RGB);
    cvtColor(xyDS1Resize, xyDS1Resize, COLOR_BGR2RGB);
    cvtColor(xyDS2Resize, xyDS2Resize, COLOR_BGR2RGB);
    cvtColor(xyDS3Resize, xyDS3Resize, COLOR_BGR2RGB);
    QImage xyMapDisplay = QImage(xyMapResize.data, xyMapResize.cols, xyMapResize.rows, QImage::Format_RGB888);
    QImage xyMapDisplay2 = QImage(xyMap2Resize.data, xyMap2Resize.cols, xyMap2Resize.rows, QImage::Format_RGB888);
    QImage xzMapDisplay = QImage(xzMapResize.data, xzMapResize.cols, xzMapResize.rows, QImage::Format_RGB888);

    QImage xyDS1Display = QImage(xyDS1Resize.data, xyDS1Resize.cols, xyDS1Resize.rows, QImage::Format_RGB888);
    QImage xyDS2Display = QImage(xyDS2Resize.data, xyDS2Resize.cols, xyDS2Resize.rows, QImage::Format_RGB888);
    QImage xyDS3Display = QImage(xyDS3Resize.data, xyDS3Resize.cols, xyDS3Resize.rows, QImage::Format_RGB888);

    ui->label_DS_1->setPixmap(QPixmap::fromImage(xyMapDisplay));
    ui->label_DS_2->setPixmap(QPixmap::fromImage(xyMapDisplay2));
    ui->label_XZMAP->setPixmap(QPixmap::fromImage(xzMapDisplay));
    ui->label_L1->setPixmap(QPixmap::fromImage(xyDS1Display));
    ui->label_L2->setPixmap(QPixmap::fromImage(xyDS2Display));
    ui->label_L3->setPixmap(QPixmap::fromImage(xyDS3Display));
    xyMap3.copyTo(org); xyMap3.copyTo(img);//MVD
    applyColorMap(org, org, ui->comboBox_colormap->currentIndex());
    applyColorMap(img, img, ui->comboBox_colormap->currentIndex());
}

/*
函数功能：保存当前重建结果图像
参数说明：
返回值：
*/
void Widget::saveImage()
{
    Mat xzMapResize, xyMapResize, xyMap2Resize, xyDS1Resize, xyDS2Resize, xyDS3Resize;
    cv::resize(xzMap, xzMapResize, Size(500,500), INTER_CUBIC);
    rotate(xzMapResize, xzMapResize, ROTATE_90_CLOCKWISE);
    cv::resize(xyMap, xyMapResize, Size(500,500), INTER_CUBIC);
    cv::resize(xyMap2, xyMap2Resize, Size(500,500), INTER_CUBIC);
    cv::resize(xyDS1, xyDS1Resize, Size(500,500), INTER_CUBIC);
    cv::resize(xyDS2, xyDS2Resize, Size(500,500), INTER_CUBIC);
    cv::resize(xyDS3, xyDS3Resize, Size(500,500), INTER_CUBIC);
    applyColorMap(xzMapResize, xzMapResize, ui->comboBox_colormap->currentIndex());
    applyColorMap(xyMapResize, xyMapResize, ui->comboBox_colormap->currentIndex());
    applyColorMap(xyMap2Resize, xyMap2Resize, ui->comboBox_colormap->currentIndex());
    applyColorMap(xyDS1Resize, xyDS1Resize, ui->comboBox_colormap->currentIndex());
    applyColorMap(xyDS2Resize, xyDS2Resize, ui->comboBox_colormap->currentIndex());
    applyColorMap(xyDS3Resize, xyDS3Resize, ui->comboBox_colormap->currentIndex());
    QString xzMapPath = ui->textEdit_2->toPlainText() + "/xzMap.png";
    QString xyMapPath = ui->textEdit_2->toPlainText() + "/xyMap.png";
    QString xyMap2Path = ui->textEdit_2->toPlainText() + "/xyMap2.png";
    QString xyDS1Path = ui->textEdit_2->toPlainText() + "/xyDS1.png";
    QString xyDS2Path = ui->textEdit_2->toPlainText() + "/xyDS2.png";
    QString xyDS3Path = ui->textEdit_2->toPlainText() + "/xyDS3.png";
    imwrite(xzMapPath.toStdString(), xzMapResize);
    imwrite(xyMapPath.toStdString(), xyMapResize);
    imwrite(xyMap2Path.toStdString(), xyMap2Resize);
    imwrite(xyDS1Path.toStdString(), xyDS1Resize);
    imwrite(xyDS2Path.toStdString(), xyDS2Resize);
    imwrite(xyDS3Path.toStdString(), xyDS3Resize);
}

/*
函数功能：设置伪彩查找表
参数说明：
返回值：
*/
void Widget::lookUpTable()
{
    Mat table(1, 256, CV_8UC1, Scalar::all(0));
    uchar *tableRow = table.ptr<uchar>(0);
    for(int i = 0; i < 256; ++i) tableRow[i] = i;

    applyColorMap(table, table, ui->comboBox_colormap->currentIndex());
    cvtColor(table, table, COLOR_BGR2RGB);
    QImage xyMapDisplay = QImage(table.data, table.cols, table.rows, QImage::Format_RGB888);
    ui->label_colormap->setPixmap(QPixmap::fromImage(xyMapDisplay));
}

/*
函数功能：OpenCV鼠标回调函数1，选取矩形区域，计算血管参数
参数说明：
返回值：
*/
void Widget::onMouse1(int event, int x, int y, int flags, void *ustc)
{
    static Point prePoint(-1, -1);//初始坐标
    static Point curPoint(-1, -1);//实时坐标
    char text[16];
    if (event == EVENT_LBUTTONDOWN)//左键按下，读取初始坐标
    {
        org.copyTo(img);//将原始图片复制到img中
        sprintf_s(text, "(%d,%d)", x, y);
        prePoint = Point(x, y);
        putText(img, text, prePoint, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 1, 8);//在窗口上显示坐标
        imshow("Select Rectangular Area", img);
    }
    else if (event == EVENT_MOUSEMOVE && !(flags & EVENT_FLAG_LBUTTON))//左键没有按下的情况下鼠标移动的处理函数
    {
        img.copyTo(tmp);//将img复制到临时图像tmp上，用于显示实时坐标
        sprintf_s(text, "(%d,%d)", x, y);
        curPoint = Point(x, y);
        putText(tmp, text, curPoint, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 1, 8);//只是实时显示鼠标移动的坐标
        imshow("Select Rectangular Area", tmp);
    }
    else if (event == EVENT_MOUSEMOVE && (flags & EVENT_FLAG_LBUTTON))//左键按下时，鼠标移动，则在图像上划矩形
    {
        img.copyTo(tmp);
        sprintf_s(text, "(%d,%d)", x, y);
        curPoint = Point(x, y);
        putText(tmp, text, curPoint, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 1, 8);
        rectangle(tmp, prePoint, curPoint, Scalar(0, 255, 0), 1, 8, 0);//在临时图像上实时显示鼠标拖动时形成的矩形
        imshow("Select Rectangular Area", tmp);
    }
    else if (event == EVENT_LBUTTONUP)//左键松开，将在图像上划矩形
    {
        org.copyTo(img);
        sprintf_s(text, "(%d,%d)", x, y);
        curPoint = Point(x, y);
        putText(img, text, curPoint, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 1, 8);
        rectangle(img, prePoint, curPoint, Scalar(0, 255, 0), 1, 8, 0);//根据初始点和结束点，将矩形画到img上
        imshow("Select Rectangular Area", img);
        img.copyTo(tmp);
        //截取矩形包围的图像，并保存到dst中
        int width = abs(prePoint.x - curPoint.x);
        int height = abs(prePoint.y - curPoint.y);
        if (width == 0 || height == 0)
        {
            printf("width == 0 || height == 0");
            return;
        }
        dst = org(Rect(min(curPoint.x, prePoint.x), min(curPoint.y, prePoint.y), width, height));
        cvtColor(dst, dst, COLOR_BGR2GRAY);
        Mat dst2 = dst.clone();

        threshold(dst, dst, 10, 255, THRESH_BINARY);

        double cnt = 0;
        for (int i = 0; i < dst.rows; ++i)
        {
            uchar *srcRow = dst.ptr<uchar>(i);
            for (int j = 0; j < dst.cols; ++j)
            {
                if (srcRow[j] > 0) cnt++;
            }
        }
        double density = cnt / (dst.rows * dst.cols);
        double volumeDensity = density / 150;
        string text1 = "MVD:" + to_string(density);
        string text2 = "V-MVD:" + to_string(volumeDensity);
        putText(img, text1, Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 0), 1, 8);
        putText(img, text2, Point(10, 50), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 0), 1, 8);
        //if(density != 1) diamTort(diamRes, tortRes, dst2);

        imshow("Select Rectangular Area", img);
        waitKey(0);
    }
}

/*
函数功能：OpenCV鼠标回调函数2，选取不规则区域，计算血管参数
参数说明：
返回值：
*/
void Widget::onMouse2(int event, int x, int y, int flags, void *ustc)
{
    static Point prePoint(-1, -1);//初始坐标
    static Point curPoint(-1, -1);//实时坐标
    static vector<Point> contourPoint;
    char text[16];
    if (event == EVENT_LBUTTONDOWN)//左键按下，读取初始坐标
    {
        org.copyTo(img);//将原始图片复制到img中
        sprintf_s(text, "(%d,%d)", x, y);
        prePoint = Point(x, y);
        contourPoint.clear();
        contourPoint.push_back(prePoint);
        putText(img, text, prePoint, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 1, 8);//在窗口上显示坐标
        imshow("Select Area", img);
    }
    else if (event == EVENT_MOUSEMOVE && !(flags & EVENT_FLAG_LBUTTON))//左键没有按下的情况下鼠标移动的处理函数
    {
        img.copyTo(tmp);//将img复制到临时图像tmp上，用于显示实时坐标
        sprintf_s(text, "(%d,%d)", x, y);
        curPoint = Point(x, y);
        putText(tmp, text, curPoint, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 1, 8);//只是实时显示鼠标移动的坐标
        imshow("Select Area", tmp);
    }
    else if (event == EVENT_MOUSEMOVE && (flags & EVENT_FLAG_LBUTTON))//左键按下时，鼠标移动，则在图像上划多边形
    {
        img.copyTo(tmp);
        sprintf_s(text, "(%d,%d)", x, y);
        curPoint = Point(x, y);
        contourPoint.push_back(curPoint);
        putText(tmp, text, curPoint, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 1, 8);
        polylines(tmp, contourPoint, true, Scalar(0, 255, 0), 1, 8, 0);//在临时图像上实时显示鼠标拖动时形成的多边形
        imshow("Select Area", tmp);
    }
    else if (event == EVENT_LBUTTONUP)//左键松开，将在图像上划多边形
    {
        org.copyTo(img);
        sprintf_s(text, "(%d,%d)", x, y);
        curPoint = Point(x, y);
        polylines(img, contourPoint, true, Scalar(0, 255, 0), 1, 8, 0);//根据初始点和结束点，将多边形画到img上
        imshow("Select Area", img);
        //截取矩形包围的图像，并保存到dst中
        img.copyTo(tmp);
        tmp.setTo(Scalar::all(0));
        fillPoly(tmp, contourPoint, Scalar(255, 255, 255));
        bitwise_and(img, tmp, dst);
        cvtColor(dst, dst, COLOR_BGR2GRAY);
        Mat dst2 = dst.clone();

        threshold(dst, dst, 10, 255, THRESH_BINARY);
        //imshow("dst", dst);
        double cnt = 0, roi = 0;
        for (int i = 0; i < dst.rows; ++i)
        {
            int left = 0, right = 0;
            uchar *srcRow = dst.ptr<uchar>(i);
            for (int l = 0; l < dst.cols; ++l)
            {
                if (srcRow[l] > 0)
                {
                    left = l;
                    break;
                }
            }
            for (int r = dst.cols - 1; r >= 0; --r)
            {
                if (srcRow[r] > 0)
                {
                    right = r;
                    break;
                }
            }
            roi += (right - left);
            for (int mid = left; mid < right; ++mid)
            {
                if (srcRow[mid] > 0) cnt++;
            }
        }
        double density = cnt / roi;
        double volumeDensity = density / 150;
        string text1 = "MVD:" + to_string(density);
        string text2 = "V-MVD:" + to_string(volumeDensity);
        putText(img, text1, Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 0), 1, 8);
        putText(img, text2, Point(10, 50), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 0), 1, 8);
        //if(density != 1) diamTort(diamRes, tortRes, dst2);

        imshow("Select Area", img);
        waitKey(0);
    }
}

/*
函数功能：血管直径、弯曲度计算
参数说明：diamRes存十个区间直径结果和区间间隔，tortRes存十个区间弯曲度结果和区间间隔，src输入图像
返回值：
*/
void diamTort(vector<double>& diamRes, vector<double>& tortRes, Mat& src)
{
    diamRes.clear(); tortRes.clear();
    double Xdistance = paraLog[6]/paraLog[0]*paraLog[2]*paraLog[4]/500;
    double Ydistance = paraLog[7]/500*paraLog[5]/500;
    threshold(src, src, 10, 255, THRESH_BINARY);
    Mat element = getStructuringElement(MORPH_CROSS, Size(3, 3));
    morphologyEx(src, src, MORPH_OPEN, element);

    Mat edge;
    Mat src2 = src.clone();
    blur(src, edge, Size(3, 3));
    Canny(edge, edge, 3, 9, 3);
    normalize(edge, edge, 1, 0, NORM_MINMAX);

    Mat skel(src.size(), CV_8UC1, Scalar(0));
    Mat temp(src.size(), CV_8UC1, Scalar(0));

    bool done = false;
    do
    {
        morphologyEx(src, temp, MORPH_OPEN, element);
        bitwise_not(temp, temp);
        bitwise_and(src, temp, temp);
        bitwise_or(skel, temp, skel);
        erode(src, src, element);
        double max;
        minMaxLoc(src, 0, &max);
        done = (cvFloor(max) == 0);
    } while (!done);
    normalize(skel, skel, 1, 0, NORM_MINMAX);

    int skelRow = skel.rows, skelCol = skel.cols;
    const int range1 = 2, range2 = 50;
    vector<double> diam;
    vector<double> validDiam;
    for (int x = 0; x < skelRow; ++x)
    {
        for (int y = 0; y < skelCol; ++y)
        {
            if (skel.at<uchar>(x, y) != 0)
            {
                double k = 0, b = 0;
                int row = -1, col = -1;
                int iMax = (x + range1) < skelRow ? (x + range1) : skelRow;
                int jMax = (y + range1) < skelCol ? (y + range1) : skelCol;
                for (int i = x; i < iMax; ++i)
                {
                    for (int j = y - range1; j < jMax; ++j)
                    {
                        if (i >= 0 && j >= 0 && skel.at<uchar>(i, j) != 0)
                        {
                            if (x == i && y != j)
                            {
                                col = y;
                                break;
                            }
                            else if (x != i && y != j)
                            {
                                k = static_cast<double>((j - y) / (i - x));
                                b = static_cast<double>(y + x / k);
                                break;
                            }
                            else if (x != i && y == j)
                            {
                                row = x;
                                break;
                            }
                        }
                    }
                    if (cvCeil(k) != 0 || col != -1 || row != -1) break;
                }
                Point first(-1, -1), second(-1, -1);
                int mMax = (x + range2) < skelRow ? (x + range2) : skelRow;
                int mMin = (x - range2) < 0 ? 0 : (x - range2);
                int nMax = (y + range2) < skelCol ? (y + range2) : skelCol;
                int nMin = (y - range2) < 0 ? 0 : (y - range2);
                for (int m = mMin; m < mMax; ++m)
                {
                    for (int n = nMin; n < nMax; ++n)
                    {
                        if (edge.at<uchar>(m, n) != 0)
                        {
                            if (col == n)
                            {
                                first = Point(m, n);
                                break;
                            }
                            else if (cvRound(-m / k + b) == n)
                            {
                                first = Point(m, n);
                                break;
                            }
                            else if (row == m)
                            {
                                first = Point(m, n);
                                break;
                            }
                        }
                    }
                    if (first != Point(-1, -1)) break;
                }
                for (int m = mMax - 1; m >= mMin; --m)
                {
                    for (int n = nMax - 1; n >= nMin; --n)
                    {
                        if (edge.at<uchar>(m, n) != 0)
                        {
                            if (col == n)
                            {
                                second = Point(m, n);
                                break;
                            }
                            else if (cvRound(-m / k + b) == n)
                            {
                                second = Point(m, n);
                                break;
                            }
                            else if (row == m)
                            {
                                second = Point(m, n);
                                break;
                            }
                        }
                    }
                    if (second != Point(-1, -1)) break;
                }
                diam.push_back(sqrt(Xdistance*Xdistance*(second.x - first.x)*(second.x - first.x) +
                                    Ydistance*Ydistance*(second.y - first.y)*(second.y - first.y)));
            }
        }
    }
    for (int i = 0; i < diam.size(); ++i) if (diam[i] > 1) validDiam.push_back(diam[i]);

    int diamCnt = validDiam.size();
    sort(validDiam.begin(), validDiam.end());
    double diamMax = validDiam[diamCnt - 1] / 10;
    int m_diam = 1, n_diam = 0;
    for (int i = 0; i < diamCnt; ++i)
    {
        if (validDiam[i] < m_diam*diamMax) ++n_diam;
        else
        {
            ++m_diam;
            diamRes.push_back(n_diam);
            n_diam = 0;
            --i;
        }
    }
    double sum_diam = 0;
    for (int i = 0; i < diamRes.size(); ++i) sum_diam += diamRes[i];
    for (int i = 0; i < diamRes.size(); ++i) diamRes[i] /= sum_diam;
    diamRes.push_back(diamMax);

    vector<vector<Point>> contours;
    vector<vector<Point>> validContours;
    findContours(skel, contours, RETR_TREE, CHAIN_APPROX_NONE);
    Mat contour(src2.rows, src2.cols, CV_8UC3, Scalar::all(0));
    for (int i = 0; i < contours.size(); ++i)
    {
        Scalar color = Scalar(rand() % 255, rand() % 255, rand() % 255);
        drawContours(contour, contours, i, color, 1, 8);
    }
    for (int i = 0; i < contours.size(); ++i) if (contours[i].size() > 1) validContours.push_back(contours[i]);

    vector<double> tortuosity;
    for (int i = 0; i < validContours.size(); ++i)
    {
        double length = validContours[i].size();
        Point start = validContours[i][0];
        Point end = validContours[i][validContours[i].size() / 2];
        if (start.x == end.x || start.y == end.y) tortuosity.push_back(1);
        else
        {
            double distance = sqrt((start.x - end.x)*(start.x - end.x) + (start.y - end.y)*(start.y - end.y));
            tortuosity.push_back(length / distance);
        }
    }

    int tortCnt = tortuosity.size();
    sort(tortuosity.begin(), tortuosity.end());
    double tortMax = tortuosity[tortCnt - 1] / 10;
    int m_tort = 1, n_tort = 0;
    for (int i = 0; i < tortCnt; ++i)
    {
        if (tortuosity[i] < m_tort*tortMax) ++n_tort;
        else
        {
            ++m_tort;
            tortRes.push_back(n_tort);
            n_tort = 0;
            --i;
        }
    }
    double sum_tort = 0;
    for (int i = 0; i < tortRes.size(); ++i) sum_tort += tortRes[i];
    for (int i = 0; i < tortRes.size(); ++i) tortRes[i] /= sum_tort;
    tortRes.push_back(tortMax);
}

/*
函数功能：绘制血管直径、弯曲度统计结果
参数说明：
返回值：
*/
void Widget::drawBarGraph()
{
    QBarCategoryAxis *axisX = new QBarCategoryAxis();
    QStringList categories;
    categories << "1" << "2" << "3" << "4" << "5" << "6" << "7" << "8" << "9" << "10";
    axisX->append(categories);
    QString text = "直径×" + QString::number(diamRes[diamRes.size()-1]) + " " + "弯曲程度×" + QString::number(tortRes[tortRes.size()-1]);
    axisX->setTitleText(text);
    m_chart->setAxisX(axisX, m_series);
    for(int i = 0; i < 10; ++i)
    {
        set0->remove(i);
        set0->insert(i, diamRes[i]);
        set1->remove(i);
        set1->insert(i, tortRes[i]);
    }
}

/*
函数功能：将Qvtkwidget当前显示内容截取为图像文件存储到textEdit中的路径
参数说明：
返回值：
*/
void Widget::saveVTKwidget()
{
    static int n = 1;

    QScreen *screen = QGuiApplication::primaryScreen();
    QPixmap pixmap = screen->grabWindow(QApplication::activeWindow()->winId(), 45, 70, 870, 425);

    QString savePath = ui->textEdit->toPlainText() + "/volume_" + QString::number(n++) + ".png";
    pixmap.save(savePath, "png");
}
