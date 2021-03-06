# Software-development-of-photoacoustic-imaging-system

一、项目简介

光声成像是近年迅速发展的一种无损生物医学影像技术。光声成像利用短脉冲光源作为激发源照射生物组织，生物组织吸收光能以后产生光致超声信号（光声信号），携带组织光学吸收信息的超声信号被接收后通过不同成像算法反演出生物组织内部吸收结构的可视化图像。该成像技术结合了光学成像以及超声成像的优点，其成像具备光学高对比度和超声学高穿透深度的优势，可提供反映组织生理病理特异性的结构和功能信息，极具临床应用价值。

此项目的预期目标是开发一套集数据采集、数据处理、数据投影和数据存储于一体，程序稳定可靠运行，满足用户需求，且界面专业、美观的成像系统。特别的，为了实现多模块并行，应涉及到多线程和并发队列编程。

基本功能包括：

1）为了适用不同场景下成像区域的生理特性，应实现数据采集卡的实时控制，量程、采样率可调；

2）为了观测不用深度范围的目标图像，在数据处理部分，应实现Aline截取点、截取长度可调，深度切片起始点、终止点可调；

3）为了回溯成像结果，应实现断层图像、最大值投影图像、原始点数据存储；

4）为了在光声成像前准确寻找目标成像区域，应实现白光摄像头图像显示；

5）为了快速检测目标区域，应实现断层图像、最大值投影图像、深度切片图像实时显示；

6）为了多视角观测目标区域成像结果，应实现断层图像三维体重建实时显示。

新增重建部分功能：（2021.06）

7）根据XZ最大值投影选择深度范围，XY深度剥除最大值投影；

8）基于第一个极值，对Aline进行结构分层；

9）提取血管参数：密度、直径、弯曲度。

二、实现方法

基于C++语言，利用OpenCV实现图像处理、Qt实现图形用户界面、VTK实现三维体重建。本项目是一个经典的生产者-消费者模型问题。

1）多线程方法

Qt提供了简便的方法实现多线程并发，利用QtConcurrent::run(QThreadPool *pool, Function function, ...) ，可在单独的线程中运行function。线程是从QThreadPool池中获取的。应注意，函数可能不会立即运行，函数只在线程可用时运行。

2）并发队列

https://github.com/cameron314/concurrentqueue 提供了高性能的并发队列实现。本项目中用到的API：

bool enqueue_bulk(item_first, count)  如果需要，分配更多的内存

size_t try_dequeue_bulk(item_first, max)  尝试出队列(不分配内存)

3）图像处理

OpenCV是一个跨平台的计算机视觉和机器学习软件库，可以运行在多个操作系统上，它轻量且高效，由一系列C函数和C++类构成。提供了包括C++和Python等语言的接口，实现了图像处理和计算机视觉方面的很多通用算法。本项目中涉及到图像像素投影、图像插值、图像滤波和摄像头视频读取。

4）图形用户界面

Qt是一个跨平台的C++图形用户界面应用程序开发框架，良好的封装机制使得它的模块化程度非常高，对于用户开发非常的方便。

5）三维体重建

VTK是一个开源的、主要用于三维计算机图形学、图像处理和可视化的软件系统。一个典型的VTK程序的渲染过程：需要一个数据源(source)，将这个数据源通过映射器(mapper)映射到原始图像，再创建一个演员(actor)来表示数据源，然后将演员添加到渲染器(renderer)，最后创建一个渲染窗口(renderwindow)来显示渲染器里的内容。
