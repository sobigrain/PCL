#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_internship.h"

#include <vtkGenericOpenGLRenderWindow.h>
#include <QVTKOpenGLNativeWidget.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
class internship : public QMainWindow
{
    Q_OBJECT

public:
    internship(QWidget *parent = nullptr);
    ~internship();

private:
    Ui::internshipClass ui;
    void initializeViewer(); // 仅修改此处

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    pcl::visualization::PCLVisualizer::Ptr viewer;
    pcl::PointXYZ min_pt, max_pt;

    // Qt的槽函数声明区域
private slots:
    /**
     * @brief 响应 "loadPcdButton" 按钮点击事件的槽函数。
     *        Qt 的 `on_对象名_信号名` 命名约定可以实现信号和槽的自动连接。
     */
    void on_btn_openFile_clicked();
    void on_actionRun_triggered();
    void on_btn_run_segment_clicked();
    void log(const QString& message); // 我们的新日志函数

    void on_btn_run_filter_clicked();
};

