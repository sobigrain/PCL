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
    void initializeViewer(); // ���޸Ĵ˴�

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    pcl::visualization::PCLVisualizer::Ptr viewer;
    pcl::PointXYZ min_pt, max_pt;

    // Qt�Ĳۺ�����������
private slots:
    /**
     * @brief ��Ӧ "loadPcdButton" ��ť����¼��Ĳۺ�����
     *        Qt �� `on_������_�ź���` ����Լ������ʵ���źźͲ۵��Զ����ӡ�
     */
    void on_btn_openFile_clicked();
    void on_actionRun_triggered();
    void on_btn_run_segment_clicked();
    void log(const QString& message); // ���ǵ�����־����

    void on_btn_run_filter_clicked();
};

