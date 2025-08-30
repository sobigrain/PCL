#include "internship.h"

#include <vtkGenericOpenGLRenderWindow.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/extract_indices.h> // ������ȡ�ָ���ĵ�
#include <pcl/visualization/pcl_visualizer.h>
#include<pcl/features/normal_3d.h>
#include <QVTKOpenGLNativeWidget.h>
#include<pcl/segmentation/extract_clusters.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <QFileDialog>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>
#include <QDebug> // ���ڴ�ӡ������Ϣ
#include <QDateTime> // Ϊ�˸���־����ʱ���.
#include <QProcess>
#include <QDir> // ���ڴ���·��
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>

internship::internship(QWidget *parent)
    : QMainWindow(parent)
{
    ui.setupUi(this);
    this->setWindowTitle(" Point Cloud Segmentation and Filtering");
    this->setWindowIcon(QIcon(":/new/prefix1/image.png"));

    cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    initializeViewer();
    connect(ui.combo_filter, QOverload<int>::of(&QComboBox::currentIndexChanged),
        ui.stackedWidget_filter, &QStackedWidget::setCurrentIndex);

    connect(ui.combo_segment, QOverload<int>::of(&QComboBox::currentIndexChanged),
        ui.stackedWidget_segment, &QStackedWidget::setCurrentIndex);

    // ��ʼ���˲��㷨������
    //ui.combo_filter->addItem("VoxelGrid");
    //ui.combo_filter->addItem("StatisticalOutlierRemoval"); // Ϊ�Ժ���չ��׼��

    //// ��ʼ���ָ��㷨������
    //ui.combo_segment->addItem("RANSAC_Plane");
    //ui.combo_segment->addItem("EuclideanClusterExtraction"); // Ϊ�Ժ���չ��׼��
}

internship::~internship()
{
    if (ui.openGLWidget->renderWindow())
    {
        // ͨ�� static_cast �� nullptr ��ʽ��ת��Ϊ vtkRenderWindow* ���͵Ŀ�ָ��
        ui.openGLWidget->setRenderWindow(static_cast<vtkRenderWindow*>(nullptr));
    }
    //delete ui;
}

void internship::initializeViewer() // ���޸Ĵ˴�
{
    // ���� PCL ���ӻ���ʵ��
    viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));


    vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
    vtkSmartPointer<vtkGenericOpenGLRenderWindow> renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    renderWindow->AddRenderer(renderer);
    viewer.reset(new pcl::visualization::PCLVisualizer(renderer, renderWindow, "viewer", false));
    viewer->setupInteractor(ui.openGLWidget->interactor(), ui.openGLWidget->renderWindow());
    ui.openGLWidget->setRenderWindow(viewer->getRenderWindow());
    // ���ý�����������������ܿ��Ƴ����Ĺؼ�
    viewer->setupInteractor(ui.openGLWidget->interactor(), ui.openGLWidget->renderWindow());
    // ���ñ�����ɫ
    viewer->setBackgroundColor(0.1, 0.1, 0.15); // ʹ������ɫ����
    // ���һ������ϵ������۲�
    //viewer->addCoordinateSystem(1.0);
    // ˢ����ʾ
    ui.openGLWidget->update();
}

void internship::on_btn_openFile_clicked()
{
    // ���ļ��Ի������û�ѡ��PCD�ļ�
    QString fileName = QFileDialog::getOpenFileName(
        this,
        tr("�򿪵����ļ�"),
        QDir::currentPath(),
        tr("PCD Files (*.pcd)")
    );

    // ����û�ѡ�����ļ�������ز���ʾ
    if (!fileName.isEmpty()) {
        // ��QStringת��Ϊstd::string
        std::string filePath = fileName.toStdString();

        // ����PCD�ļ�
        pcl::io::loadPCDFile(filePath, *cloud);

        float mean_distance = 0.0f;
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(cloud);
        float sum_distances = 0.0f;

        for (const auto& point : cloud->points) {
            // ����ÿ���㵽������ڵľ���
            std::vector<int> indices(2);
            // ����ֻ��Ҫ��������ڵ�������
            std::vector<float> distances(2);

            //kdtree.nearestKSearch(point, 2, indices, distances);
            kdtree.nearestKSearch(point, 2, indices, distances);
            sum_distances += sqrt(distances[1]);
        }

        float avg_distance = sum_distances / cloud->size(); // ƽ������
        //float radius = avg_distance;

        log("Loaded point cloud with " + QString::number(cloud->size()) + " points.");
        log("Average distance between points: " + QString::number(avg_distance));
   
        // =======================================================================
        // +++ �������룺������Ƶ�XYZ���귶Χ +++
        // =======================================================================
        if (!cloud->empty())
        {
            // 1. ����������������ڴ洢��Сֵ�����ֵ
           

            // 2. ���� pcl::getMinMax3D ����
            pcl::getMinMax3D(*cloud, min_pt, max_pt);

            // 3. �������������log����
            log("--- Point Cloud Bounding Box ---");
            log(QString("X range: %1 to %2 (size: %3)")
                .arg(min_pt.x).arg(max_pt.x).arg(max_pt.x - min_pt.x));
            log(QString("Y range: %1 to %2 (size: %3)")
                .arg(min_pt.y).arg(max_pt.y).arg(max_pt.y - min_pt.y));
            log(QString("Z range: %1 to %2 (size: %3)")
                .arg(min_pt.z).arg(max_pt.z).arg(max_pt.z - min_pt.z));
            log("---------------------------------");
        }
        // =======================================================================
        // +++ ����������� +++
        // =======================================================================





        // ���֮ǰ��ӵĵ��ƣ�����еĻ���
        viewer->removePointCloud("sample cloud");

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorHandler(cloud, 255,215,0); // ���õ�����ɫΪ��ɫ

        // ����µĵ���
        viewer->addPointCloud(cloud, colorHandler, "sample cloud");

        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "sample cloud"); // ���õ��ƴ�СΪ1



        viewer->resetCamera(); // �����ӽ�
        ui.openGLWidget->renderWindow()->Render();
        // ����OpenGLС��������ʾ�µĵ���
        ui.openGLWidget->update();
    }
}

void internship::on_btn_run_filter_clicked()
{
    // --- 1. ��ȫ��� ---
    if (!cloud || cloud->empty())
    {
        log("Error: Point cloud is empty, cannot perform filtering!");
        // ��Ҳ������������QMessageBox����һ����ʾ����û�
        // QMessageBox::warning(this, "����", "���ȼ���һ�������ļ���");
        return;
    }

    // --- 2. ���� ComboBox ��ѡ��������ִ���ĸ��㷨 ---
    QString current_algo = ui.combo_filter->currentText();

    if (current_algo == "VoxelGrid")
    {
        // --- 3. �� stackedWidget �л�ȡ VoxelGrid �Ĳ��� ---

        // ��ȡ��ǰ stackedWidget �ĵ�һҳ (VoxelGrid�Ĳ���ҳ)
        QWidget* voxel_page = ui.stackedWidget_filter->widget(0);
        
        // �Ӹ�ҳ���в���������Ҫ�� QDoubleSpinBox
        // Ϊ�˴������׳����ø��ؼ�����һ��Ψһ�� objectName������ "spinBox_leafSize"
        // �����û�����ã�findChild Ҳ���ҵ���һ���������͵Ŀؼ����������˸��á�
        auto* leafSizeSpinBox = voxel_page->findChild<QDoubleSpinBox*>();

        if (!leafSizeSpinBox)
        {
            log("Error: Could not find leaf size input box in UI!");
            return;
        }

        double leaf_size = leafSizeSpinBox->value();

        log("Running VoxelGrid filter with leaf size: " + QString::number(leaf_size));


        // --- 4. ִ�� PCL �����˲����� ---

        // ����һ�����ڴ洢�˲������ĵ��ƶ���
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

        // ���� VoxelGrid �˲�������
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud); // �����������
        sor.setLeafSize(leaf_size, leaf_size, leaf_size); // �������ش�С
        sor.filter(*cloud_filtered); // ִ���˲�������洢�� cloud_filtered

        qDebug() << "Filtering complete! Original points: " << cloud->size()
            << ", Filtered points:" << cloud_filtered->size();

        log("Filtering complete! Original points: " + QString::number(cloud->size()));
        log("Filtering complete! Filtered points:" + QString::number(cloud_filtered->size()));


        // ���֮ǰ��ӵĵ��ƣ�����еĻ���
        viewer->removeAllPointClouds();

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorHandler(cloud_filtered, 0, 215, 0); // ���õ�����ɫΪ��ɫ

        // ����µĵ���
        viewer->addPointCloud(cloud_filtered, colorHandler, "sample cloud");

        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud"); // ���õ��ƴ�СΪ1

        ui.openGLWidget->renderWindow()->Render();
        ui.openGLWidget->update();

        // (��ѡ) ����Խ��˲���ĵ�����Ϊ��һ�β���������
        // cloud = cloud_filtered; 
        // ��Ҫע�⣬��Ḳ�ǵ�ԭʼ���ơ����õ����������û�ѡ������ĸ����ơ�
    }
    else if(current_algo == "passthrough") {

        QWidget* passthrough_pagez = ui.stackedWidget_filter->widget(1);
        auto* SpinBoxz = passthrough_pagez->findChild<QDoubleSpinBox*>("doubleSpinBox_11");
        double leaf_size_z = SpinBoxz->value();

        QWidget* passthrough_pagey = ui.stackedWidget_filter->widget(1);
        auto* SpinBoxy = passthrough_pagey->findChild<QDoubleSpinBox*>("doubleSpinBox_12");
        double leaf_size_y = SpinBoxy->value();

        QWidget* passthrough_pagex = ui.stackedWidget_filter->widget(1);
        auto* SpinBoxx = passthrough_pagex->findChild<QDoubleSpinBox*>("doubleSpinBox_13");
        double leaf_size_x = SpinBoxx->value();
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        if (leaf_size_z) {
           
            // ���� passthrough �˲�������
            pcl::PassThrough<pcl::PointXYZ> pass;
            pass.setInputCloud(cloud); // �����������
            pass.setFilterFieldName("z"); // �����˲��ֶ�
            pass.setFilterLimits(leaf_size_z, max_pt.z); // �����˲���Χ
            pass.filter(*cloud_filtered); // ִ���˲�������洢�� cloud_filtered
            pcl::io::savePCDFile("cloud_filtered_z.pcd", *cloud_filtered);
        }

        if (leaf_size_y) {
            //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
            // ���� passthrough �˲�������
            pcl::PassThrough<pcl::PointXYZ> pass;
            pass.setInputCloud(cloud); // �����������
            pass.setFilterFieldName("y"); // �����˲��ֶ�
            pass.setFilterLimits(leaf_size_y, max_pt.y); // �����˲���Χ
            pass.filter(*cloud_filtered); // ִ���˲�������洢�� cloud_filtered

            pcl::io::savePCDFile("cloud_filtered_y.pcd", *cloud_filtered);
        }

        if (leaf_size_x) {
            //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
            // ���� passthrough �˲�������
            pcl::PassThrough<pcl::PointXYZ> pass;
            pass.setInputCloud(cloud); // �����������
            pass.setFilterFieldName("x"); // �����˲��ֶ�
            pass.setFilterLimits(leaf_size_x, max_pt.x); // �����˲���Χ
            pass.filter(*cloud_filtered); // ִ���˲�������洢�� cloud_filtered
            pcl::io::savePCDFile("cloud_filtered_x.pcd", *cloud_filtered);
        }

        qDebug() << "Filtering complete! Original points: " << cloud->size()
            << ", Filtered points:" << cloud_filtered->size();

        log("Filtering complete! Original points: " + QString::number(cloud->size()));
        log("Filtering complete! Filtered points:" + QString::number(cloud_filtered->size()));


        // ���֮ǰ��ӵĵ��ƣ�����еĻ���
        viewer->removeAllPointClouds();

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorHandler(cloud_filtered, 0, 215, 0); // ���õ�����ɫΪ��ɫ

        // ����µĵ���
        viewer->addPointCloud(cloud_filtered, colorHandler, "sample cloud");

        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud"); // ���õ��ƴ�СΪ1

        ui.openGLWidget->renderWindow()->Render();

        ui.openGLWidget->update();
    }
    else
    {
        log("Error: Selected filter algorithm not yet implemented!");
        //
        // qDebug() << "Selected feature not yet implemented:" << current_algo;
        // QMessageBox::information(this, "��ʾ", "���˲�������δʵ�֣�");
    }
}


void internship::log(const QString& message)
{
    // 1. ׼��Ҫ��ʾ��UI�ϵ�������־��Ϣ
    QString log_message = QDateTime::currentDateTime().toString("[yyyy-MM-dd hh:mm:ss] ") + message;

    // 2. ��ӵ� QListWidget ��
    ui.listWidget_clouds->addItem(log_message);

    // 3. (��ѡ���Ƽ�) �Զ��������ײ���ȷ�����µ���־���ǿɼ���
    ui.listWidget_clouds->scrollToBottom();

    // 4. (��ѡ���Ƽ�) ��Ȼʹ�� qDebug() ��ӡ��VS��������ڣ��������
    qDebug() << message;
}


void internship::on_btn_run_segment_clicked() // ������UI�� "Run Segment" ��ť�Ĳۺ���
{
    // --- 1. ��ȫ��� ---
    if (!cloud || cloud->empty())
    {
        log("Error: Point cloud is empty, cannot perform segmentation!");
        // QMessageBox::warning(this, "����", "���ȼ���һ�������ļ���");
        return;
    }

    // --- 2. ������ȷ�� ComboBox (combo_segment) �������㷨 ---
    QString current_algo = ui.combo_segment->currentText(); // *** ���� #1: ʹ�÷ָ��㷨�������� ***

    if (current_algo == "sac_segmentation") // �����������������������һ��
    {
        // --- 3. ����ȷ�� stackedWidget (stackedWidget_segment) �л�ȡ���� ---

        // ��ȡ��ǰ stackedWidget ��ҳ��
        QWidget* current_page = ui.stackedWidget_segment->widget(0);
        // *** ���� #2: ʹ�÷ָ�� stackedWidget������ȡ��ǰҳ ***

        if (!current_page)
        {
            log("Error: Could not find the current parameter page in UI!");
            return;
        }

        // ʹ�� objectName ��׼���ҿؼ�
        auto* mtypeCombo = current_page->findChild<QComboBox*>("comboBox_2"); // *** ���� #3: ʹ��objectName ***
        auto* methodCombo = current_page->findChild<QComboBox*>("comboBox_3");
        auto* distanceSpinBox = current_page->findChild<QDoubleSpinBox*>("doubleSpinBox_2");

        if (!mtypeCombo || !methodCombo || !distanceSpinBox)
        {
            log("Error: Could not find parameter controls in UI! Please check objectNames.");
            return;
        }

        QString model_type_str = mtypeCombo->currentText();
        QString method_type_str = methodCombo->currentText();
        double distance_threshold = distanceSpinBox->value();

        log("Running SAC Segmentation with Model=" + model_type_str +
            ", Method=" + method_type_str +
            ", Distance=" + QString::number(distance_threshold));

        // --- 4. ִ�� PCL ���ķָ���� ---

        // �����ָ����
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        
        // A. ���÷ָ����
        seg.setOptimizeCoefficients(true);
        seg.setMaxIterations(1000);
        // ��QStringת��ΪPCLö������
        if (model_type_str == "Plane")
        {
            seg.setModelType(pcl::SACMODEL_PLANE);
        }
        else { /* ...����Ϊ����ģ�����else if... */ 
            seg.setModelType(pcl::SACMODEL_CIRCLE3D);
        }

        if (method_type_str == "RANSAC")
        {
            seg.setMethodType(pcl::SAC_RANSAC);
        }
        else { /* ...����Ϊ�����������else if... */ }

        seg.setDistanceThreshold(distance_threshold);
        seg.setInputCloud(cloud);

        // B. ִ�зָ�
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.size() == 0)
        {
            log("Error: Could not estimate a planar model for the given dataset.");
            // QMessageBox::warning(this, "����", "�޷��ڵ�������ϳ�ƽ�棡");
            return;
        }

        log("Segmentation complete! Found " + QString::number(inliers->indices.size()) + " inlier points.");
        // ��Ҳ���԰�ģ��ϵ����ӡ����
        // std::cerr << "Model coefficients: " << *coefficients << std::endl;

        // --- 5. ����������ȡ�ڵ����㣬�����ӻ� ---

        //// ����һ�����ƶ������洢�ڵ� (ƽ���ϵĵ�)
        //pcl::ExtractIndices<pcl::PointXYZ> extract;
        //pcl::PointCloud<pcl::PointXYZ>::Ptr SACcloud_xyz_filtered(new pcl::PointCloud<pcl::PointXYZ>());
        //// ����������ƣ����������ָ���Ǹ����ƣ�
        //extract.setInputCloud(cloud);

        //// ����Ҫ��ȡ���������ָ��㷨�õ����ڵ㣩
        //extract.setIndices(inliers);
        //extract.setNegative(true); // ȡ�����㣡
        //extract.filter(*SACcloud_xyz_filtered);


        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inliers(new pcl::PointCloud<pcl::PointXYZ>);
        // ������һ�����ƶ������洢��� (ƽ����ĵ�)
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_outliers(new pcl::PointCloud<pcl::PointXYZ>);

        // ����������ȡ��
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);

        // A. ��ȡ�ڵ�
        extract.setNegative(false); // false��ʾ��ȡ������Ӧ�ĵ�
        extract.filter(*cloud_inliers);

        // ��ģ��ϵ������ȡԲ�Ĳ���
        //float center_x = coefficients->values[0];
        //float center_z = coefficients->values[1];
        //float radius = coefficients->values[2];
        //float radius_sq = radius * radius; // Ԥ�ȼ���뾶��ƽ�������Ч��
        //log("dasdas" + QString::number(center_x) +"cssd" + QString::number(center_z)+"dsdsd" + QString::number(radius));
        //pcl::PointIndices::Ptr cylinder_inliers(new pcl::PointIndices);
        //cylinder_inliers->indices.reserve(cloud->size());

        //// ��������ԭʼ3D����
        //for (size_t i = 0; i < cloud->size(); ++i)
        //{
        //    const auto& point = cloud->points[i];

        //    // **�ؼ�����XZƽ���ϼ��㵽Բ�ĵľ���**
        //    float dist_in_xz_plane_sq = std::pow(point.z - center_z, 2) + std::pow(point.x - center_x, 2);

        //    // ������2D����С�ڰ뾶����ô��������Y�᷽�������Բ������
        //    if (dist_in_xz_plane_sq < std::pow(radius, 2))
        //    {
        //        cylinder_inliers->indices.push_back(i);
        //    }
        //}

        //// ֮���� ExtractIndices �Ƴ���Щ��...
        //pcl::ExtractIndices<pcl::PointXYZ> extractx;
        //extractx.setInputCloud(cloud);
        //extractx.setIndices(cylinder_inliers);
        //extractx.setNegative(false);
        //extractx.filter(*cloud_inliers);




        // B. ��ȡ���
        extract.setNegative(true);  // true��ʾ��ȡ����֮��ĵ�
        extract.filter(*cloud_outliers);

        pcl::io::savePCDFile("SACcloud_xyz_filtered.pcd", *cloud_outliers);
        // --- 6. ��PCL Viewer�и�����ʾ ---

        // �����Ƴ�����֮ǰ�ĵ��ƣ�ȷ����ͼ�ɾ�
        viewer->removeAllPointClouds();

        // ���ӻ��ڵ� (ƽ��)������ɫ��ʾ
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> inlier_color(cloud_inliers, 0, 255, 0); // Green
        viewer->addPointCloud<pcl::PointXYZ>(cloud_inliers, inlier_color, "cloud_inliers sample cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "outlier_color sample cloud");

        // ���ӻ���� (����ĵ�)���ð�ɫ��ʾ
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> outlier_color(cloud_outliers, 255, 255, 255); // White
        viewer->addPointCloud<pcl::PointXYZ>(cloud_outliers, outlier_color, "outlier_color sample cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "outlier_color sample cloud");

        // ˢ����Ⱦ����
        ui.openGLWidget->renderWindow()->Render();
        ui.openGLWidget->update();
    }
    else if (current_algo == "region_growing") // �����������������������һ��
    {
        QWidget* current_page = ui.stackedWidget_segment->widget(2);
        // *** ���� #2: ʹ�÷ָ�� stackedWidget������ȡ��ǰҳ ***

        if (!current_page)
        {
            log("Error: Could not find the current parameter page in UI!");
            return;
        }

        // ʹ�� objectName ��׼���ҿؼ�
        //auto* mtypeCombo = current_page->findChild<QComboBox*>("comboBox_2"); // *** ���� #3: ʹ��objectName ***
        auto* Misize = current_page->findChild<QDoubleSpinBox*>("doubleSpinBox_3"); // *** ���� #3: ʹ��objectName ***
        auto* masize = current_page->findChild<QDoubleSpinBox*>("doubleSpinBox_4");
        auto* nonet = current_page->findChild<QDoubleSpinBox*>("doubleSpinBox_5");
        auto* SThreshould = current_page->findChild<QDoubleSpinBox*>("doubleSpinBox_6");
        auto* CThreshold = current_page->findChild<QDoubleSpinBox*>("doubleSpinBox_7");

        if (!Misize || !masize || !nonet || !SThreshould || !CThreshold)
        {
            log("Error: Could not find parameter controls in UI! Please check objectNames.");
            return;
        }

     
        double min_size = Misize->value();
        double max_size = masize->value();
        double num_neighbours = nonet->value();
        double smoothness_threshold = SThreshould->value();
        double curvature_threshold = CThreshold->value();


        log("Running Growing Region Segmentation with Model=" + current_algo +
            ", Misize=" + QString::number(min_size) +
            ", masize=" + QString::number(max_size)+ ", nonet=" + QString::number(num_neighbours) + 
            ", SThreshould="+ QString::number(smoothness_threshold) + ", CThreshold=" + QString::number(curvature_threshold));


        //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        log("Running Normal Estimation.....");
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;
        norm_est.setInputCloud(cloud);

        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        norm_est.setSearchMethod(tree);

        norm_est.setKSearch(num_neighbours);
        norm_est.compute(*normals);

        pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
        reg.setInputCloud(cloud);
        reg.setInputNormals(normals);
        reg.setMinClusterSize(min_size);
        reg.setMaxClusterSize(max_size);
        reg.setNumberOfNeighbours(num_neighbours);
        reg.setSmoothnessThreshold(smoothness_threshold / 180.0 * M_PI);
        reg.setCurvatureThreshold(curvature_threshold);

        std::vector<pcl::PointIndices> cluster_indices;
        reg.extract(cluster_indices);
        log("Running Region Growing.....");
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();

        // --- ���� 2: ��鲢����� ---
        if (cluster_indices.empty())
        {
            std::cout << "No clusters were found." << std::endl;
            return ;
        }

        std::cout << "Found " << cluster_indices.size() << " clusters before sorting." << std::endl;

        // **�����ġ�ʹ�� std::sort �� Lambda ���ʽ�Դذ���С���н�������**
        std::sort(cluster_indices.begin(), cluster_indices.end(),
            [](const pcl::PointIndices& a, const pcl::PointIndices& b) {
                return a.indices.size() > b.indices.size();
            }
        );

        std::cout << "Clusters have been sorted by size in descending order." << std::endl;

        // --- ���� 3: ��ȡ���Ĵ� ---

        // ����󣬵�һ��Ԫ�ؾ������Ĵ�
        const pcl::PointIndices& largest_cluster = cluster_indices[0];

        // ����һ���µĵ��ƶ��������洢���صĵ�
        pcl::PointCloud<pcl::PointXYZ>::Ptr largest_cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        // �������ص����е������
        for (const auto& idx : largest_cluster.indices) {
            // ������������ԭʼ�����аѵ���ӵ��µĵ�����
            largest_cluster_cloud->points.push_back(cloud->points[idx]);
        }

        largest_cluster_cloud->width = largest_cluster_cloud->points.size();
        largest_cluster_cloud->height = 1;
        largest_cluster_cloud->is_dense = true;

        // --- ���� 4: ʹ�û���֤��� ---
        std::cout << "The largest cluster contains " << largest_cluster_cloud->points.size() << " points." << std::endl;

        // ���ڣ�`largest_cluster_cloud` ��������Ҫ���Ǹ��������Ĵصĵ���
        // ����Զ������п��ӻ������浽�ļ��������һ������
         pcl::io::savePCDFileASCII("ORIXYZ.pcd", *largest_cluster_cloud);



        //if (cluster_indices.empty())
        //{
        //    log("Error: Could not estimate any clusters for the given dataset.");
        //    // QMessageBox::warning(this, "����", "�޷��ڵ������ҵ����࣡");
        //    return;
        //}

        //log("Segmentation complete! Found " + QString::number(cluster_indices.size()) + " clusters.");

        //// --- 5. ����������ȡ�ڵ����㣬�����ӻ� ---

        //// ����һ�����ƶ������洢�ڵ� (ƽ���ϵĵ�)
        //pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();

        //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inliers(new pcl::PointCloud<pcl::PointXYZ>);
        //// ������һ�����ƶ������洢��� (ƽ����ĵ�)
        //// 3. ʹ�� copyPointCloud ����ת��
        //if (colored_cloud && !colored_cloud->empty())
        //{
        //    pcl::copyPointCloud(*colored_cloud, *cloud_inliers);
        //}

        //pcl::io::savePCDFile("ORIXYZ.pcd", *colored_cloud);

        viewer->removeAllPointClouds();
        // ���ӻ��ڵ� (ƽ��)������ɫ��ʾ
        //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> inlier_color(cloud_inliers, 0, 255, 0); // Green
        viewer->addPointCloud<pcl::PointXYZRGB>(colored_cloud, "cloud_inliers sample cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_inliers sample cloud");


        viewer->addPointCloud<pcl::PointXYZ>(largest_cluster_cloud, "largest_cluster_cloud sample cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "largest_cluster_cloud sample cloud");


        // ˢ����Ⱦ����
        ui.openGLWidget->renderWindow()->Render();
        ui.openGLWidget->update();
    }
    else if (current_algo == "extract_clusters") {
        QWidget* current_page = ui.stackedWidget_segment->widget(1);
        // *** ���� #2: ʹ�÷ָ�� stackedWidget������ȡ��ǰҳ ***

        if (!current_page)
        {
            log("Error: Could not find the current parameter page in UI!");
            return;
        }

        // ʹ�� objectName ��׼���ҿؼ�
        //auto* mtypeCombo = current_page->findChild<QComboBox*>("comboBox_2"); // *** ���� #3: ʹ��objectName ***
        auto* Misize = current_page->findChild<QDoubleSpinBox*>("doubleSpinBox_8"); // *** ���� #3: ʹ��objectName ***
        auto* masize = current_page->findChild<QDoubleSpinBox*>("doubleSpinBox_9");
        auto* Tolerance = current_page->findChild<QDoubleSpinBox*>("doubleSpinBox_10");


        if (!Misize || !masize || !Tolerance)
        {
            log("Error: Could not find parameter controls in UI! Please check objectNames.");
            return;
        }


        double min_size = Misize->value();
        double max_size = masize->value();
        double num_neighbours = Tolerance->value();



        log("Running Extract Clusters Segmentation with Model=" + current_algo +
            ", Misize=" + QString::number(min_size) +
            ", masize=" + QString::number(max_size) + ", Tolerance=" + QString::number(num_neighbours)
            );

        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud);


        pcl::EuclideanClusterExtraction<pcl::PointXYZ> extract_clusters;
        extract_clusters.setInputCloud(cloud);
        extract_clusters.setSearchMethod(tree);
        extract_clusters.setClusterTolerance(num_neighbours);
        extract_clusters.setMinClusterSize(min_size);
        extract_clusters.setMaxClusterSize(max_size);

        std::vector<pcl::PointIndices> clusters;
        extract_clusters.extract(clusters);
        log("Running Extract Clusters Segmentation over!");
        pcl::PointCloud<pcl::PointXYZ>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        


        // --- ���� 2: ��鲢����� ---
        if (clusters.empty())
        {
            log("No clusters were found.");
            return;
        }

        log("Found :" + QString::number(clusters.size()) + " clusters.");
        //std::cout << "Found " << clusters.size() << " clusters before sorting." << std::endl;

        // **�����ġ�ʹ�� std::sort �� Lambda ���ʽ�Դذ���С���н�������**
        std::sort(clusters.begin(), clusters.end(),
            [](const pcl::PointIndices& a, const pcl::PointIndices& b) {
                return a.indices.size() > b.indices.size();
            }
        );

        log("Clusters have been sorted by size in descending order.");
        //std::cout << "Clusters have been sorted by size in descending order." << std::endl;

        // --- ���� 3: ��ȡ���Ĵ� ---

        // ����󣬵�һ��Ԫ�ؾ������Ĵ�
        const pcl::PointIndices& largest_cluster = clusters[0];

        // ����һ���µĵ��ƶ��������洢���صĵ�
        pcl::PointCloud<pcl::PointXYZ>::Ptr largest_cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        // --- ��ʼ��ȡ���� ---

// 1. ���� ExtractIndices �˲�������
        pcl::ExtractIndices<pcl::PointXYZ> extract;

        // 2. �����������
        extract.setInputCloud(cloud);

        // 3. ����Ҫ���ų����ĵ�������������ص�����
        // ע�⣺setIndices ���յ�������ָ�룬����������Ҫ���� largest_cluster �ĵ�ַ
        // ���ߴ���һ������ָ������װ�������򵥵������Ǵ���һ�� PointIndices::Ptr��
        pcl::PointIndices::Ptr largest_cluster_indices_ptr(new pcl::PointIndices(largest_cluster));
        extract.setIndices(largest_cluster_indices_ptr);

        // 4. *** �ؼ����裺����Ϊ������ȡ ***
        extract.setNegative(true);

        // 5. ִ���˲���������洢����ĵ�����
        extract.filter(*largest_cluster_cloud);

        pcl::io::savePCDFileASCII("end.pcd", *largest_cluster_cloud);

        // �������ص����е������
        for (const auto& idx : largest_cluster.indices) {
            // ������������ԭʼ�����аѵ���ӵ��µĵ�����
            largest_cluster_cloud->points.push_back(cloud->points[idx]);
        }

        largest_cluster_cloud->width = largest_cluster_cloud->points.size();
        largest_cluster_cloud->height = 1;
        largest_cluster_cloud->is_dense = true;



        // --- ���� 4: ʹ�û���֤��� ---
        log("The largest cluster contains :" + QString::number(largest_cluster_cloud->points.size()) + " points.");
        //std::cout << "The largest cluster contains " << largest_cluster_cloud->points.size() << " points." << std::endl;

        // ���ڣ�`largest_cluster_cloud` ��������Ҫ���Ǹ��������Ĵصĵ���
        // ����Զ������п��ӻ������浽�ļ��������һ������
        //pcl::io::savePCDFileASCII("ORIXYZ.pcd", *largest_cluster_cloud);

        viewer->removeAllPointClouds();
        // ���ӻ��ڵ� (ƽ��)������ɫ��ʾ
        //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> inlier_color(cloud_inliers, 0, 255, 0); // Green
        //viewer->addPointCloud<pcl::PointXYZRGB>(largest_cluster_cloud, "cloud_inliers sample cloud");
        //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_inliers sample cloud");

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> inlier_color(largest_cluster_cloud, 255, 240, 0); // Green
        viewer->addPointCloud<pcl::PointXYZ>(largest_cluster_cloud, inlier_color, "largest_cluster_cloud sample cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "largest_cluster_cloud sample cloud");


        // ˢ����Ⱦ����
        ui.openGLWidget->renderWindow()->Render();
        ui.openGLWidget->update();

    }
else 
    {
        log("Error: Selected segmentation algorithm not yet implemented!");
    }
}


void internship::on_actionRun_triggered()
{
    log("Average distance between points: ");
    qDebug() << "Run action triggered! Starting PointNeXt Python script...";

    // --- 1. ����·�� ---

    // **�ؼ���ָ��������⻷���е�python.exe������·��**
    // ע�⣺��C++�ַ����У���б�� \ ��Ҫת��� \\
    QString python_executable = "C:\\Path\\To\\Your\\venv\\Scripts\\python.exe"; 
    // ���� conda: "C:\\Users\\YourName\\anaconda3\\envs\\pointnext_env\\python.exe"

    // �������Python�ű���·��
    QString script_path = "C:\\PCLandC\\Conv_models\\models\\cws.py";
    QString python_executable = "C:\\Miniconda\\python.exe";
    // (��ѡ) ������������ļ���·��
    QString input_pcd_path = "C:/temp/input_for_python.pcd";
    QString output_pcd_path = "C:/temp/output_from_python.pcd";

    // --- 2. ׼��Ҫ���ݸ�Python�ű��������в��� ---
    QStringList args;
    args << script_path; // ��һ���������ǽű�����
    args << "--input" << input_pcd_path;
    args << "--output" << output_pcd_path;
    // ... �����ű���Ҫ�������κβ��� ...

    // --- 3. ���������� QProcess ---
    QProcess* pythonProcess = new QProcess(this);

    // (��ѡ) �����źţ��Ա���Python�ű�������õ�֪ͨ
    connect(pythonProcess, &QProcess::finished, this, [=](int exitCode, QProcess::ExitStatus exitStatus) {
        qDebug() << "Python script finished with exit code:" << exitCode;
        if (exitStatus == QProcess::NormalExit && exitCode == 0) {
            // �ű��ɹ�����
            qDebug() << "Successfully processed point cloud. Loading result from:" << output_pcd_path;
            // ������д���� output_pcd_path ����ʾ�Ĵ���
        }
        else {
            // �ű����г���
            qDebug() << "Python script failed!";
            // ��ȡ����ʾ������Ϣ
            QByteArray error_output = pythonProcess->readAllStandardError();
            qDebug() << "Error output:" << QString(error_output);
        }
        pythonProcess->deleteLater(); // ����QProcess����
        });

    // (��ѡ) ��ȡ��׼���
    connect(pythonProcess, &QProcess::readyReadStandardOutput, this, [=]() {
        QByteArray output = pythonProcess->readAllStandardOutput();
        qDebug() << "Python stdout:" << QString(output);
        });

    // �������̣�
    pythonProcess->start(python_executable, args);

    // ����Ƿ�ɹ�����
    if (!pythonProcess->waitForStarted()) {
        qDebug() << "Failed to start Python process!";
        qDebug() << "Error:" << pythonProcess->errorString();
        delete pythonProcess;
        log("Average distance between points222: ");
    }
    log("Average distance between points:333 ");
}
