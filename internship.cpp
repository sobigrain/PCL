#include "internship.h"

#include <vtkGenericOpenGLRenderWindow.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/extract_indices.h> // 用于提取分割出的点
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
#include <QDebug> // 用于打印调试信息
#include <QDateTime> // 为了给日志加上时间戳.
#include <QProcess>
#include <QDir> // 用于处理路径
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

    // 初始化滤波算法下拉框
    //ui.combo_filter->addItem("VoxelGrid");
    //ui.combo_filter->addItem("StatisticalOutlierRemoval"); // 为以后扩展做准备

    //// 初始化分割算法下拉框
    //ui.combo_segment->addItem("RANSAC_Plane");
    //ui.combo_segment->addItem("EuclideanClusterExtraction"); // 为以后扩展做准备
}

internship::~internship()
{
    if (ui.openGLWidget->renderWindow())
    {
        // 通过 static_cast 将 nullptr 显式地转换为 vtkRenderWindow* 类型的空指针
        ui.openGLWidget->setRenderWindow(static_cast<vtkRenderWindow*>(nullptr));
    }
    //delete ui;
}

void internship::initializeViewer() // 仅修改此处
{
    // 创建 PCL 可视化器实例
    viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));


    vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
    vtkSmartPointer<vtkGenericOpenGLRenderWindow> renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    renderWindow->AddRenderer(renderer);
    viewer.reset(new pcl::visualization::PCLVisualizer(renderer, renderWindow, "viewer", false));
    viewer->setupInteractor(ui.openGLWidget->interactor(), ui.openGLWidget->renderWindow());
    ui.openGLWidget->setRenderWindow(viewer->getRenderWindow());
    // 设置交互器，这是让鼠标能控制场景的关键
    viewer->setupInteractor(ui.openGLWidget->interactor(), ui.openGLWidget->renderWindow());
    // 设置背景颜色
    viewer->setBackgroundColor(0.1, 0.1, 0.15); // 使用深蓝色背景
    // 添加一个坐标系，方便观察
    //viewer->addCoordinateSystem(1.0);
    // 刷新显示
    ui.openGLWidget->update();
}

void internship::on_btn_openFile_clicked()
{
    // 打开文件对话框，让用户选择PCD文件
    QString fileName = QFileDialog::getOpenFileName(
        this,
        tr("打开点云文件"),
        QDir::currentPath(),
        tr("PCD Files (*.pcd)")
    );

    // 如果用户选择了文件，则加载并显示
    if (!fileName.isEmpty()) {
        // 将QString转换为std::string
        std::string filePath = fileName.toStdString();

        // 加载PCD文件
        pcl::io::loadPCDFile(filePath, *cloud);

        float mean_distance = 0.0f;
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(cloud);
        float sum_distances = 0.0f;

        for (const auto& point : cloud->points) {
            // 计算每个点到其最近邻的距离
            std::vector<int> indices(2);
            // 这里只需要计算最近邻的两个点
            std::vector<float> distances(2);

            //kdtree.nearestKSearch(point, 2, indices, distances);
            kdtree.nearestKSearch(point, 2, indices, distances);
            sum_distances += sqrt(distances[1]);
        }

        float avg_distance = sum_distances / cloud->size(); // 平均距离
        //float radius = avg_distance;

        log("Loaded point cloud with " + QString::number(cloud->size()) + " points.");
        log("Average distance between points: " + QString::number(avg_distance));
   
        // =======================================================================
        // +++ 新增代码：计算点云的XYZ坐标范围 +++
        // =======================================================================
        if (!cloud->empty())
        {
            // 1. 创建两个点对象，用于存储最小值和最大值
           

            // 2. 调用 pcl::getMinMax3D 函数
            pcl::getMinMax3D(*cloud, min_pt, max_pt);

            // 3. 将结果输出到你的log窗口
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
        // +++ 新增代码结束 +++
        // =======================================================================





        // 清除之前添加的点云（如果有的话）
        viewer->removePointCloud("sample cloud");

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorHandler(cloud, 255,215,0); // 设置点云颜色为白色

        // 添加新的点云
        viewer->addPointCloud(cloud, colorHandler, "sample cloud");

        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "sample cloud"); // 设置点云大小为1



        viewer->resetCamera(); // 重置视角
        ui.openGLWidget->renderWindow()->Render();
        // 更新OpenGL小部件以显示新的点云
        ui.openGLWidget->update();
    }
}

void internship::on_btn_run_filter_clicked()
{
    // --- 1. 安全检查 ---
    if (!cloud || cloud->empty())
    {
        log("Error: Point cloud is empty, cannot perform filtering!");
        // 你也可以在这里用QMessageBox弹出一个提示框给用户
        // QMessageBox::warning(this, "错误", "请先加载一个点云文件！");
        return;
    }

    // --- 2. 根据 ComboBox 的选择来决定执行哪个算法 ---
    QString current_algo = ui.combo_filter->currentText();

    if (current_algo == "VoxelGrid")
    {
        // --- 3. 从 stackedWidget 中获取 VoxelGrid 的参数 ---

        // 获取当前 stackedWidget 的第一页 (VoxelGrid的参数页)
        QWidget* voxel_page = ui.stackedWidget_filter->widget(0);
        
        // 从该页面中查找我们需要的 QDoubleSpinBox
        // 为了代码更健壮，最好给控件设置一个唯一的 objectName，比如 "spinBox_leafSize"
        // 如果你没有设置，findChild 也能找到第一个符合类型的控件，但设置了更好。
        auto* leafSizeSpinBox = voxel_page->findChild<QDoubleSpinBox*>();

        if (!leafSizeSpinBox)
        {
            log("Error: Could not find leaf size input box in UI!");
            return;
        }

        double leaf_size = leafSizeSpinBox->value();

        log("Running VoxelGrid filter with leaf size: " + QString::number(leaf_size));


        // --- 4. 执行 PCL 核心滤波代码 ---

        // 创建一个用于存储滤波后结果的点云对象
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

        // 创建 VoxelGrid 滤波器对象
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud); // 设置输入点云
        sor.setLeafSize(leaf_size, leaf_size, leaf_size); // 设置体素大小
        sor.filter(*cloud_filtered); // 执行滤波，结果存储在 cloud_filtered

        qDebug() << "Filtering complete! Original points: " << cloud->size()
            << ", Filtered points:" << cloud_filtered->size();

        log("Filtering complete! Original points: " + QString::number(cloud->size()));
        log("Filtering complete! Filtered points:" + QString::number(cloud_filtered->size()));


        // 清除之前添加的点云（如果有的话）
        viewer->removeAllPointClouds();

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorHandler(cloud_filtered, 0, 215, 0); // 设置点云颜色为白色

        // 添加新的点云
        viewer->addPointCloud(cloud_filtered, colorHandler, "sample cloud");

        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud"); // 设置点云大小为1

        ui.openGLWidget->renderWindow()->Render();
        ui.openGLWidget->update();

        // (可选) 你可以将滤波后的点云作为下一次操作的输入
        // cloud = cloud_filtered; 
        // 但要注意，这会覆盖掉原始点云。更好的做法是让用户选择操作哪个点云。
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
           
            // 创建 passthrough 滤波器对象
            pcl::PassThrough<pcl::PointXYZ> pass;
            pass.setInputCloud(cloud); // 设置输入点云
            pass.setFilterFieldName("z"); // 设置滤波字段
            pass.setFilterLimits(leaf_size_z, max_pt.z); // 设置滤波范围
            pass.filter(*cloud_filtered); // 执行滤波，结果存储在 cloud_filtered
            pcl::io::savePCDFile("cloud_filtered_z.pcd", *cloud_filtered);
        }

        if (leaf_size_y) {
            //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
            // 创建 passthrough 滤波器对象
            pcl::PassThrough<pcl::PointXYZ> pass;
            pass.setInputCloud(cloud); // 设置输入点云
            pass.setFilterFieldName("y"); // 设置滤波字段
            pass.setFilterLimits(leaf_size_y, max_pt.y); // 设置滤波范围
            pass.filter(*cloud_filtered); // 执行滤波，结果存储在 cloud_filtered

            pcl::io::savePCDFile("cloud_filtered_y.pcd", *cloud_filtered);
        }

        if (leaf_size_x) {
            //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
            // 创建 passthrough 滤波器对象
            pcl::PassThrough<pcl::PointXYZ> pass;
            pass.setInputCloud(cloud); // 设置输入点云
            pass.setFilterFieldName("x"); // 设置滤波字段
            pass.setFilterLimits(leaf_size_x, max_pt.x); // 设置滤波范围
            pass.filter(*cloud_filtered); // 执行滤波，结果存储在 cloud_filtered
            pcl::io::savePCDFile("cloud_filtered_x.pcd", *cloud_filtered);
        }

        qDebug() << "Filtering complete! Original points: " << cloud->size()
            << ", Filtered points:" << cloud_filtered->size();

        log("Filtering complete! Original points: " + QString::number(cloud->size()));
        log("Filtering complete! Filtered points:" + QString::number(cloud_filtered->size()));


        // 清除之前添加的点云（如果有的话）
        viewer->removeAllPointClouds();

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorHandler(cloud_filtered, 0, 215, 0); // 设置点云颜色为白色

        // 添加新的点云
        viewer->addPointCloud(cloud_filtered, colorHandler, "sample cloud");

        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud"); // 设置点云大小为1

        ui.openGLWidget->renderWindow()->Render();

        ui.openGLWidget->update();
    }
    else
    {
        log("Error: Selected filter algorithm not yet implemented!");
        //
        // qDebug() << "Selected feature not yet implemented:" << current_algo;
        // QMessageBox::information(this, "提示", "该滤波功能尚未实现！");
    }
}


void internship::log(const QString& message)
{
    // 1. 准备要显示在UI上的完整日志信息
    QString log_message = QDateTime::currentDateTime().toString("[yyyy-MM-dd hh:mm:ss] ") + message;

    // 2. 添加到 QListWidget 中
    ui.listWidget_clouds->addItem(log_message);

    // 3. (可选但推荐) 自动滚动到底部，确保最新的日志总是可见的
    ui.listWidget_clouds->scrollToBottom();

    // 4. (可选但推荐) 仍然使用 qDebug() 打印到VS的输出窗口，方便调试
    qDebug() << message;
}


void internship::on_btn_run_segment_clicked() // 这是你UI里 "Run Segment" 按钮的槽函数
{
    // --- 1. 安全检查 ---
    if (!cloud || cloud->empty())
    {
        log("Error: Point cloud is empty, cannot perform segmentation!");
        // QMessageBox::warning(this, "错误", "请先加载一个点云文件！");
        return;
    }

    // --- 2. 根据正确的 ComboBox (combo_segment) 来决定算法 ---
    QString current_algo = ui.combo_segment->currentText(); // *** 修正 #1: 使用分割算法的下拉框 ***

    if (current_algo == "sac_segmentation") // 假设你在下拉框里添加了这一项
    {
        // --- 3. 从正确的 stackedWidget (stackedWidget_segment) 中获取参数 ---

        // 获取当前 stackedWidget 的页面
        QWidget* current_page = ui.stackedWidget_segment->widget(0);
        // *** 修正 #2: 使用分割的 stackedWidget，并获取当前页 ***

        if (!current_page)
        {
            log("Error: Could not find the current parameter page in UI!");
            return;
        }

        // 使用 objectName 精准查找控件
        auto* mtypeCombo = current_page->findChild<QComboBox*>("comboBox_2"); // *** 修正 #3: 使用objectName ***
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

        // --- 4. 执行 PCL 核心分割代码 ---

        // 创建分割对象
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        
        // A. 设置分割参数
        seg.setOptimizeCoefficients(true);
        seg.setMaxIterations(1000);
        // 将QString转换为PCL枚举类型
        if (model_type_str == "Plane")
        {
            seg.setModelType(pcl::SACMODEL_PLANE);
        }
        else { /* ...可以为其他模型添加else if... */ 
            seg.setModelType(pcl::SACMODEL_CIRCLE3D);
        }

        if (method_type_str == "RANSAC")
        {
            seg.setMethodType(pcl::SAC_RANSAC);
        }
        else { /* ...可以为其他方法添加else if... */ }

        seg.setDistanceThreshold(distance_threshold);
        seg.setInputCloud(cloud);

        // B. 执行分割
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.size() == 0)
        {
            log("Error: Could not estimate a planar model for the given dataset.");
            // QMessageBox::warning(this, "错误", "无法在点云中拟合出平面！");
            return;
        }

        log("Segmentation complete! Found " + QString::number(inliers->indices.size()) + " inlier points.");
        // 你也可以把模型系数打印出来
        // std::cerr << "Model coefficients: " << *coefficients << std::endl;

        // --- 5. 根据索引提取内点和外点，并可视化 ---

        //// 创建一个点云对象来存储内点 (平面上的点)
        //pcl::ExtractIndices<pcl::PointXYZ> extract;
        //pcl::PointCloud<pcl::PointXYZ>::Ptr SACcloud_xyz_filtered(new pcl::PointCloud<pcl::PointXYZ>());
        //// 设置输入点云（你用来做分割的那个点云）
        //extract.setInputCloud(cloud);

        //// 设置要提取的索引（分割算法得到的内点）
        //extract.setIndices(inliers);
        //extract.setNegative(true); // 取反运算！
        //extract.filter(*SACcloud_xyz_filtered);


        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inliers(new pcl::PointCloud<pcl::PointXYZ>);
        // 创建另一个点云对象来存储外点 (平面外的点)
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_outliers(new pcl::PointCloud<pcl::PointXYZ>);

        // 创建索引提取器
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);

        // A. 提取内点
        extract.setNegative(false); // false表示提取索引对应的点
        extract.filter(*cloud_inliers);

        // 从模型系数中提取圆的参数
        //float center_x = coefficients->values[0];
        //float center_z = coefficients->values[1];
        //float radius = coefficients->values[2];
        //float radius_sq = radius * radius; // 预先计算半径的平方以提高效率
        //log("dasdas" + QString::number(center_x) +"cssd" + QString::number(center_z)+"dsdsd" + QString::number(radius));
        //pcl::PointIndices::Ptr cylinder_inliers(new pcl::PointIndices);
        //cylinder_inliers->indices.reserve(cloud->size());

        //// 遍历整个原始3D点云
        //for (size_t i = 0; i < cloud->size(); ++i)
        //{
        //    const auto& point = cloud->points[i];

        //    // **关键：在XZ平面上计算到圆心的距离**
        //    float dist_in_xz_plane_sq = std::pow(point.z - center_z, 2) + std::pow(point.x - center_x, 2);

        //    // 如果这个2D距离小于半径，那么这个点就在Y轴方向的无限圆柱体内
        //    if (dist_in_xz_plane_sq < std::pow(radius, 2))
        //    {
        //        cylinder_inliers->indices.push_back(i);
        //    }
        //}

        //// 之后用 ExtractIndices 移除这些点...
        //pcl::ExtractIndices<pcl::PointXYZ> extractx;
        //extractx.setInputCloud(cloud);
        //extractx.setIndices(cylinder_inliers);
        //extractx.setNegative(false);
        //extractx.filter(*cloud_inliers);




        // B. 提取外点
        extract.setNegative(true);  // true表示提取索引之外的点
        extract.filter(*cloud_outliers);

        pcl::io::savePCDFile("SACcloud_xyz_filtered.pcd", *cloud_outliers);
        // --- 6. 在PCL Viewer中更新显示 ---

        // 首先移除所有之前的点云，确保视图干净
        viewer->removeAllPointClouds();

        // 可视化内点 (平面)，用绿色显示
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> inlier_color(cloud_inliers, 0, 255, 0); // Green
        viewer->addPointCloud<pcl::PointXYZ>(cloud_inliers, inlier_color, "cloud_inliers sample cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "outlier_color sample cloud");

        // 可视化外点 (其余的点)，用白色显示
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> outlier_color(cloud_outliers, 255, 255, 255); // White
        viewer->addPointCloud<pcl::PointXYZ>(cloud_outliers, outlier_color, "outlier_color sample cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "outlier_color sample cloud");

        // 刷新渲染窗口
        ui.openGLWidget->renderWindow()->Render();
        ui.openGLWidget->update();
    }
    else if (current_algo == "region_growing") // 假设你在下拉框里添加了这一项
    {
        QWidget* current_page = ui.stackedWidget_segment->widget(2);
        // *** 修正 #2: 使用分割的 stackedWidget，并获取当前页 ***

        if (!current_page)
        {
            log("Error: Could not find the current parameter page in UI!");
            return;
        }

        // 使用 objectName 精准查找控件
        //auto* mtypeCombo = current_page->findChild<QComboBox*>("comboBox_2"); // *** 修正 #3: 使用objectName ***
        auto* Misize = current_page->findChild<QDoubleSpinBox*>("doubleSpinBox_3"); // *** 修正 #3: 使用objectName ***
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

        // --- 步骤 2: 检查并排序簇 ---
        if (cluster_indices.empty())
        {
            std::cout << "No clusters were found." << std::endl;
            return ;
        }

        std::cout << "Found " << cluster_indices.size() << " clusters before sorting." << std::endl;

        // **【核心】使用 std::sort 和 Lambda 表达式对簇按大小进行降序排序**
        std::sort(cluster_indices.begin(), cluster_indices.end(),
            [](const pcl::PointIndices& a, const pcl::PointIndices& b) {
                return a.indices.size() > b.indices.size();
            }
        );

        std::cout << "Clusters have been sorted by size in descending order." << std::endl;

        // --- 步骤 3: 提取最大的簇 ---

        // 排序后，第一个元素就是最大的簇
        const pcl::PointIndices& largest_cluster = cluster_indices[0];

        // 创建一个新的点云对象，用来存储最大簇的点
        pcl::PointCloud<pcl::PointXYZ>::Ptr largest_cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        // 遍历最大簇的所有点的索引
        for (const auto& idx : largest_cluster.indices) {
            // 根据索引，从原始点云中把点添加到新的点云中
            largest_cluster_cloud->points.push_back(cloud->points[idx]);
        }

        largest_cluster_cloud->width = largest_cluster_cloud->points.size();
        largest_cluster_cloud->height = 1;
        largest_cluster_cloud->is_dense = true;

        // --- 步骤 4: 使用或验证结果 ---
        std::cout << "The largest cluster contains " << largest_cluster_cloud->points.size() << " points." << std::endl;

        // 现在，`largest_cluster_cloud` 就是你想要的那个点数最多的簇的点云
        // 你可以对它进行可视化、保存到文件或进行下一步处理
         pcl::io::savePCDFileASCII("ORIXYZ.pcd", *largest_cluster_cloud);



        //if (cluster_indices.empty())
        //{
        //    log("Error: Could not estimate any clusters for the given dataset.");
        //    // QMessageBox::warning(this, "错误", "无法在点云中找到聚类！");
        //    return;
        //}

        //log("Segmentation complete! Found " + QString::number(cluster_indices.size()) + " clusters.");

        //// --- 5. 根据索引提取内点和外点，并可视化 ---

        //// 创建一个点云对象来存储内点 (平面上的点)
        //pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();

        //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inliers(new pcl::PointCloud<pcl::PointXYZ>);
        //// 创建另一个点云对象来存储外点 (平面外的点)
        //// 3. 使用 copyPointCloud 进行转换
        //if (colored_cloud && !colored_cloud->empty())
        //{
        //    pcl::copyPointCloud(*colored_cloud, *cloud_inliers);
        //}

        //pcl::io::savePCDFile("ORIXYZ.pcd", *colored_cloud);

        viewer->removeAllPointClouds();
        // 可视化内点 (平面)，用绿色显示
        //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> inlier_color(cloud_inliers, 0, 255, 0); // Green
        viewer->addPointCloud<pcl::PointXYZRGB>(colored_cloud, "cloud_inliers sample cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_inliers sample cloud");


        viewer->addPointCloud<pcl::PointXYZ>(largest_cluster_cloud, "largest_cluster_cloud sample cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "largest_cluster_cloud sample cloud");


        // 刷新渲染窗口
        ui.openGLWidget->renderWindow()->Render();
        ui.openGLWidget->update();
    }
    else if (current_algo == "extract_clusters") {
        QWidget* current_page = ui.stackedWidget_segment->widget(1);
        // *** 修正 #2: 使用分割的 stackedWidget，并获取当前页 ***

        if (!current_page)
        {
            log("Error: Could not find the current parameter page in UI!");
            return;
        }

        // 使用 objectName 精准查找控件
        //auto* mtypeCombo = current_page->findChild<QComboBox*>("comboBox_2"); // *** 修正 #3: 使用objectName ***
        auto* Misize = current_page->findChild<QDoubleSpinBox*>("doubleSpinBox_8"); // *** 修正 #3: 使用objectName ***
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
        


        // --- 步骤 2: 检查并排序簇 ---
        if (clusters.empty())
        {
            log("No clusters were found.");
            return;
        }

        log("Found :" + QString::number(clusters.size()) + " clusters.");
        //std::cout << "Found " << clusters.size() << " clusters before sorting." << std::endl;

        // **【核心】使用 std::sort 和 Lambda 表达式对簇按大小进行降序排序**
        std::sort(clusters.begin(), clusters.end(),
            [](const pcl::PointIndices& a, const pcl::PointIndices& b) {
                return a.indices.size() > b.indices.size();
            }
        );

        log("Clusters have been sorted by size in descending order.");
        //std::cout << "Clusters have been sorted by size in descending order." << std::endl;

        // --- 步骤 3: 提取最大的簇 ---

        // 排序后，第一个元素就是最大的簇
        const pcl::PointIndices& largest_cluster = clusters[0];

        // 创建一个新的点云对象，用来存储最大簇的点
        pcl::PointCloud<pcl::PointXYZ>::Ptr largest_cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        // --- 开始提取操作 ---

// 1. 创建 ExtractIndices 滤波器对象
        pcl::ExtractIndices<pcl::PointXYZ> extract;

        // 2. 设置输入点云
        extract.setInputCloud(cloud);

        // 3. 设置要“排除”的点的索引，即最大簇的索引
        // 注意：setIndices 接收的是智能指针，所以我们需要传递 largest_cluster 的地址
        // 或者创建一个智能指针来包装它。更简单的做法是创建一个 PointIndices::Ptr。
        pcl::PointIndices::Ptr largest_cluster_indices_ptr(new pcl::PointIndices(largest_cluster));
        extract.setIndices(largest_cluster_indices_ptr);

        // 4. *** 关键步骤：设置为反向提取 ***
        extract.setNegative(true);

        // 5. 执行滤波，将结果存储到你的点云中
        extract.filter(*largest_cluster_cloud);

        pcl::io::savePCDFileASCII("end.pcd", *largest_cluster_cloud);

        // 遍历最大簇的所有点的索引
        for (const auto& idx : largest_cluster.indices) {
            // 根据索引，从原始点云中把点添加到新的点云中
            largest_cluster_cloud->points.push_back(cloud->points[idx]);
        }

        largest_cluster_cloud->width = largest_cluster_cloud->points.size();
        largest_cluster_cloud->height = 1;
        largest_cluster_cloud->is_dense = true;



        // --- 步骤 4: 使用或验证结果 ---
        log("The largest cluster contains :" + QString::number(largest_cluster_cloud->points.size()) + " points.");
        //std::cout << "The largest cluster contains " << largest_cluster_cloud->points.size() << " points." << std::endl;

        // 现在，`largest_cluster_cloud` 就是你想要的那个点数最多的簇的点云
        // 你可以对它进行可视化、保存到文件或进行下一步处理
        //pcl::io::savePCDFileASCII("ORIXYZ.pcd", *largest_cluster_cloud);

        viewer->removeAllPointClouds();
        // 可视化内点 (平面)，用绿色显示
        //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> inlier_color(cloud_inliers, 0, 255, 0); // Green
        //viewer->addPointCloud<pcl::PointXYZRGB>(largest_cluster_cloud, "cloud_inliers sample cloud");
        //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_inliers sample cloud");

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> inlier_color(largest_cluster_cloud, 255, 240, 0); // Green
        viewer->addPointCloud<pcl::PointXYZ>(largest_cluster_cloud, inlier_color, "largest_cluster_cloud sample cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "largest_cluster_cloud sample cloud");


        // 刷新渲染窗口
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

    // --- 1. 定义路径 ---

    // **关键：指定你的虚拟环境中的python.exe的完整路径**
    // 注意：在C++字符串中，反斜杠 \ 需要转义成 \\
    QString python_executable = "C:\\Path\\To\\Your\\venv\\Scripts\\python.exe"; 
    // 或者 conda: "C:\\Users\\YourName\\anaconda3\\envs\\pointnext_env\\python.exe"

    // 定义你的Python脚本的路径
    QString script_path = "C:\\PCLandC\\Conv_models\\models\\cws.py";
    QString python_executable = "C:\\Miniconda\\python.exe";
    // (可选) 定义输入输出文件的路径
    QString input_pcd_path = "C:/temp/input_for_python.pcd";
    QString output_pcd_path = "C:/temp/output_from_python.pcd";

    // --- 2. 准备要传递给Python脚本的命令行参数 ---
    QStringList args;
    args << script_path; // 第一个参数总是脚本本身
    args << "--input" << input_pcd_path;
    args << "--output" << output_pcd_path;
    // ... 添加你脚本需要的其他任何参数 ...

    // --- 3. 创建并启动 QProcess ---
    QProcess* pythonProcess = new QProcess(this);

    // (可选) 连接信号，以便在Python脚本结束后得到通知
    connect(pythonProcess, &QProcess::finished, this, [=](int exitCode, QProcess::ExitStatus exitStatus) {
        qDebug() << "Python script finished with exit code:" << exitCode;
        if (exitStatus == QProcess::NormalExit && exitCode == 0) {
            // 脚本成功运行
            qDebug() << "Successfully processed point cloud. Loading result from:" << output_pcd_path;
            // 在这里写加载 output_pcd_path 并显示的代码
        }
        else {
            // 脚本运行出错
            qDebug() << "Python script failed!";
            // 读取并显示错误信息
            QByteArray error_output = pythonProcess->readAllStandardError();
            qDebug() << "Error output:" << QString(error_output);
        }
        pythonProcess->deleteLater(); // 清理QProcess对象
        });

    // (可选) 读取标准输出
    connect(pythonProcess, &QProcess::readyReadStandardOutput, this, [=]() {
        QByteArray output = pythonProcess->readAllStandardOutput();
        qDebug() << "Python stdout:" << QString(output);
        });

    // 启动进程！
    pythonProcess->start(python_executable, args);

    // 检查是否成功启动
    if (!pythonProcess->waitForStarted()) {
        qDebug() << "Failed to start Python process!";
        qDebug() << "Error:" << pythonProcess->errorString();
        delete pythonProcess;
        log("Average distance between points222: ");
    }
    log("Average distance between points:333 ");
}
