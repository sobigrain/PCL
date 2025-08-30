点云处理工具 (Point Cloud Processing Tool)
<img width="1920" height="1032" alt="image" src="https://github.com/user-attachments/assets/bf9c56f0-4d30-46c7-ab6a-d34047a5109b" />

简介
本项目是一个基于 PCL (Point Cloud Library)、Qt6 和 C++ 开发的简单点云处理工业软件。它旨在为点云数据提供直观的可视化、基础的滤波操作以及初步的分割功能，以满足常见的工业应用需求。本软件致力于提供一个用户友好的界面，方便工程师和研究人员进行点云数据的预处理和分析。
主要功能
点云可视化：
支持加载多种格式的点云文件 (例如 .pcd, .ply)。
提供灵活的交互式三维视图，包括旋转、平移、缩放等操作。
可显示点云的颜色、法线等属性。
点云滤波：
    直通滤波 (Pass-Through Filter)：根据指定轴向的坐标范围对点云进行裁剪。
    体素栅格滤波 (Voxel Grid Filter)：对点云进行下采样，减少点数量同时保持点云结构。
    统计滤波 (Statistical Outlier Removal)：移除点云中的离群点或噪声点。
    （可根据需要添加更多滤波方法，如半径滤波、条件滤波等）
点云分割：
    平面分割 (Plane Segmentation)：基于 RANSAC 算法识别并提取点云中的主要平面。
    欧式聚类分割 (Euclidean Cluster Extraction)：根据点的空间邻近性将点云分割成不同的独立簇。
    （可根据需要添加更多分割方法，如区域生长、基于法线的分割等）
结果保存：
    支持将处理后的点云数据保存为 .pcd 或其他常用格式。
技术栈
    核心库：Point Cloud Library (PCL) - 1.12.1 或更高版本
    图形用户界面：Qt 6.x
    编程语言：C++
    构建系统：CMake
编译与运行
1. 环境准备
    C++ 编译器：支持 C++17 或更高标准的编译器 (如 MSVC, GCC, Clang)。
    Qt 6：请从 Qt 官网 下载并安装 Qt 6.x 开发工具。
    PCL：
      推荐从 PCL 官网 下载预编译库（自带的VTK为阉割版，要联合QT必须自己重新编译）。
      或者自行编译 PCL 库及其依赖项 (如 Eigen, FLANN, VTK, Boost)。
      注意：确保 PCL 的版本与你的 Visual Studio/Qt 配置兼容。
3. 克隆项目
code
Bash
git clone https://github.com/sobigrain/PCL.git
cd PCL
4. 使用 CMake 构建 (Visual Studio 为例)
安装 CMake：确保你的系统安装了 CMake。
创建构建目录：
code
Bash
mkdir build
cd build
运行 CMake 配置：
code
Bash
cmake .. -DCMAKE_PREFIX_PATH="<你的Qt安装路径>/<版本号>/msvc2019_64/lib/cmake"
请根据你的实际 Qt 和 PCL 安装路径修改 CMAKE_PREFIX_PATH 和 PCL_DIR。
生成解决方案：CMake 将生成一个 Visual Studio 解决方案文件 (.sln)。
在 Visual Studio 中打开：用 Visual Studio 2022 打开生成的 .sln 文件。
构建项目：选择 "构建" -> "全部重新生成" 来编译项目。
4. 运行程序
编译成功后，你可以在 Visual Studio 中直接运行 PCLProcessingTool.exe (通常在 build/Debug 或 build/Release 目录下)。
界面概览
菜单栏：提供文件操作（打开、保存）、工具选择（滤波、分割）等功能。
工具栏：常用功能的快捷按钮。
点云视图区：显示加载的点云数据。用户可以在此区域进行交互式操作。
参数设置面板：根据当前选择的工具（滤波或分割），动态显示相应的参数设置选项。
状态栏：显示当前操作状态、点云信息等。
使用示例
加载点云：点击 "文件" -> "打开点云"，选择一个 .pcd 或 .ply 文件。
应用滤波：
选择 "工具" -> "滤波" -> "体素栅格滤波"。
在参数面板中设置体素大小，然后点击 "应用"。
进行分割：
选择 "工具" -> "分割" -> "平面分割"。
在参数面板中设置距离阈值，然后点击 "应用"。
保存结果：点击 "文件" -> "保存点云"，将处理后的点云保存到新文件。
贡献
欢迎对本项目进行贡献！如果你有任何改进建议或发现 Bug，请随时提交 Pull Request 或 Issue。
许可证
本项目采用 MIT 许可证。详见 LICENSE 文件。
联系方式
作者：sobigrain
GitHub：sobigrain/PCL
