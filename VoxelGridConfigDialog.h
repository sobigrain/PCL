#pragma once

#include <QDialog>

// 注意这里包含了 uic 生成的头文件
#include "ui_VoxelGridConfigDialog.h"

class VoxelGridConfigDialog : public QDialog
{
    Q_OBJECT

public:
    VoxelGridConfigDialog(QWidget* parent = nullptr);
    ~VoxelGridConfigDialog();

    // 我们之前设计的获取参数的函数
    double getLeafSize() const;
    void setLeafSize(double size); // 添加一个设置初始值的函数，更完善

private:
    // 这个指针将指向由 ui_VoxelGridConfigDialog.h 定义的UI类
    Ui::VoxelGridConfigDialog* ui;
};