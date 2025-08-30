#include "VoxelGridConfigDialog.h"

VoxelGridConfigDialog::VoxelGridConfigDialog(QWidget* parent) :
    QDialog(parent),
    ui(new Ui::VoxelGridConfigDialog) // 创建UI对象
{
    ui->setupUi(this); // 初始化UI
}

VoxelGridConfigDialog::~VoxelGridConfigDialog()
{
    delete ui;
}

double VoxelGridConfigDialog::getLeafSize() const
{
    // 确保你在.ui文件中给spinbox设置的objectName是spinBox_leafSize
    return ui->spinBox_leafSize->value();
}

void VoxelGridConfigDialog::setLeafSize(double size)
{
    ui->spinBox_leafSize->setValue(size);
}