#include "VoxelGridConfigDialog.h"

VoxelGridConfigDialog::VoxelGridConfigDialog(QWidget* parent) :
    QDialog(parent),
    ui(new Ui::VoxelGridConfigDialog) // ����UI����
{
    ui->setupUi(this); // ��ʼ��UI
}

VoxelGridConfigDialog::~VoxelGridConfigDialog()
{
    delete ui;
}

double VoxelGridConfigDialog::getLeafSize() const
{
    // ȷ������.ui�ļ��и�spinbox���õ�objectName��spinBox_leafSize
    return ui->spinBox_leafSize->value();
}

void VoxelGridConfigDialog::setLeafSize(double size)
{
    ui->spinBox_leafSize->setValue(size);
}