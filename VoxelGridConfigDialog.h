#pragma once

#include <QDialog>

// ע����������� uic ���ɵ�ͷ�ļ�
#include "ui_VoxelGridConfigDialog.h"

class VoxelGridConfigDialog : public QDialog
{
    Q_OBJECT

public:
    VoxelGridConfigDialog(QWidget* parent = nullptr);
    ~VoxelGridConfigDialog();

    // ����֮ǰ��ƵĻ�ȡ�����ĺ���
    double getLeafSize() const;
    void setLeafSize(double size); // ���һ�����ó�ʼֵ�ĺ�����������

private:
    // ���ָ�뽫ָ���� ui_VoxelGridConfigDialog.h �����UI��
    Ui::VoxelGridConfigDialog* ui;
};