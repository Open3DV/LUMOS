#pragma once

#include <QDialog>
#include "ui_update_firmware_gui.h"

class UpdateFirmwareGui : public QDialog
{
	Q_OBJECT

public:
	UpdateFirmwareGui(QWidget* parent = Q_NULLPTR);
	~UpdateFirmwareGui();

	void setCameraIp(QString ip);

private slots:
	void do_pushButton_select();
	void do_pushButton_update();
	void print_log(QString str);

private:
	Ui::UpdateFirmwareGui ui;

	QString fileName;
	QString camera_ip;
};
