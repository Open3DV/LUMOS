#pragma once

#include <QDialog>
#include "ui_outlier_removal_gui.h"
#include "../firmware/system_config_settings.h"

class OutlierRemovalSettingGui : public QDialog
{
	Q_OBJECT

public:
	OutlierRemovalSettingGui(QWidget *parent = nullptr);
	~OutlierRemovalSettingGui();

	void setConfigParam(struct FirmwareConfigParam config_param);

	void getConfigParam(struct FirmwareConfigParam& config_param);
		
public slots: 
	void do_pushButton_ok();

	void do_pushButton_cancel();
private:
	Ui::OutlierRemovalSettingGuiClass ui;


	struct FirmwareConfigParam firmware_config_param_;
};
