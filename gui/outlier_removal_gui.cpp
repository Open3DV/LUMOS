#include "outlier_removal_gui.h"

OutlierRemovalSettingGui::OutlierRemovalSettingGui(QWidget *parent)
	: QDialog(parent)
{
	ui.setupUi(this);


	connect(ui.pushButton_ok, SIGNAL(clicked()), this, SLOT(do_pushButton_ok()));
	connect(ui.pushButton_cancel, SIGNAL(clicked()), this, SLOT(do_pushButton_cancel()));

	ui.checkBox_reflect->hide();
}

OutlierRemovalSettingGui::~OutlierRemovalSettingGui()
{
}

void OutlierRemovalSettingGui::do_pushButton_ok()
{
	if (ui.checkBox_radius_filter->isChecked())
	{
		firmware_config_param_.use_radius_filter = 1;
	}
	else
	{
		firmware_config_param_.use_radius_filter = 0;
	}
	
	firmware_config_param_.radius_filter_r = ui.doubleSpinBox_radius->value();
	firmware_config_param_.radius_filter_threshold_num = ui.spinBox_num->value();


	if (ui.checkBox_reflect->isChecked())
	{
		firmware_config_param_.use_reflect_filter = 1;
	}
	else
	{
		firmware_config_param_.use_reflect_filter = 0;
	}
	

	this->accept();
}

void OutlierRemovalSettingGui::do_pushButton_cancel()
{

	this->reject();
}

void OutlierRemovalSettingGui::getConfigParam(struct FirmwareConfigParam& config_param)
{
	config_param = firmware_config_param_;
}

void OutlierRemovalSettingGui::setConfigParam(struct FirmwareConfigParam config_param)
{
	firmware_config_param_ = config_param;

	if (1 == firmware_config_param_.use_radius_filter)
	{
		ui.checkBox_radius_filter->setChecked(true);
	}
	else
	{
		ui.checkBox_radius_filter->setChecked(false); 
	}

	ui.doubleSpinBox_radius->setValue(firmware_config_param_.radius_filter_r);
	ui.spinBox_num->setValue(firmware_config_param_.radius_filter_threshold_num);


	if (1 == firmware_config_param_.use_reflect_filter)
	{
		ui.checkBox_reflect->setChecked(true);
	}
	else
	{
		ui.checkBox_reflect->setChecked(false);
	}
}
