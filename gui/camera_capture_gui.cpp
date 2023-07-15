#include "camera_capture_gui.h"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "file_io_function.h"
#include <QDebug>
#include <iostream>
#include <QMouseEvent>
#include <QtWidgets/qfiledialog.h>
#include <qheaderview.h>
#include "PrecisionTest.h"
#include <qdesktopservices.h>
#include <thread> 
#include <QFuture>
#include <QtConcurrent/qtconcurrentrun.h>
#include "waiting_gui.h"
#include <enumerate.h>
#include "about_gui.h"
#include "save_gui.h"

CameraCaptureGui::CameraCaptureGui(QWidget* parent)
	: QWidget(parent)
{
	ui.setupUi(this);

	connected_flag_ = false;

	ui.tableWidget_more_exposure->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
	ui.tableWidget_more_exposure->horizontalHeader()->setSectionResizeMode(2, QHeaderView::Stretch);
	ui.tableWidget_more_exposure->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
	ui.tableWidget_more_exposure->horizontalHeader()->setSortIndicatorShown(false);
	//ui.tableWidget_more_exposure->horizontalHeader()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
	ui.tableWidget_more_exposure->setEditTriggers(QAbstractItemView::NoEditTriggers); //设置不可编辑
	ui.tableWidget_more_exposure->setFrameShape(QFrame::Box);
	 


	config_system_param_machine_.loadProcessingSettingsFile("../camera_config.json");
	config_system_param_machine_.getSystemConfigData(system_config_param_);
	config_system_param_machine_.getFirmwareConfigData(firmware_config_param_);
	config_system_param_machine_.getGuiConfigData(processing_gui_settings_data_);





	initializeFunction();
	setUiData();
	undateSystemConfigUiData();
	//修复默认值不触发hdr表更新
	do_spin_exposure_num_changed(firmware_config_param_.mixed_exposure_num);

	last_path_ = processing_gui_settings_data_.last_path;
	sys_path_ = processing_gui_settings_data_.last_path;
	QDir dir(last_path_);
	QString path = dir.absolutePath();

	if (!dir.exists(path))
	{
		bool res = dir.mkpath(path);
	}

  
	radio_button_flag_ = SELECT_BRIGHTNESS_FLAG_;
	showImage();

	ui.comboBox_ip->hide();
	ui.pushButton_refresh->hide();

	m_pMaskLayer = new WaitingGui(this);
	m_pMaskLayer->setFixedSize(this->size());//设置窗口大小
	m_pMaskLayer->setVisible(false);//初始状态下隐藏，待需要显示时使用
	this->stackUnder(m_pMaskLayer);//其中pWrapper为当前窗口的QWidget
 
	
}

CameraCaptureGui::~CameraCaptureGui()
{
}



void CameraCaptureGui::setOnDrop(int (*p_function)(void*))
{
	m_p_OnDropped_ = p_function;
}


void CameraCaptureGui::resizeEvent(QResizeEvent* event)
{
	//if (event) {}//消除警告提示

	if (m_pMaskLayer != nullptr)
	{
		m_pMaskLayer->setAutoFillBackground(true); //这个很重要，否则不会显示遮罩层
		QPalette pal = m_pMaskLayer->palette();
		pal.setColor(QPalette::Background, QColor(0x00, 0x00, 0x00, 0x20));
		m_pMaskLayer->setPalette(pal);
		m_pMaskLayer->setFixedSize(this->size());
	}
}
 
//显示
void CameraCaptureGui::showLoadingForm()
{
	if (m_pMaskLayer != nullptr)
	{
		m_pMaskLayer->setVisible(true);
	}
}
//隐藏
void CameraCaptureGui::hideLoadingForm()
{
	if (m_pMaskLayer != nullptr)
	{
		m_pMaskLayer->setVisible(false);
	}
}


void CameraCaptureGui::getCameraIp(QString& ip)
{
	ip = ui.lineEdit_ip->text();
}

void CameraCaptureGui::getFirmwareVersion(QString& version)
{
	version = QString(firmware_version_);
}

void CameraCaptureGui::getProductInfo(QString& info)
{
	QTextCodec* codec = QTextCodec::codecForName("gbk");
	QTextCodec::setCodecForLocale(codec);

	info = QString::fromLocal8Bit(info_);
}

void CameraCaptureGui::setCalibrationBoard(int flag)
{

	switch (flag)
	{
	case 4:
	{
		board_message_.rows = 11;
		board_message_.cols = 7;
		board_message_.width = 4;
		board_message_.height = 2;
		    
		calibration_board_flag_ = flag;

		processing_gui_settings_data_.Instance().calibration_board = flag;
	}
	break;

	case 12:
	{
		board_message_.rows = 11;
		board_message_.cols = 7;
		board_message_.width = 12;
		board_message_.height = 6;
		 
		calibration_board_flag_ = flag;

		processing_gui_settings_data_.Instance().calibration_board = flag;
	}
	break;

	case 20:
	{
		board_message_.rows = 11;
		board_message_.cols = 7;
		board_message_.width = 20;
		board_message_.height = 10;
		 
		calibration_board_flag_ = flag;

		processing_gui_settings_data_.Instance().calibration_board = flag;
	}
	break;

	case 40:
	{
		board_message_.rows = 11;
		board_message_.cols = 7;
		board_message_.width = 40;
		board_message_.height = 20;
		 
		calibration_board_flag_ = flag;
		processing_gui_settings_data_.Instance().calibration_board = flag;
	}
	break;

	case 80:
	{
		board_message_.rows = 13;
		board_message_.cols = 9;
		board_message_.width = 80;
		board_message_.height = 40;
		 
		calibration_board_flag_ = flag;
		processing_gui_settings_data_.Instance().calibration_board = flag;
	}
	break;
	default:
		break;
	}

 
}


bool CameraCaptureGui::initializeFunction()
{
	/********************************************************************************************************************/


	connect(this, SIGNAL(send_log_update(QString)), this, SLOT(do_handleLogMessage(QString)));
	/*******************************************************************************************************************/

	connect(ui.spinBox_exposure_num, SIGNAL(valueChanged(int)), this, SLOT(do_spin_exposure_num_changed(int)));
	connect(ui.doubleSpinBox_min_z, SIGNAL(valueChanged(double)), this, SLOT(do_spin_min_z_changed(double)));
	connect(ui.doubleSpinBox_max_z, SIGNAL(valueChanged(double)), this, SLOT(do_spin_max_z_changed(double)));
	connect(ui.doubleSpinBox_confidence, SIGNAL(valueChanged(double)), this, SLOT(do_doubleSpin_confidence(double)));
	connect(ui.doubleSpinBox_depth_filter, SIGNAL(valueChanged(double)), this, SLOT(do_doubleSpin_depth_fisher(double)));
	connect(ui.doubleSpinBox_gain, SIGNAL(valueChanged(double)), this, SLOT(do_doubleSpin_gain(double)));
	connect(ui.doubleSpinBox_gamma, SIGNAL(valueChanged(double)), this, SLOT(do_doubleSpin_gamma(double)));

	connect(ui.spinBox_led, SIGNAL(valueChanged(int)), this, SLOT(do_spin_led_current_changed(int)));
	connect(ui.spinBox_camera_exposure, SIGNAL(valueChanged(int)), this, SLOT(do_spin_camera_exposure_changed(int)));
	connect(ui.spinBox_smoothing, SIGNAL(valueChanged(int)), this, SLOT(do_spin_smoothing_changed(int)));

	connect(ui.radioButton_brightness, SIGNAL(toggled(bool)), this, SLOT(do_QRadioButton_toggled_brightness(bool)));
	connect(ui.radioButton_depth_color, SIGNAL(toggled(bool)), this, SLOT(do_QRadioButton_toggled_color_depth(bool)));
	connect(ui.radioButton_depth_grey, SIGNAL(toggled(bool)), this, SLOT(do_QRadioButton_toggled_gray_depth(bool)));
	connect(ui.radioButton_calibration, SIGNAL(toggled(bool)), this, SLOT(do_QRadioButton_toggled_calibration(bool)));
	 
	connect(ui.comboBox_ip, SIGNAL(activated(int)), this, SLOT(do_comboBox_activated_ip(int))); 
	connect(ui.checkBox_over_exposure, SIGNAL(toggled(bool)), this, SLOT(do_checkBox_toggled_over_exposure(bool)));

	connect(ui.pushButton_connect, SIGNAL(clicked()), this, SLOT(do_pushButton_connect()));
	connect(ui.pushButton_refresh, SIGNAL(clicked()), this, SLOT(do_pushButton_refresh()));
	connect(ui.pushButton_capture_one_frame, SIGNAL(clicked()), this, SLOT(do_pushButton_capture_one_frame()));
	connect(ui.pushButton_capture_continuous, SIGNAL(clicked()), this, SLOT(do_pushButton_capture_continuous()));


	connect(ui.pushButton_save_as, SIGNAL(clicked()), this, SLOT(do_pushButton_save_as()));
	connect(ui.pushButton_open_folder, SIGNAL(clicked()), this, SLOT(do_pushButton_open_folder()));
	connect(ui.pushButton_test_accuracy, SIGNAL(clicked()), this, SLOT(do_pushButton_test_accuracy()));
	connect(ui.pushButton_calibrate_external_param, SIGNAL(clicked()), this, SLOT(do_pushButton_calibrate_external_param()));

	connect(&capture_timer_, SIGNAL(timeout()), this, SLOT(do_timeout_capture_slot()));
	capture_timer_.setInterval(100);
	start_timer_flag_ = false;


	connect(this, SIGNAL(send_images_update()), this, SLOT(do_undate_show_slot()));

	connect(ui.radioButton_generate_brightness_default, SIGNAL(toggled(bool)), this, SLOT(do_QRadioButton_toggled_generate_brightness_default(bool)));
	connect(ui.radioButton_generate_brightness_illuminsation_define, SIGNAL(toggled(bool)), this, SLOT(do_QRadioButton_toggled_generate_brightness_illumination(bool)));
	connect(ui.radioButton_generate_brightness_darkness_define, SIGNAL(toggled(bool)), this, SLOT(do_QRadioButton_toggled_generate_brightness_darkness(bool)));

	connect(ui.spinBox_camera_exposure_define, SIGNAL(valueChanged(int)), this, SLOT(do_spin_generate_brightness_exposure_changed(int)));

	connect(ui.checkBox_depth_filter, SIGNAL(toggled(bool)), this, SLOT(do_checkBox_toggled_depth_filter(bool)));

	min_depth_value_ = 300;
	max_depth_value_ = 3000;

	capture_show_flag_ = false;
	capturing_flag_ = false;
	camera_setting_flag_ = false;

	generate_brightness_model_ = GENERATE_BRIGHTNESS_DEFAULT_;
	generate_brightness_exposure_ = 12000;

	board_message_.rows = 11;
	board_message_.cols = 7;
	board_message_.width = 20;
	board_message_.height = 10;
 

	camera_version_ = 800;


	exposure_time_min_ = 2500;
	exposure_time_max_ = 100000;
	/**********************************************************************************************************************/




	return true;
}


void CameraCaptureGui::do_handleLogMessage(QString str)
{
	QString StrCurrentTime = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss.zzz");

	QString log = StrCurrentTime + " " + str;

	ui.textBrowser_log->append(log);

	ui.textBrowser_log->repaint();
}

void CameraCaptureGui::addLogMessage(QString str)
{
	send_log_update(str);
	//QString StrCurrentTime = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss.zzz");

	//QString log = StrCurrentTime + " " + str;

	//ui.textBrowser_log->append(log);

	//ui.textBrowser_log->repaint();
}


bool CameraCaptureGui::saveOneFrameData(QString path_name)
{
	if (path_name.isEmpty() || brightness_map_.empty() || depth_map_.empty() || height_map_.empty())
	{
		return false;
	}

	QDir dir(path_name);
	path_name = dir.absolutePath();



	if (SaveDataType::Undistort == save_data_type_)
	{
		QString brightness_str = path_name + "_bright.bmp";
		cv::imwrite(brightness_str.toLocal8Bit().toStdString(), undistort_brightness_map_);

		QString depth_str = path_name + "_depth_map.tiff";
		cv::imwrite(depth_str.toLocal8Bit().toStdString(), undistort_depth_map_);

		QString points_str = path_name + "_depth_map.ply";
		FileIoFunction file_io_machine;
		file_io_machine.SaveBinPointsToPly(undistort_pointcloud_map_, points_str, undistort_brightness_map_);
	}
	else
	{
		QString brightness_str = path_name + "_bright.bmp";
		cv::imwrite(brightness_str.toLocal8Bit().toStdString(), brightness_map_);

		QString depth_str = path_name + "_depth_map.tiff";
		cv::imwrite(depth_str.toLocal8Bit().toStdString(), depth_map_);

		QString points_str = path_name + ".ply"; 
		FileIoFunction file_io_machine;
		file_io_machine.SaveBinPointsToPly(pointcloud_map_, points_str, brightness_map_);
	}



	QString height_str = path_name + "_height_map.tiff";
	cv::imwrite(height_str.toLocal8Bit().toStdString(), height_map_);



	return true;
}


bool CameraCaptureGui::loadSettingData(QString path)
{
	bool ret = config_system_param_machine_.loadProcessingSettingsFile(path);
	if (!ret)
	{
		return false;
	}

	QString old_ip = ui.lineEdit_ip->text();

	config_system_param_machine_.getSystemConfigData(system_config_param_);
	config_system_param_machine_.getFirmwareConfigData(firmware_config_param_);
	config_system_param_machine_.getGuiConfigData(processing_gui_settings_data_);

	processing_gui_settings_data_.Instance().ip = old_ip;

	setUiData();
	undateSystemConfigUiData();

	return true;
}

bool CameraCaptureGui::saveSettingData(QString path)
{

	config_system_param_machine_.setSystemConfigData(system_config_param_);
	config_system_param_machine_.setFirmwareConfigData(firmware_config_param_);
	config_system_param_machine_.setGuiConfigData(processing_gui_settings_data_);

	bool ok = config_system_param_machine_.saveProcessingSettingsFile(path);

	return ok;
}


bool CameraCaptureGui::renderBrightnessImage(cv::Mat brightness)
{

	cv::Mat color_map(brightness.size(), CV_8UC3, cv::Scalar(0, 0, 0));

	int nr = color_map.rows;
	int nc = color_map.cols;

	for (int r = 0; r < nr; r++)
	{
		uchar* ptr_b = brightness.ptr<uchar>(r);
		cv::Vec3b* ptr_cb = color_map.ptr<cv::Vec3b>(r);
		for (int c = 0; c < nc; c++)
		{
			if (ptr_b[c] == 255)
			{
				ptr_cb[c][0] = 255;
				ptr_cb[c][1] = 0;
				ptr_cb[c][2] = 0;
			}
			else
			{
				ptr_cb[c][0] = ptr_b[c];
				ptr_cb[c][1] = ptr_b[c];
				ptr_cb[c][2] = ptr_b[c];
			}
		}

	}
	if (ui.checkBox_over_exposure->isChecked())
	{
		render_image_brightness_ = color_map.clone();

	}
	else
	{
		render_image_brightness_ = brightness.clone();

	}
	return true;
}


bool CameraCaptureGui::renderHeightImage(cv::Mat height)
{
	if (height.empty())
	{
		return false;
	}

	float low_z = processing_gui_settings_data_.Instance().low_z_value;
	float high_z = processing_gui_settings_data_.Instance().high_z_value;

	FileIoFunction io_machine;

	io_machine.depthToHeightColor(height, render_image_color_height_, render_image_gray_depth_, low_z, high_z);

	return true;

}

bool CameraCaptureGui::renderDepthImage(cv::Mat depth)
{
	if (depth.empty())
	{
		return false;
	}


	FileIoFunction io_machine;

	int max_depth = io_machine.percentile(depth, 95);
	int min_depth = io_machine.percentile(depth, 5);
	io_machine.depthToDepthColor(depth, render_image_color_depth_, render_image_gray_depth_, min_depth, max_depth);

	return true;

}

bool CameraCaptureGui::setShowImages(cv::Mat brightness, cv::Mat depth)
{
	cv::Mat img_color;
	cv::Mat gray_map;
	FileIoFunction io_machine;

	int low_z = processing_gui_settings_data_.Instance().low_z_value;
	int high_z = processing_gui_settings_data_.Instance().high_z_value;

	io_machine.depthToHeightColor(depth, img_color, gray_map, low_z, high_z);


	return true;
}


void CameraCaptureGui::setGuiSettingData(GuiConfigDataStruct& settings_data_)
{
	processing_gui_settings_data_ = settings_data_;
}



void CameraCaptureGui::undateSystemConfigUiData()
{  
	ui.spinBox_led->setValue(system_config_param_.led_current);

	ui.spinBox_exposure_num->setValue(firmware_config_param_.mixed_exposure_num); 

	ui.spinBox_camera_exposure->setValue(system_config_param_.camera_exposure_time);

	ui.spinBox_camera_exposure_define->setValue(firmware_config_param_.generate_brightness_exposure);

	switch (firmware_config_param_.generate_brightness_model)
	{
	case 1:
	{
		ui.radioButton_generate_brightness_default->setChecked(true);
	}
	break;
	case 2:
	{
		ui.radioButton_generate_brightness_illuminsation_define->setChecked(true);
	}
	break;
	case 3:
	{
		ui.radioButton_generate_brightness_darkness_define->setChecked(true);
	}
	break;
	default:
		break;
	}

	ui.doubleSpinBox_confidence->setValue(firmware_config_param_.confidence);
	ui.doubleSpinBox_gain->setValue(system_config_param_.camera_gain);
	ui.doubleSpinBox_gamma->setValue(system_config_param_.camera_gamma);

	if (0 == firmware_config_param_.use_bilateral_filter)
	{
		ui.spinBox_smoothing->setValue(0);
	}
	else
	{
		ui.spinBox_smoothing->setValue(firmware_config_param_.bilateral_filter_param_d / 2);
	}

	ui.spinBox_camera_exposure->setMaximum(exposure_time_max_);
	ui.spinBox_camera_exposure->setMinimum(exposure_time_min_);
}

void CameraCaptureGui::setUiData()
{
	ui.doubleSpinBox_min_z->setValue(processing_gui_settings_data_.Instance().low_z_value);
	ui.doubleSpinBox_max_z->setValue(processing_gui_settings_data_.Instance().high_z_value);
	ui.lineEdit_ip->setText(processing_gui_settings_data_.Instance().ip);
	 

	setCalibrationBoard(processing_gui_settings_data_.Instance().calibration_board);
	qDebug() << "processing_gui_settings_data_.Instance().calibration_board: " << processing_gui_settings_data_.Instance().calibration_board;
	//ui.spinBox_exposure_num->setDisabled(true);
	//ui.spinBox_led->setDisabled(true);

	switch (processing_gui_settings_data_.Instance().exposure_model)
	{
	case 0:
	{
		ui.radioButton_single_exposure->setChecked(true);
	}
	break;
	case 1:
	{
		ui.radioButton_hdr_exposure->setChecked(true);
	}
	break;
	default:
		break;
	}

	ui.doubleSpinBox_depth_filter->setValue(firmware_config_param_.depth_filter_threshold);
	if (1 == firmware_config_param_.use_depth_filter)
	{

		ui.checkBox_depth_filter->setChecked(true);
	}
	else
	{ 
		ui.checkBox_depth_filter->setChecked(false);
		ui.doubleSpinBox_depth_filter->setDisabled(true);
	}
	
}


double CameraCaptureGui::get_exposure_item_value(int row)
{
	if (row > ui.tableWidget_more_exposure->rowCount() - 1)
		return -1;

	return exposure_time_list_[row]->value();

}


bool CameraCaptureGui::remove_exposure_item(int row)
{

	int item_count = ui.tableWidget_more_exposure->rowCount();

	if (row > item_count - 1)
		return false;

	ui.tableWidget_more_exposure->removeRow(row);

	return true;
}



void CameraCaptureGui::add_exposure_item(int row, int exposure, int led)
{


	int item_count = ui.tableWidget_more_exposure->rowCount();

	if (row >= item_count)
	{
		ui.tableWidget_more_exposure->setRowCount(item_count + 1);
	}

	QSpinBox* exposureSpinBoxItem = new QSpinBox();
	exposureSpinBoxItem->setRange(exposure_time_min_, exposure_time_max_);//设置数值显示范围
	exposureSpinBoxItem->setValue(exposure);
	exposureSpinBoxItem->setButtonSymbols(QAbstractSpinBox::NoButtons);
	exposureSpinBoxItem->setAlignment(Qt::AlignHCenter);
	exposureSpinBoxItem->setKeyboardTracking(false);
	exposureSpinBoxItem->setSingleStep(1000);

	connect(exposureSpinBoxItem, SIGNAL(valueChanged(int)), this, SLOT(do_more_exposure_param_changed(int)));

	QSpinBox* ledSpinBoxItem = new QSpinBox();
	ledSpinBoxItem->setRange(0, 1023);//设置数值显示范围
	ledSpinBoxItem->setValue(led);
	ledSpinBoxItem->setButtonSymbols(QAbstractSpinBox::NoButtons);
	ledSpinBoxItem->setAlignment(Qt::AlignHCenter);
	ledSpinBoxItem->setKeyboardTracking(false);
	ledSpinBoxItem->setSingleStep(100);


	connect(ledSpinBoxItem, SIGNAL(valueChanged(int)), this, SLOT(do_more_exposure_param_changed(int)));

	ui.tableWidget_more_exposure->setItem(row, 0, new QTableWidgetItem(QString::number(row + 1)));
	ui.tableWidget_more_exposure->setCellWidget(row, 1, exposureSpinBoxItem);
	ui.tableWidget_more_exposure->setCellWidget(row, 2, ledSpinBoxItem);

	//ui.tableWidget_more_exposure->setColumnWidth(2 * col,10);
	//ui.tableWidget_more_exposure->item(row, +2 * col)->setTextAlignment(Qt::AlignHCenter | Qt::AlignCenter);
	//ui.tableWidget_more_exposure->item(row, +2 * col)->setSelected(false);

	exposure_time_list_.push_back(exposureSpinBoxItem);
	led_current_list_.push_back(ledSpinBoxItem);

}

void CameraCaptureGui::do_spin_min_z_changed(double val)
{
	processing_gui_settings_data_.Instance().low_z_value = val;

	renderHeightImage(height_map_);
	showImage();


}


void CameraCaptureGui::do_spin_repetition_count_changed(int val)
{
	processing_gui_settings_data_.Instance().repetition_count = val;
}


void CameraCaptureGui::do_spin_camera_exposure_changed(int val)
{

	if (camera_setting_flag_)
	{
		return;
	}


	//设置参数时加锁
	camera_setting_flag_ = true;
	if (connected_flag_)
	{

		system_config_param_.camera_exposure_time = val;

		int ret_code = -1;
		//如果连续采集在用、先暂停
		if (start_timer_flag_)
		{

			stopCapturingOneFrameBaseThread();

			ret_code = DfSetParamCameraExposure(system_config_param_.camera_exposure_time);
			if (0 == ret_code)
			{
				//ui.spinBox_camera_exposure->setValue(system_config_param_.camera_exposure_time);
				for (int i = 0; i < 6; i += 1)
				{
					firmware_config_param_.mixed_exposure_param_list[i] = system_config_param_.camera_exposure_time;
				}

				do_spin_exposure_num_changed(firmware_config_param_.mixed_exposure_num);

				QString str = u8"设置相机曝光时间: " + QString::number(system_config_param_.camera_exposure_time);
				addLogMessage(str);
			}

			do_pushButton_capture_continuous();

		}
		else
		{
			ret_code = DfSetParamCameraExposure(system_config_param_.camera_exposure_time);

			if (0 == ret_code)
			{
				QString str = u8"设置相机曝光时间: " + QString::number(system_config_param_.camera_exposure_time);
				addLogMessage(str);
			}

		}

	}

	camera_setting_flag_ = false;
}


void CameraCaptureGui::do_spin_led_current_changed(int val)
{
	if (camera_setting_flag_)
	{
		return;
	}

	//设置参数时加锁
	camera_setting_flag_ = true;
	if (connected_flag_)
	{

		system_config_param_.led_current = val;

		int ret_code = -1;
		//如果连续采集在用、先暂停
		if (start_timer_flag_)
		{

			stopCapturingOneFrameBaseThread();


			int ret_code = DfSetSystemConfigParam(system_config_param_);
			if (0 == ret_code)
			{
				ui.spinBox_led->setValue(system_config_param_.led_current);
				QString str = u8"设置投影亮度: " + QString::number(system_config_param_.led_current);
				addLogMessage(str);
			}

			do_pushButton_capture_continuous();

		}
		else
		{
			ret_code = DfSetSystemConfigParam(system_config_param_);

			if (0 == ret_code)
			{
				ui.spinBox_led->setValue(system_config_param_.led_current);
				QString str = u8"设置投影亮度: " + QString::number(system_config_param_.led_current);
				addLogMessage(str);
			}

		}

	}

	camera_setting_flag_ = false;
}

void CameraCaptureGui::do_spin_max_z_changed(double val)
{

	processing_gui_settings_data_.Instance().high_z_value = val;


	renderHeightImage(height_map_);
	showImage();


}

void CameraCaptureGui::do_spin_smoothing_changed(int val)
{

	if (camera_setting_flag_)
	{
		return;
	}


	//设置参数时加锁
	camera_setting_flag_ = true;
	if (connected_flag_)
	{
		if (0 == val)
		{
			firmware_config_param_.use_bilateral_filter = 0;
		}
		else
		{
			firmware_config_param_.bilateral_filter_param_d = 2 * val + 1;
		}


		int ret_code = -1;
		//如果连续采集在用、先暂停
		if (start_timer_flag_)
		{

			stopCapturingOneFrameBaseThread();

			ret_code = DfSetParamSmoothing(val);

			if (0 == ret_code)
			{
				QString str = u8"设置平滑参数: " + QString::number(val);
				addLogMessage(str);
			}

			do_pushButton_capture_continuous();

		}
		else
		{
			ret_code = DfSetParamSmoothing(val);

			if (0 == ret_code)
			{
				QString str = u8"设置平滑参数: " + QString::number(val);
				addLogMessage(str);
			}

		}

	}

	camera_setting_flag_ = false;
}

void CameraCaptureGui::do_doubleSpin_gain(double val)
{

	if (camera_setting_flag_)
	{
		return;
	}


	//设置参数时加锁
	camera_setting_flag_ = true;
	if (connected_flag_)
	{

		system_config_param_.camera_gain = val;

		int ret_code = -1;
		//如果连续采集在用、先暂停
		if (start_timer_flag_)
		{

			stopCapturingOneFrameBaseThread();

			ret_code = DfSetParamCameraGain(system_config_param_.camera_gain);
			if (0 == ret_code)
			{
				//ui.spinBox_camera_exposure->setValue(system_config_param_.camera_exposure_time);
				QString str = u8"设置相机增益: " + QString::number(system_config_param_.camera_gain);
				addLogMessage(str);
			}

			do_pushButton_capture_continuous();

		}
		else
		{
			ret_code = DfSetParamCameraGain(system_config_param_.camera_gain);

			if (0 == ret_code)
			{
				//ui.spinBox_camera_exposure->setValue(system_config_param_.camera_exposure_time);
				QString str = u8"设置相机增益: " + QString::number(system_config_param_.camera_gain);
				addLogMessage(str);
			}

		}

	}

	camera_setting_flag_ = false;
}

void CameraCaptureGui::do_doubleSpin_gamma(double val)
{

	if (camera_setting_flag_)
	{
		return;
	}


	//设置参数时加锁
	camera_setting_flag_ = true;
	if (connected_flag_)
	{

		system_config_param_.camera_gamma = val;

		int ret_code = -1;
		//如果连续采集在用、先暂停
		if (start_timer_flag_)
		{

			stopCapturingOneFrameBaseThread();

			ret_code = DfSetParamCameraGamma(system_config_param_.camera_gamma);
			if (0 == ret_code)
			{
				//ui.spinBox_camera_exposure->setValue(system_config_param_.camera_exposure_time);
				QString str = u8"设置相机Gamma: " + QString::number(system_config_param_.camera_gamma);
				addLogMessage(str);
			}

			do_pushButton_capture_continuous();

		}
		else
		{
			ret_code = DfSetParamCameraGamma(system_config_param_.camera_gamma);

			if (0 == ret_code)
			{
				//ui.spinBox_camera_exposure->setValue(system_config_param_.camera_exposure_time);
				QString str = u8"设置相机增益: " + QString::number(system_config_param_.camera_gamma);
				addLogMessage(str);
			}

		}

	}

	camera_setting_flag_ = false;
}

void CameraCaptureGui::updateDepthFilter(int use, float threshold)
{

	if (camera_setting_flag_)
	{
		return;
	}


	//设置参数时加锁
	camera_setting_flag_ = true;
	if (connected_flag_)
	{
		  
		int ret_code = -1;
		//如果连续采集在用、先暂停
		if (start_timer_flag_)
		{

			stopCapturingOneFrameBaseThread();

			ret_code = DfSetParamDepthFilter(use, threshold);
			if (0 == ret_code)
			{
				//ui.spinBox_camera_exposure->setValue(system_config_param_.camera_exposure_time);
				if (1 == use)
				{
					addLogMessage(u8"打开深度图滤波！");
					QString str = u8"阈值: " + QString::number(threshold);
					addLogMessage(str);
				}
				else
				{
					addLogMessage(u8"关闭深度图滤波！");
				}


			}


			do_pushButton_capture_continuous();

		}
		else
		{
			ret_code = DfSetParamDepthFilter(use, threshold);

			if (0 == ret_code)
			{
				if (1 == use)
				{
					addLogMessage(u8"打开深度图滤波！");
					//ui.spinBox_camera_exposure->setValue(system_config_param_.camera_exposure_time);
					QString str = u8"设置噪点过滤: " + QString::number(threshold);
					addLogMessage(str);
				}
				else
				{
					addLogMessage(u8"关闭深度图滤波！");
				}

			}
			else
			{
				addLogMessage(u8"设置深度图滤波失败！");
			}

		}

	}

	camera_setting_flag_ = false;
}

void CameraCaptureGui::do_doubleSpin_depth_fisher(double val)
{
	firmware_config_param_.depth_filter_threshold = val;
	updateDepthFilter(firmware_config_param_.use_depth_filter, firmware_config_param_.depth_filter_threshold);
 
}

void CameraCaptureGui::do_doubleSpin_confidence(double val)
{

	if (camera_setting_flag_)
	{
		return;
	}


	//设置参数时加锁
	camera_setting_flag_ = true;
	if (connected_flag_)
	{

		firmware_config_param_.confidence = (float)val;

		int ret_code = -1;
		//如果连续采集在用、先暂停
		if (start_timer_flag_)
		{

			stopCapturingOneFrameBaseThread();

			ret_code = DfSetParamCameraConfidence(firmware_config_param_.confidence);
			if (0 == ret_code)
			{
				//ui.spinBox_camera_exposure->setValue(system_config_param_.camera_exposure_time);
				QString str = u8"设置相机置信度: " + QString::number(firmware_config_param_.confidence);
				addLogMessage(str);
			}

			do_pushButton_capture_continuous();

		}
		else
		{
			ret_code = DfSetParamCameraConfidence(firmware_config_param_.confidence);

			if (0 == ret_code)
			{
				//ui.spinBox_camera_exposure->setValue(system_config_param_.camera_exposure_time);
				QString str = u8"设置相机置信度: " + QString::number(firmware_config_param_.confidence);
				addLogMessage(str);
			}

		}

	}

	camera_setting_flag_ = false;

}

bool CameraCaptureGui::manyExposureParamHasChanged()
{
	if (firmware_config_param_.mixed_exposure_num != ui.spinBox_exposure_num->value())
	{
		return true;
	}

	for (int i = 0; i < exposure_time_list_.size(); i++)
	{
		if (firmware_config_param_.mixed_exposure_param_list[i] != exposure_time_list_[i]->value() || firmware_config_param_.mixed_led_param_list[i] != led_current_list_[i]->value())
		{
			return true;
		}
	}

	return false;
}

bool CameraCaptureGui::setCameraConfigParam()
{
	int ret_code = DfSetParamLedCurrent(system_config_param_.led_current);
	if (0 != ret_code)
	{
		qDebug() << "Set Led Curretn Error;";
		addLogMessage(u8"设置亮度失败！");
		return false;
	}


	//更新多曝光参数 
	ret_code = DfSetParamMixedHdr(firmware_config_param_.mixed_exposure_num, firmware_config_param_.mixed_exposure_param_list, firmware_config_param_.mixed_led_param_list);
	if (0 != ret_code)
	{
		qDebug() << "Set HDR Param Error;";
		addLogMessage(u8"设置高动态参数失败！");
		return false;
	}


	ret_code = DfSetParamStandardPlaneExternal(&system_config_param_.standard_plane_external_param[0],
		&system_config_param_.standard_plane_external_param[9]);

	if (0 != ret_code)
	{
		addLogMessage(u8"设置基准平面参数失败！");
		return false;
	}

	ret_code = DfSetParamCameraExposure(system_config_param_.camera_exposure_time);
	if (0 != ret_code)
	{
		addLogMessage(u8"设置曝光参数失败！");
		return false;
	}

	ret_code = DfSetParamGenerateBrightness(firmware_config_param_.generate_brightness_model, firmware_config_param_.generate_brightness_exposure);
	if (0 != ret_code)
	{
		addLogMessage(u8"设置生成亮度图参数失败！");
	}

	ret_code = DfSetParamBilateralFilter(firmware_config_param_.use_bilateral_filter, firmware_config_param_.bilateral_filter_param_d);
	if (0 != ret_code)
	{
		addLogMessage(u8"设置平滑参数失败！");
	}

	ret_code = DfSetParamCameraConfidence(firmware_config_param_.confidence);
	if (0 != ret_code)
	{
		addLogMessage(u8"设置置信度失败！");
	}

	ret_code = DfSetParamCameraGain(system_config_param_.camera_gain);
	if (0 != ret_code)
	{
		addLogMessage(u8"设置相机增益失败！");
	}

	ret_code = DfSetParamCameraGamma(system_config_param_.camera_gamma);
	if (0 != ret_code)
	{
		addLogMessage(u8"设置相机伽马矫正失败！");
	}

	ret_code = DfSetParamRadiusFilter(firmware_config_param_.use_radius_filter, firmware_config_param_.radius_filter_r, firmware_config_param_.radius_filter_threshold_num);
	if (0 != ret_code)
	{
		addLogMessage(u8"设置半径滤波参数失败！");
	}

	ret_code = DfSetParamDepthFilter(firmware_config_param_.use_depth_filter, firmware_config_param_.depth_filter_threshold);
	if (0 != ret_code)
	{
		addLogMessage(u8"设置深度去噪参数失败！");
	}

	int projector_version = 3010;
	ret_code = DfGetProjectorVersion(projector_version);
	if (0 != ret_code)
	{
		addLogMessage(u8"获取光机型号失败！");
	}
	else
	{
		switch (projector_version)
		{
		case DF_PROJECTOR_3010:
		{
			exposure_time_max_ = 100000;
			exposure_time_min_ = 10000;
		}
		break;
		case DF_PROJECTOR_4710:
		{
			exposure_time_max_ = 28000;
			exposure_time_min_ = 1700;
		}
		break;
		default:
			break;
		}
	}

	return true;
}

void CameraCaptureGui::sleep(int sectime)
{
	QTime dieTime = QTime::currentTime().addMSecs(sectime);

	while (QTime::currentTime() < dieTime) {
		QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
	}
}


void CameraCaptureGui::updateOutlierRemovalConfigParam(struct FirmwareConfigParam param)
{
	//if (param.use_reflect_filter != firmware_config_param_.use_reflect_filter)
	//{
	//	if (DF_SUCCESS == DfSetParamReflectFilter(param.use_reflect_filter))
	//	{
	//		firmware_config_param_.use_reflect_filter = param.use_reflect_filter;
	//		QString str = u8"设置反射点滤除： "+QString::number(param.use_reflect_filter);
	//		addLogMessage(str);
	//	}
	//}

	//if (param.use_radius_filter != firmware_config_param_.use_radius_filter)
	//{
		if (DF_SUCCESS == DfSetParamRadiusFilter(param.use_radius_filter, param.radius_filter_r, param.radius_filter_threshold_num));
		{
			firmware_config_param_.use_radius_filter = param.use_radius_filter;
			firmware_config_param_.radius_filter_r = param.radius_filter_r;
			firmware_config_param_.radius_filter_threshold_num = param.radius_filter_threshold_num;

			QString cmd = (param.use_radius_filter ? u8"打开" : u8"关闭");
			QString str = cmd + u8"半径滤波";
			if (param.use_radius_filter)
			{
				str += u8"，";
				str += u8"半径： " + QString::number(param.radius_filter_r, 'f', 1) + "mm";
				str += u8"，";
				str += u8"点数： " + QString::number(param.radius_filter_threshold_num);
			}
			else
			{
				str += u8"！";

			}
			addLogMessage(str);
		}
	//}
	 
}

void CameraCaptureGui::getFirmwareConfigParam(struct FirmwareConfigParam& param)
{
	param = firmware_config_param_;
}

void CameraCaptureGui::getGuiConfigParam(struct GuiConfigDataStruct& gui_param)
{
	gui_param = processing_gui_settings_data_;
}

bool CameraCaptureGui::getShowCalibrationMessage(struct SystemConfigParam& config_param, struct CameraCalibParam& calibration_param)
{
	if (!isConnect())
	{
		return false;
	}

	config_param = system_config_param_;
	calibration_param = camera_calibration_param_;

	return true;
}

//更新生成亮度图参数
void CameraCaptureGui::updateGenerateBrightnessParam()
{
	if (camera_setting_flag_)
	{
		return;
	}

	//设置参数时加锁
	camera_setting_flag_ = true;
	if (connected_flag_)
	{

		int ret_code = -1;
		//如果连续采集在用、先暂停
		if (start_timer_flag_)
		{

			stopCapturingOneFrameBaseThread();


			ret_code = DfSetParamGenerateBrightness(generate_brightness_model_, generate_brightness_exposure_);
			if (0 == ret_code)
			{
				QString str = u8"设置生成亮度图参数 ";
				addLogMessage(str);
			}
			do_pushButton_capture_continuous();
		}
		else
		{
			ret_code = DfSetParamGenerateBrightness(generate_brightness_model_, generate_brightness_exposure_);

			if (0 == ret_code)
			{
				QString str = u8"设置生成亮度图参数 ";
				addLogMessage(str);
			}

		}

	}
	camera_setting_flag_ = false;
}

//更新多曝光参数
void CameraCaptureGui::updateManyExposureParam()
{
	for (int i = 0; i < exposure_time_list_.size(); i++)
	{
		firmware_config_param_.mixed_exposure_param_list[i] = exposure_time_list_[i]->value();
		firmware_config_param_.mixed_led_param_list[i] = led_current_list_[i]->value();
	}

	firmware_config_param_.mixed_exposure_num = exposure_time_list_.size();


	int ret_code = DfSetParamMixedHdr(firmware_config_param_.mixed_exposure_num, firmware_config_param_.mixed_exposure_param_list, firmware_config_param_.mixed_led_param_list);
	if (0 != ret_code)
	{
		qDebug() << "Get Param Error;";
		return;
	}


	//int ret_code = DfSetParamHdr(system_config_param_.exposure_num, system_config_param_.exposure_param);

	//if (0 != ret_code)
	//{
	//	qDebug() << "Set HDR Param Error;";
	//	return;
	//}

	QString str = u8"同步多曝光参数 ";
	addLogMessage(str);

}


void CameraCaptureGui::do_more_exposure_param_changed(int val)
{
	qDebug() << "value changed!";

	bool changed = manyExposureParamHasChanged();
	if (changed)
	{

		if (camera_setting_flag_)
		{
			return;
		}


		//设置参数时加锁
		camera_setting_flag_ = true;
		if (connected_flag_)
		{
			//如果连续采集在用、先暂停
			if (start_timer_flag_)
			{
				stopCapturingOneFrameBaseThread();
				updateManyExposureParam();
				do_pushButton_capture_continuous();
			}
			else
			{
				updateManyExposureParam();
			}
		}

		camera_setting_flag_ = false;

	}

}


void CameraCaptureGui::do_spin_exposure_num_changed(int val)
{

	int item_num = ui.tableWidget_more_exposure->rowCount();


	for (int row = item_num - 1; row >= 0; row--)
	{
		remove_exposure_item(row);
	}

	std::vector<int> old_exposure_list;
	std::vector<int> old_led_list;

	std::vector<QSpinBox*> old_exposure_time_list = exposure_time_list_;
	std::vector<QSpinBox*> old_led_current_list = led_current_list_;


	for (int i = 0; i < exposure_time_list_.size(); i++)
	{
		old_exposure_list.push_back(exposure_time_list_.at(i)->value());
		old_led_list.push_back(led_current_list_.at(i)->value());
	}

	for (int i = 0; i < exposure_time_list_.size(); i++)
	{

		disconnect(exposure_time_list_[i], SIGNAL(valueChanged(int)), this, SLOT(do_more_exposure_param_changed(int)));
		disconnect(led_current_list_[i], SIGNAL(valueChanged(int)), this, SLOT(do_more_exposure_param_changed(int)));
		delete exposure_time_list_[i];
		delete led_current_list_[i];
	}

	exposure_time_list_.clear();
	led_current_list_.clear();

	for (int i = 0; i < val; i++)
	{
		int rows = i;
		int cols = 0;

		//int led_val = system_config_param_.led_current;
		int exposure_val = firmware_config_param_.mixed_exposure_param_list[i];
		int led_val = firmware_config_param_.mixed_led_param_list[i];


		add_exposure_item(rows, exposure_val, led_val);
	}

	//qDebug() << "exposure_time_list size: " << exposure_time_list_.size();
	ui.tableWidget_more_exposure->repaint();
	ui.verticalLayout->update();

	do_more_exposure_param_changed(0);
}

/*************************************************************************************************************************************/

bool CameraCaptureGui::connectCamera(QString ip)
{

	return true;
}

bool CameraCaptureGui::stopCapturingOneFrameBaseThread()
{
	do_pushButton_capture_continuous();

	for (int i = 0; i < 1000; i++)
	{
		sleep(10);
		if (!capturing_flag_)
		{
			//qDebug() << "capturing_flag_: " << capturing_flag_;
			return true;
		}

	}

	return false;
}

void CameraCaptureGui::captureOneFrameBaseThread(bool hdr)
{

	if (!connected_flag_)
	{
		return;
	}


	if (capturing_flag_)
	{
		return;
	}
	capturing_flag_ = true;

	//addLogMessage(u8"采集数据：");

	int width = camera_width_;
	int height = camera_height_;

	int image_size = width * height;

	cv::Mat brightness(height, width, CV_8U, cv::Scalar(0));
	cv::Mat depth(height, width, CV_32F, cv::Scalar(0.));
	cv::Mat point_cloud(height, width, CV_32FC3, cv::Scalar(0.));

	int depth_buf_size = image_size * 1 * 4;
	int brightness_bug_size = image_size;

	int ret_code = -1;

	int exposure_num = 1;
 

	if (ui.radioButton_single_exposure->isChecked())
	{
		exposure_model_ = EXPOSURE_MODEL_SINGLE_;
	}
	else if (ui.radioButton_hdr_exposure->isChecked())
	{
		exposure_model_ = EXPOSURE_MODEL_HDR_;
	}

	switch (exposure_model_)
	{
	case EXPOSURE_MODEL_SINGLE_:
	{
		//单曝光
		//ret_code = DfGetFrame04((float*)depth.data, depth_buf_size, (uchar*)brightness.data, brightness_bug_size);
		exposure_num = 1;
	}
	break;
	case EXPOSURE_MODEL_HDR_:
	{
		//HDR
		bool changed = manyExposureParamHasChanged();

		if (changed)
		{
			updateManyExposureParam();
		}


		//ret_code = DfGetFrameHdr((float*)depth.data, depth_buf_size, (uchar*)brightness.data, brightness_bug_size);
		exposure_num = firmware_config_param_.mixed_exposure_num;

		ret_code = DfSetParamMultipleExposureModel(1);
		if (0 != ret_code)
		{
			std::cout << "Set Multiple Exposure Model Error;" << std::endl;
		}
	}
	break;
	break;

	default:
		break;
	}

	 

	char c_time[100];

	ret_code = DfCaptureData(exposure_num, c_time);

	if (DF_SUCCESS != ret_code)
	{
		addLogMessage(u8"采集数据异常： " + QString::number(ret_code));
		ret_code = 0;
	}


	/***************************************************************************/
	if (0 == ret_code)
	{
		//获取亮度图数据
		ret_code = DfGetBrightnessData(brightness.data);
		if (DF_SUCCESS != ret_code)
		{
			std::cout << "Get Brightness Error!" << std::endl;
		}

		//获取深度图数据
		ret_code = DfGetDepthDataFloat((float*)depth.data);

		if (DF_SUCCESS != ret_code)
		{
			std::cout << "Get Depth Error!" << std::endl;
		}

		/******************************************************************************/


		float temperature = 0;
		ret_code = DfGetDeviceTemperature(temperature);
		std::cout << "temperature: " << temperature << std::endl;
		emit send_temperature_update(temperature);

		//capture_m_mutex_.lock();

		brightness_map_ = brightness.clone();
		depth_map_ = depth.clone();
		undistort_brightness_map_ = brightness_map_;
		undistort_depth_map_ = depth_map_;

		depthTransformPointcloud((float*)depth.data, (float*)point_cloud.data); 

		pointcloud_map_ = point_cloud.clone();
		undistort_pointcloud_map_ = pointcloud_map_;
		transformPointcloud((float*)point_cloud.data, (float*)point_cloud.data, system_config_param_.standard_plane_external_param, &system_config_param_.standard_plane_external_param[9]);

		std::vector<cv::Mat> channels;
		cv::split(point_cloud, channels);
		height_map_ = channels[2].clone();

		//capture_m_mutex_.unlock();


		capture_show_flag_ = true;


		addLogMessage(u8"采集完成！");
		emit send_images_update();


	}
	else
	{
		start_timer_flag_ = false;
		addLogMessage(u8"采集失败！");
	}

	capturing_flag_ = false;
}

void CameraCaptureGui::captureOneRawFrameBaseThread()
{

	if (!connected_flag_)
	{
		return;
	}


	if (capturing_flag_)
	{
		return;
	}
	capturing_flag_ = true;

	//addLogMessage(u8"采集数据：");

	int width = camera_width_;
	int height = camera_height_;

	int image_size = width * height;

	unsigned char* raw_images_buffer = new unsigned char[(long long)image_size * 28];

	int ret_code = -1;
 
	ret_code = DfGetCameraRawData01(raw_images_buffer, (long long)image_size * 28);

	if (DF_SUCCESS != ret_code)
	{
		addLogMessage(u8"采集数据异常： " + QString::number(ret_code));
		ret_code = 0;
	}


	/***************************************************************************/
	if (0 == ret_code)
	{
		// 从里面获得正确的图像
		cv::Mat image_left(height, width, CV_8UC1, raw_images_buffer + ((long long)image_size * 13));
		cv::Mat image_right(height, width, CV_8UC1, raw_images_buffer + ((long long)image_size * 27));

		cv::Mat image_left_bright(height, width, CV_8UC1, raw_images_buffer + ((long long)image_size * 12));
		cv::Mat image_right_bright(height, width, CV_8UC1, raw_images_buffer + ((long long)image_size * 26));

		//image_left_bright.convertTo(image_left_bright, CV_8UC3);
		//image_right_bright.convertTo(image_right_bright, CV_8UC3);
		cv::cvtColor(image_left_bright, image_left_bright, cv::COLOR_GRAY2BGR);
		cv::cvtColor(image_right_bright, image_right_bright, cv::COLOR_GRAY2BGR);

		Calibrate_Function calib_function;
		calib_function.setBoardMessage(board_message_);

		std::vector<cv::Point2f> circle_points[2];
		bool found_l = calib_function.findCircleBoardFeature(image_left, circle_points[0]);
		bool found_r = calib_function.findCircleBoardFeature(image_right, circle_points[1]);

		cv::Mat color_img_l;
		cv::Mat color_img_r;
		cv::Size board_size = calib_function.getBoardSize();
		cv::cvtColor(image_left, color_img_l, cv::COLOR_GRAY2BGR);
		cv::cvtColor(image_right, color_img_r, cv::COLOR_GRAY2BGR);
		cv::drawChessboardCorners(color_img_l, board_size, circle_points[0], found_l);
		cv::drawChessboardCorners(color_img_r, board_size, circle_points[1], found_r);

		cv::Mat dark_temp;
		cv::Mat bright_temp;
		cv::hconcat(color_img_l, color_img_r, dark_temp);
		cv::hconcat(image_left_bright, image_right_bright, bright_temp);
		cv::vconcat(dark_temp, bright_temp, render_image_raw_);

		float temperature = 0;
		ret_code = DfGetDeviceTemperature(temperature);
		std::cout << "temperature: " << temperature << std::endl;
		emit send_temperature_update(temperature);

		capture_show_flag_ = true;

		addLogMessage(u8"采集Raw完成！");
		emit send_images_update();


	}
	else
	{
		start_timer_flag_ = false;
		addLogMessage(u8"采集Raw失败！");
	}

	delete[] raw_images_buffer;

	capturing_flag_ = false;
}

bool CameraCaptureGui::captureOneFrameData()
{

	if (!connected_flag_)
	{
		return false;
	}

	int width = camera_width_;
	int height = camera_height_;


	addLogMessage(u8"采集数据：");
	/*****************************************************************************/
	int image_size = width * height;

	cv::Mat brightness(height, width, CV_8U, cv::Scalar(0));
	cv::Mat depth(height, width, CV_32F, cv::Scalar(0.));
	cv::Mat point_cloud(height, width, CV_32FC3, cv::Scalar(0.));

	int depth_buf_size = image_size * 1 * 4;
	int brightness_bug_size = image_size;

	if (ui.radioButton_single_exposure->isChecked())
	{
		exposure_model_ = EXPOSURE_MODEL_SINGLE_;
	}
	else if (ui.radioButton_hdr_exposure->isChecked())
	{ 
		exposure_model_ = EXPOSURE_MODEL_HDR_;
	}

	int ret_code = 0;

	int exposure_num = 1;

	switch (exposure_model_)
	{
	case EXPOSURE_MODEL_SINGLE_:
		{
			//单曝光
			//ret_code = DfGetFrame04((float*)depth.data, depth_buf_size, (uchar*)brightness.data, brightness_bug_size);
			exposure_num = 1;
		}
		break;
	case EXPOSURE_MODEL_HDR_:
		{
			//HDR
			bool changed = manyExposureParamHasChanged();

			if (changed)
			{
				updateManyExposureParam();
			}

			exposure_num = firmware_config_param_.mixed_exposure_num;

			ret_code = DfSetParamMultipleExposureModel(1);
			if (0 != ret_code)
			{
				std::cout << "Set Multiple Exposure Model Error;" << std::endl;
			}

			//ret_code = DfGetFrameHdr((float*)depth.data, depth_buf_size, (uchar*)brightness.data, brightness_bug_size);
		}
		break;
		break;

		default:
			break;
	}

	char c_time[100];
	
	ret_code = DfCaptureData(exposure_num, c_time);

	if (DF_SUCCESS != ret_code)
	{ 
		addLogMessage(u8"采集数据异常： "+ QString::number(ret_code));
		ret_code = 0;
	}


	/***************************************************************************/
	if (0 == ret_code)
	{

		//获取亮度图数据
		ret_code = DfGetBrightnessData(brightness.data);
		if (DF_SUCCESS != ret_code)
		{
			std::cout << "Get Brightness Error!" << std::endl;
		}

		//获取深度图数据
		ret_code = DfGetDepthDataFloat((float*)depth.data);

		if (DF_SUCCESS != ret_code)
		{
			std::cout << "Get Depth Error!" << std::endl;
		}

		/******************************************************************************/

		processing_gui_settings_data_.Instance().exposure_model = exposure_model_;


		brightness_map_ = brightness.clone();
		depth_map_ = depth.clone();
		undistort_brightness_map_ = brightness_map_;
		undistort_depth_map_ = depth_map_;

		 
		depthTransformPointcloud((float*)depth.data, (float*)point_cloud.data);
		pointcloud_map_ = point_cloud.clone();
		addLogMessage(u8"采集完成！");

		transformPointcloud((float*)point_cloud.data, (float*)point_cloud.data, system_config_param_.standard_plane_external_param, &system_config_param_.standard_plane_external_param[9]);

		std::vector<cv::Mat> channels;
		cv::split(point_cloud, channels);
		height_map_ = channels[2].clone();



		float temperature = 0;
		ret_code = DfGetDeviceTemperature(temperature);
		//ret_code = DfGetProjectorTemperature(temperature);


		emit send_temperature_update(temperature);

		return true;
	}

	addLogMessage(u8"采集失败！");
	return false;

}


bool CameraCaptureGui::isConnect()
{
	return connected_flag_;
}


void CameraCaptureGui::do_pushButton_refresh()
{
	device_mac_list_.clear();
	device_ip_list_.clear();
	ui.comboBox_ip->clear();

	int ret_code = 0;
	//更新相机设备列表
	int camera_num = 0;
	ret_code = DfUpdateDeviceList(camera_num);
	if (0 != ret_code || 0 == camera_num)
	{
		return  ;
	}

	DeviceBaseInfo* pBaseinfo = (DeviceBaseInfo*)malloc(sizeof(DeviceBaseInfo) * camera_num);
	int n_size = camera_num * sizeof(DeviceBaseInfo);
	//获取设备信息
	ret_code = DfGetAllDeviceBaseInfo(pBaseinfo, &n_size);
	for (int i = 0; i < camera_num; i++)
	{
		device_mac_list_.push_back(QString(pBaseinfo[i].mac));
		device_ip_list_.push_back(QString(pBaseinfo[i].ip));
		//std::cout << "mac: " << pBaseinfo[i].mac << "  ip: " << pBaseinfo[i].ip << std::endl;
		QString text = QString(pBaseinfo[i].ip) + "(" + QString(pBaseinfo[i].mac)+  ")";
		ui.comboBox_ip->addItem(text);
	}


}


void CameraCaptureGui::do_pushButton_connect_async()
{
	showLoadingForm();
	QFuture<void> fut = QtConcurrent::run(this, &CameraCaptureGui::do_pushButton_connect);
	while (!fut.isFinished())
	{
		QApplication::processEvents(QEventLoop::AllEvents, 100);
		//qDebug() << "timeout";
 
	}

	hideLoadingForm();

}

void  CameraCaptureGui::do_pushButton_connect()
{
	ui.pushButton_connect->setDisabled(true);    

	do {

		if (!connected_flag_)
		{

			camera_ip_ = ui.lineEdit_ip->text();
			if (camera_ip_.isEmpty())
			{
				addLogMessage(u8"请设置IP！");
				break;
			}

			QRegExp rx2("((2[0-4]\\d|25[0-5]|[01]?\\d\\d?)\\.){3}(2[0-4]\\d|25[0-5]|[01]?\\d\\d?)");
			if (!rx2.exactMatch(camera_ip_))
			{

				addLogMessage(u8"IP无效，请确认相机IP！");
				break;
			}
 


			addLogMessage(u8"连接相机：");
			int ret_code = DfConnect(camera_ip_.toStdString().c_str());
 
  
			DfRegisterOnDropped(m_p_OnDropped_);


			if (DF_SUCCESS == ret_code)
			{
				//必须连接相机成功后，才可获取相机分辨率
				ret_code = DfGetCameraResolution(&camera_width_, &camera_height_);
				std::cout << "Width: " << camera_width_ << "    Height: " << camera_height_ << std::endl;

				//获取相机标定参数
				ret_code = DfGetCalibrationParam(camera_calibration_param_);
				if (DF_SUCCESS != ret_code)
				{
					qDebug() << "Get Calibration Param Error!;";
					 
					break;
				}

				ret_code = DfGetFirmwareVersion(firmware_version_, _VERSION_LENGTH_);
				if (DF_SUCCESS != ret_code)
				{
					qDebug() << "Get Firmware Version Error!;";
					break;
				}

				ret_code = DfGetProductInfo(info_, INFO_SIZE);
				if (DF_SUCCESS != ret_code)
				{
					qDebug() << "Get Firmware Version Error!;";
					break;
				}

				//获取相机型号参数
				//ret_code = DfGetCameraVersion(camera_version_);
				//if (0 != ret_code)
				//{
				//	qDebug() << "Get Calibration Param Error!;";
				//	return;
				//}

				//switch (camera_version_)
				//{
				//case 800:
				//{
				//	addLogMessage(u8"连接DFX800成功！");
				//}
				//break;
				//case 1800:
				//{
				//	addLogMessage(u8"连接DFX1800成功！");
				//}
				//break;
				//default:
				//	break;
				//}


				addLogMessage(u8"连接相机成功！");
				//保存ip配置
				processing_gui_settings_data_.Instance().ip = camera_ip_;


				//设置配置参数
				ret_code = DfSetSystemConfigParam(system_config_param_);
				if (DF_SUCCESS != ret_code)
				{
					qDebug() << "Set Param Error;";
					//return;
				}

				if (!setCameraConfigParam())
				{
					qDebug() << "Set Signal Param Error;";
				}

				undateSystemConfigUiData();

				int network_speed = 0;
				ret_code = DfGetNetworkBandwidth(network_speed);
				if (DF_SUCCESS != ret_code)
				{
					qDebug() << "Get Network Speed Error;";
					//return;
				}
				else
				{
					if (1000 != network_speed)
					{
						addLogMessage(u8"请注意网络带宽:" + QString::number(network_speed) + "M");

					}
				}

			}
			else if (DF_BUSY == ret_code)
			{
				addLogMessage(u8"相机忙！");
				 
				break;
			}
			else if (DF_ERROR_2D_CAMERA == ret_code)
			{
				addLogMessage(u8"2D相机故障！");
				 
				break;
			}
			else
			{
				std::cout << "Connect Camera Error!";
				addLogMessage(u8"连接相机失败，请确认相机IP！");
				 
				break;
			}


			connected_flag_ = true;
			ui.lineEdit_ip->setDisabled(true);

			CalibrationParam calib_param;
			ret_code = DfGetCalibrationParam(&calib_param);

			if (0 == ret_code)
			{
				std::cout << "intrinsic: " << std::endl;
				for (int r = 0; r < 3; r++)
				{
					for (int c = 0; c < 3; c++)
					{
						std::cout << calib_param.intrinsic[3 * r + c] << "\t";
					}
					std::cout << std::endl;
				}

				std::cout << "extrinsic: " << std::endl;
				for (int r = 0; r < 4; r++)
				{
					for (int c = 0; c < 4; c++)
					{
						std::cout << calib_param.extrinsic[4 * r + c] << "\t";
					}
					std::cout << std::endl;
				}

				std::cout << "distortion: " << std::endl;
				for (int r = 0; r < 1; r++)
				{
					for (int c = 0; c < 12; c++)
					{
						std::cout << calib_param.distortion[1 * r + c] << "\t";
					}
					std::cout << std::endl;
				}
			}
			else
			{
				std::cout << "Get Calibration Data Error!"; 
				break;
			}

			ui.pushButton_connect->setIcon(QIcon(":/dexforce_camera_gui/image/disconnect.png"));
		}
		else
		{
			//断开相机
			do_pushButton_disconnect();

		}
		 

	} while (0);

	 
	ui.pushButton_connect->setEnabled(true); 

}

void  CameraCaptureGui::do_pushButton_disconnect()
{

	if (connected_flag_)
	{
		if (start_timer_flag_)
		{
			stopCapturingOneFrameBaseThread();
		}

		int ret = DfDisconnect(camera_ip_.toStdString().c_str());
		//if (DF_SUCCESS != ret)
		//{
		//	return;
		//}

		addLogMessage(u8"断开相机！");
		connected_flag_ = false; 
		ui.lineEdit_ip->setDisabled(false);

		ui.pushButton_connect->setIcon(QIcon(":/dexforce_camera_gui/image/connect.png")); 
		ui.pushButton_capture_continuous->setIcon(QIcon(":/dexforce_camera_gui/image/video_start.png"));
	}


}


double CameraCaptureGui::computePointsDistance(cv::Point2f p_0, cv::Point2f p_1, cv::Mat point_cloud)
{

	std::vector<cv::Mat> point_cloud_channels;
	cv::split(point_cloud, point_cloud_channels);

	//插值
	cv::Point3f f_p_0_inter, f_p_1_inter;

	Calibrate_Function calib_function;
	calib_function.setBoardMessage(board_message_);
	f_p_0_inter.x = calib_function.Bilinear_interpolation(p_0.x, p_0.y, point_cloud_channels[0]);
	f_p_0_inter.y = calib_function.Bilinear_interpolation(p_0.x, p_0.y, point_cloud_channels[1]);
	f_p_0_inter.z = calib_function.Bilinear_interpolation(p_0.x, p_0.y, point_cloud_channels[2]);

	f_p_1_inter.x = calib_function.Bilinear_interpolation(p_1.x, p_1.y, point_cloud_channels[0]);
	f_p_1_inter.y = calib_function.Bilinear_interpolation(p_1.x, p_1.y, point_cloud_channels[1]);
	f_p_1_inter.z = calib_function.Bilinear_interpolation(p_1.x, p_1.y, point_cloud_channels[2]);

	cv::Point3f differ_p = f_p_1_inter - f_p_0_inter;
	double differ_val = std::sqrt(differ_p.x * differ_p.x + differ_p.y * differ_p.y + differ_p.z * differ_p.z);

	return differ_val;
}

void CameraCaptureGui::do_pushButton_calibrate_external_param()
{

	if (depth_map_.empty() || brightness_map_.empty())
	{
		return;
	}


	cv::Mat cameraMatrix(3, 3, CV_32FC1, camera_calibration_param_.camera_intrinsic);
	cv::Mat distCoeffs = cv::Mat(5, 1, CV_32F, camera_calibration_param_.camera_distortion);

	if (true)
	{
		//ICP
		//cv::Mat undist_img;
		//cv::undistort(brightness_map_, undist_img, cameraMatrix, distCoeffs);
		//std::vector<cv::Point2f> undist_circle_points;


		Calibrate_Function calib_function;
		calib_function.setBoardMessage(board_message_);
		std::vector<cv::Point2f> circle_points;
		bool found = calib_function.findCircleBoardFeature(brightness_map_, circle_points);

		if (!found)
		{ 
			addLogMessage(u8"无法识别标定板！");
			return;
		}


		cv::medianBlur(depth_map_, depth_map_, 3);

		cv::Mat points_map(brightness_map_.size(), CV_32FC3, cv::Scalar(0., 0., 0.));
		depthTransformPointcloud((float*)depth_map_.data, (float*)points_map.data);

		/*******************************************************************************************/
		std::vector<cv::Point3f> point_3d;
		bilinearInterpolationFeaturePoints(circle_points, point_3d, points_map);

		PrecisionTest precision_machine;
		cv::Mat pc1(point_3d.size(), 3, CV_64F, cv::Scalar(0));
		cv::Mat pc2(point_3d.size(), 3, CV_64F, cv::Scalar(0));

		std::vector<cv::Point3f> world_points = calib_function.generateAsymmetricWorldFeature(board_message_);

		for (int r = 0; r < point_3d.size(); r++)
		{
			pc2.at<double>(r, 0) = point_3d[r].x;
			pc2.at<double>(r, 1) = point_3d[r].y;
			pc2.at<double>(r, 2) = point_3d[r].z;
		}
		for (int r = 0; r < world_points.size(); r++)
		{
			pc1.at<double>(r, 0) = world_points[r].x;
			pc1.at<double>(r, 1) = world_points[r].y;
			pc1.at<double>(r, 2) = world_points[r].z;
		}

		cv::Mat r(3, 3, CV_64F, cv::Scalar(0));
		cv::Mat t(3, 3, CV_64F, cv::Scalar(0));

		precision_machine.svdIcp(pc1, pc2, r, t);

		/******************************************************************************************/

		std::vector<cv::Point3f> transform_points;

		precision_machine.transformPoints(point_3d, transform_points, r, t);

		double diff = precision_machine.computeTwoPointSetDistance(world_points, transform_points);
		addLogMessage(u8"映射精度: "+QString::number(diff,'f',3));
		if (diff > 10)
		{
			addLogMessage(u8"映射基准平面失败，请检查标定板型号！");
			return;
		}
		/******************************************************************************************/


		r.convertTo(r, CV_32F);
		t.convertTo(t, CV_32F);
		transformPointcloud((float*)points_map.data, (float*)points_map.data, (float*)r.data, (float*)t.data);


		std::vector<cv::Mat> channels;
		cv::split(points_map, channels);
		cv::Mat height_map = channels[2].clone();

		height_map_ = height_map.clone();
		renderHeightImage(height_map);
		showImage();

		ui.doubleSpinBox_min_z->setValue(-10);
		ui.doubleSpinBox_max_z->setValue(10);

		for (int i = 0; i < 9; i++)
		{
			system_config_param_.standard_plane_external_param[i] = r.ptr<float>(0)[i];
			//qDebug() << system_config_param_.external_param[i];
		}

		for (int i = 0; i < 3; i++)
		{
			system_config_param_.standard_plane_external_param[9 + i] = t.ptr<float>(0)[i];
			//qDebug() << translation_mat.ptr<float>(0)[i];
		}
		if (connected_flag_)
		{
			int ret_code = DfSetSystemConfigParam(system_config_param_);
			if (0 != ret_code)
			{
				qDebug() << "Get Param Error;";
				return;
			}
			QString str = u8"保存高度映射基准平面参数";
			addLogMessage(str);

		}
		else
		{
			QString str = u8"相机已断连！";
			addLogMessage(str);
		}
	}

	/**************************************************************************************************/
}

void CameraCaptureGui::do_pushButton_test_accuracy()
{
	Calibrate_Function calib_function;
	calib_function.setBoardMessage(board_message_);

	if (brightness_map_.empty() || brightness_map_.type() != CV_8UC1)
	{
		return;
	}

	cv::Mat cameraMatrix(3, 3, CV_32FC1, camera_calibration_param_.camera_intrinsic);
	cv::Mat distCoeffs = cv::Mat(5, 1, CV_32F, camera_calibration_param_.camera_distortion);


	cv::Mat img = brightness_map_.clone();
	cv::Mat undist_img;
	cv::undistort(brightness_map_, undist_img, cameraMatrix, distCoeffs);



	/*******************************************************************************/
	//PNP
	if (false)
	{
		/*************************************************************************************/
		//PNP精度

		std::vector<cv::Point2f> circle_points;
		bool found = calib_function.findCircleBoardFeature(img, circle_points);

		if (found)
		{
			Calibrate_Function calib_machine; 
			calib_machine.setBoardMessage(board_message_);
			std::vector<cv::Point3f> objects = calib_machine.generateAsymmetricWorldFeature(board_message_);

			cv::Mat raux, taux;
			std::vector<cv::Point2f> image_points_pro;

			cv::solvePnP(objects, circle_points, cameraMatrix, distCoeffs, raux, taux, false, cv::SOLVEPNP_EPNP);
			cv::projectPoints(objects, raux, taux, cameraMatrix, distCoeffs, image_points_pro);   //通过得到的摄像机内外参数，对角点的空间三维坐标进行重新投影计算
			double err = cv::norm(cv::Mat(circle_points), cv::Mat(image_points_pro), cv::NORM_L2);
			//addLogMessage(u8"NORM_L2: ") + QString::number(err));

			/*********************************************************************************/
			//平均像素距离
			double sumvalue = 0.0;
			for (size_t i = 0; i < image_points_pro.size(); i++)
			{
				double x = image_points_pro[i].x - circle_points[i].x;
				double y = image_points_pro[i].y - circle_points[i].y;
				sumvalue += sqrt(x * x + y * y);
			}
			sumvalue /= image_points_pro.size();
			addLogMessage(u8"偏差: " + QString::number(sumvalue, 'f', 5) + u8" 像素");
			/*********************************************************************************/

			cv::Mat color_img;
			cv::Size board_size = calib_function.getBoardSize();
			cv::cvtColor(brightness_map_, color_img, cv::COLOR_GRAY2BGR);
			cv::drawChessboardCorners(color_img, board_size, circle_points, found);
			//cv::circle(color_img, circle_points[0], 15, cv::Scalar(0, 255, 0), 2);
			//cv::circle(color_img, circle_points[6], 20, cv::Scalar(0, 255, 0), 2);

			render_image_brightness_ = color_img.clone();
			showImage();
		}
		else
		{
			addLogMessage(u8"无法识别标定板！"); 
		}


		/*************************************************************************************/
	}


	/*****************************************************************************************************/
	//平台测试精度
	if (false)
	{

		PrecisionTest p_test_machine;

		std::vector<cv::Point2f> circle_points;
		bool found = calib_function.findCircleBoardFeature(undist_img, circle_points);

		if (!found)
		{  
			addLogMessage(u8"无法识别标定板！"); 
			return;
		}
		cv::Mat points_map(brightness_map_.size(), CV_32FC3, cv::Scalar(0., 0., 0.));
		depthTransformPointcloud((float*)depth_map_.data, (float*)points_map.data);


		cv::Mat color_img;
		cv::Size board_size = calib_function.getBoardSize();
		cv::cvtColor(brightness_map_, color_img, cv::COLOR_GRAY2BGR);
		cv::drawChessboardCorners(color_img, board_size, circle_points, found);
		render_image_brightness_ = color_img.clone();
		showImage();

		/*******************************************************************************************/
		std::vector<cv::Point3f> point_3d;
		bilinearInterpolationFeaturePoints(circle_points, point_3d, points_map);



		//QString StrCurrentTime = last_path_ + "/feature_points_" + QDateTime::currentDateTime().toString("yyyy-MM-dd_hh-mm-ss"); 
		//StrCurrentTime += ".ply"; 
		//FileIoFunction file_io_machine;
		//file_io_machine.SaveAsciiPointsToPly(point_3d, StrCurrentTime); 
		//addLogMessage(u8"保存圆点: ") + StrCurrentTime);



		std::vector<float> plane;
		cv::Point3f center_point;

		float rms = p_test_machine.fitPlaneBaseLeastSquares(point_3d, plane, center_point);

		//addLogMessage(u8"平面拟合RMS: ") + QString::number(rms));


		center_points_list_.push_back(center_point);
		rms_list_.push_back(rms);
		plane_list_.push_back(plane);
		feature_points_list_.push_back(point_3d);


		if (center_points_list_.size() > 1)
		{
			float dist = p_test_machine.computePointToPlaneDistance(center_points_list_[center_points_list_.size() - 1],
				plane_list_[center_points_list_.size() - 2]);

			//float value = std::floorf(dist);


			addLogMessage(u8"平面拟合RMS: " + QString::number(rms) +
				u8"距离: " + QString::number(dist));
		}
		else
		{
			addLogMessage(u8"平面拟合RMS: " + QString::number(rms) +
				u8"距离: " + QString::number(0));
		}

	}


	/*********************************************************************************************/
	//icp
	if (true)
	{
		//ICP
		//cv::Mat undist_img;
		//cv::undistort(brightness_map_, undist_img, cameraMatrix, distCoeffs);
		//std::vector<cv::Point2f> undist_circle_points;
		Calibrate_Function calib_function;
		calib_function.setBoardMessage(board_message_);

		std::vector<cv::Point2f> circle_points;
		bool found = calib_function.findCircleBoardFeature(brightness_map_, circle_points);

		if (!found)
		{ 
			addLogMessage(u8"无法识别标定板！"); 
			return;
		}

		//显示
		if (true)
		{
			cv::Mat color_img;
			cv::Size board_size = calib_function.getBoardSize();
			cv::cvtColor(brightness_map_, color_img, cv::COLOR_GRAY2BGR);
			cv::drawChessboardCorners(color_img, board_size, circle_points, found);
			render_image_brightness_ = color_img.clone();
			showImage();
		}


		cv::medianBlur(depth_map_, depth_map_, 3);
		cv::Mat points_map(brightness_map_.size(), CV_32FC3, cv::Scalar(0., 0., 0.));
		depthTransformPointcloud((float*)depth_map_.data, (float*)points_map.data);

		/*******************************************************************************************/
		std::vector<cv::Point3f> point_3d;
		bilinearInterpolationFeaturePoints(circle_points, point_3d, points_map);

		PrecisionTest precision_machine;
		cv::Mat pc1(point_3d.size(), 3, CV_64F, cv::Scalar(0));
		cv::Mat pc2(point_3d.size(), 3, CV_64F, cv::Scalar(0));

		std::vector<cv::Point3f> world_points = calib_function.generateAsymmetricWorldFeature(board_message_);

		for (int r = 0; r < point_3d.size(); r++)
		{
			pc2.at<double>(r, 0) = point_3d[r].x;
			pc2.at<double>(r, 1) = point_3d[r].y;
			pc2.at<double>(r, 2) = point_3d[r].z;
		}
		for (int r = 0; r < world_points.size(); r++)
		{
			pc1.at<double>(r, 0) = world_points[r].x;
			pc1.at<double>(r, 1) = world_points[r].y;
			pc1.at<double>(r, 2) = world_points[r].z;
		}

		cv::Mat r(3, 3, CV_64F, cv::Scalar(0));
		cv::Mat t(3, 3, CV_64F, cv::Scalar(0));

		precision_machine.svdIcp(pc1, pc2, r, t);

		std::vector<cv::Point3f> transform_points;

		precision_machine.transformPoints(point_3d, transform_points, r, t);

		double diff = precision_machine.computeTwoPointSetDistance(world_points, transform_points);

		addLogMessage(u8"标定精度: " + QString::number(diff, 'f', 3));

		if (diff > 10)
		{
			addLogMessage(u8"请检查标定板型号！");
		}

		//addLogMessage(u8"平均点偏差: ") + QString::number(diff) +
		//	u8" 距离: ") + QString::number(0));


		//计算平面距离

		std::vector<float> plane;
		cv::Point3f center_point;
		float rms = precision_machine.fitPlaneBaseLeastSquares(point_3d, plane, center_point);


		center_points_list_.push_back(center_point);
		rms_list_.push_back(rms);
		plane_list_.push_back(plane);
		feature_points_list_.push_back(point_3d);


		if (center_points_list_.size() > 1)
		{
			float dist = precision_machine.computePointToPlaneDistance(center_points_list_[center_points_list_.size() - 1],
				plane_list_[center_points_list_.size() - 2]);


			//addLogMessage(u8"标定精度: " + QString::number(diff) +
			//	u8"	距离: " + QString::number(dist));
			 
		}
		else
		{
			//addLogMessage(u8"标定精度: " + QString::number(diff) +
			//	u8"	距离: " + QString::number(0)); 
		}


	}

	/**********************************************************************************************/
	//点距离
	if (false)
	{

		std::vector<cv::Point2f> circle_points;
		bool found = calib_function.findCircleBoardFeature(undist_img, circle_points);

		if (!found)
		{ 
			addLogMessage(u8"无法识别标定板！");
			return;
		}
		cv::Mat points_map(brightness_map_.size(), CV_32FC3, cv::Scalar(0., 0., 0.));
		depthTransformPointcloud((float*)depth_map_.data, (float*)points_map.data);


		cv::Point2f f_p_0_0 = circle_points[0];
		cv::Point2f f_p_0_1 = circle_points[6];

		double dist_0 = computePointsDistance(f_p_0_0, f_p_0_1, points_map) - 120.0;

		double max_err = std::abs(dist_0);

		QString dist_str = QString::number(dist_0);

		cv::Point2f f_p_1_0 = circle_points[6];
		cv::Point2f f_p_1_1 = circle_points[76];

		double dist_1 = computePointsDistance(f_p_1_0, f_p_1_1, points_map) - 100.0;
		dist_str += " , ";
		dist_str += QString::number(dist_1);

		if (std::abs(dist_1) > max_err)
		{
			max_err = std::abs(dist_1);
		}

		cv::Point2f f_p_2_0 = circle_points[76];
		cv::Point2f f_p_2_1 = circle_points[70];

		double dist_2 = computePointsDistance(f_p_2_0, f_p_2_1, points_map) - 120.0;
		dist_str += " , ";
		dist_str += QString::number(dist_2);

		if (std::abs(dist_2) > max_err)
		{
			max_err = std::abs(dist_2);
		}

		cv::Point2f f_p_3_0 = circle_points[70];
		cv::Point2f f_p_3_1 = circle_points[0];

		double dist_3 = computePointsDistance(f_p_3_0, f_p_3_1, points_map) - 100.0;
		dist_str += " , ";
		dist_str += QString::number(dist_3);

		if (std::abs(dist_3) > max_err)
		{
			max_err = std::abs(dist_3);
		}

		//addLogMessage(u8"标定板距离：") + dist_str);
		addLogMessage(u8"偏差：" + QString::number(max_err) + "mm");

	}
}


void CameraCaptureGui::do_pushButton_open_folder()
{

	QFileInfo fileInfo(last_path_);
	QDir dir(fileInfo.absoluteFilePath());

	if (!dir.exists())
	{
		dir.setPath("../TestData");
	}

	QDesktopServices::openUrl(QUrl::fromLocalFile(dir.absolutePath()));//, QUrl::TolerantMode)

}

void CameraCaptureGui::do_pushButton_save_as()
{
	if (brightness_map_.empty() || depth_map_.empty() || height_map_.empty())
	{
		return;
	}

	if (!hide_save_gui_flag_)
	{
		SaveGui config_save_dialog(this);
		config_save_dialog.setDataType(save_data_type_);

		if (QDialog::Accepted == config_save_dialog.exec())
		{
			config_save_dialog.getDateType(save_data_type_);
			hide_save_gui_flag_ = config_save_dialog.isHideUi();

			switch (save_data_type_)
			{
			case SaveDataType::Origin:
				addLogMessage(tr("保存原图"));
				break;
			case SaveDataType::Undistort:
				addLogMessage(tr("保存去畸变图"));
				break;
			default:
				break;
			}
		} 
	}




	QString StrCurrentTime = "/" + QDateTime::currentDateTime().toString("yyyy-MM-dd_hh-mm-ss");

	if (SaveDataType::Undistort == save_data_type_)
	{
		StrCurrentTime += "_undistort";
	}

	QString path = QFileDialog::getSaveFileName(this, "Set Save Name", last_path_ + StrCurrentTime);

	if (path.isEmpty())
	{
		return;
	}

	QFileInfo fileInfo(path);
	last_path_ = fileInfo.absolutePath();


	processing_gui_settings_data_.Instance().last_path = last_path_; 
 

	std::thread t_s(&CameraCaptureGui::saveOneFrameData, this, path);
	t_s.detach();
	addLogMessage(u8"保存路径：" + path);


}


bool CameraCaptureGui::captureOneFrameAndRender()
{

	addLogMessage(u8"采集数据：");
	std::thread t_s(&CameraCaptureGui::captureOneFrameBaseThread, this, false);
	t_s.detach();

	//bool ret = captureOneFrameData();


	//if (ret)
	//{

	//	renderBrightnessImage(brightness_map_);
	//	renderDepthImage(depth_map_);
	//	renderHeightImage(height_map_);
	//	showImage();

	//	if (ui.checkBox_auto_save->isChecked())
	//	{
	//		QString StrCurrentTime = "/" + QDateTime::currentDateTime().toString("yyyy-MM-dd_hh-mm-ss");
	//		QString path_name = sys_path_ + StrCurrentTime;
	//		//saveOneFrameData(path_name);

	//		std::thread t_s(&CameraCaptureGui::saveOneFrameData, this, path_name);
	//		t_s.detach();
	//		addLogMessage(u8"保存路径：" + path_name);
	//	}
	//}
	//else
	//{
	//	addLogMessage(u8"请先连接相机");
	//	return false;
	//}

	return true;
}

bool CameraCaptureGui::captureOneRawFrameAndRender()
{

	addLogMessage(u8"采集Raw数据：");
	std::thread t_s(&CameraCaptureGui::captureOneRawFrameBaseThread, this);
	t_s.detach();

	return true;
}


void  CameraCaptureGui::do_pushButton_capture_one_frame()
{
	//if (!connected_flag_)
	//{
	//	do_pushButton_connect();
	//	captureOneFrameAndRender();
	//	do_pushButton_disconnect();
	//	return;
	//}
	 
	switch (radio_button_flag_)
	{
	case 4:
		captureOneRawFrameAndRender();
		break;

	default:
		captureOneFrameAndRender();
		break;
	}
}


void CameraCaptureGui::do_undate_show_slot()
{

	//capture_m_mutex_.lock();

	if (!capture_show_flag_)
	{
		return;
	}

	renderBrightnessImage(brightness_map_);
	renderDepthImage(depth_map_);
	renderHeightImage(height_map_);

	showImage();

	capture_show_flag_ = false;

	//capture_m_mutex_.unlock();
	if (ui.checkBox_auto_save->isChecked())
	{
		QString StrCurrentTime = "/" + QDateTime::currentDateTime().toString("yyyy-MM-dd_hh-mm-ss");
		QString path_name = sys_path_ + StrCurrentTime;
		//saveOneFrameData(path_name);

		std::thread t_s(&CameraCaptureGui::saveOneFrameData, this, path_name);
		t_s.detach();
		addLogMessage(u8"保存路径：" + path_name);
	}


}


void  CameraCaptureGui::do_timeout_capture_slot()
{

	//std::cout << "capture timeout" << std::endl;

	capture_timer_.stop();

	if (start_timer_flag_ && connected_flag_)
	{
		//bool ret = captureOneFrameAndRender();  
		//if (!ret)
		//{
		//	//停止连续采集
		//	do_pushButton_capture_continuous();
		//} 
		//else
		//{
		//	capture_timer_.start();
		//}
		if (radio_button_flag_ == 4)
		{
			if (!capturing_flag_)
			{
				addLogMessage(u8"采集Raw数据：");
			}
			std::thread t_s(&CameraCaptureGui::captureOneRawFrameBaseThread, this);
			t_s.detach();
			capture_timer_.start();


		}
		else
		{
			if (!capturing_flag_)
			{
				addLogMessage(u8"采集数据：");
			}

			std::thread t_s(&CameraCaptureGui::captureOneFrameBaseThread, this, false);
			t_s.detach();
			capture_timer_.start();
		}

	}
	else
	{
		//	//停止连续采集
		do_pushButton_capture_continuous();

	}

}


void  CameraCaptureGui::do_pushButton_capture_continuous()
{
	if (start_timer_flag_)
	{
		start_timer_flag_ = false;
		capture_timer_.stop();

		ui.pushButton_capture_continuous->setIcon(QIcon(":/dexforce_camera_gui/image/video_start.png"));

		ui.pushButton_capture_one_frame->setEnabled(true);
		ui.pushButton_calibrate_external_param->setEnabled(true);
		ui.pushButton_test_accuracy->setEnabled(true);
	}
	else
	{
		if (connected_flag_)
		{
			start_timer_flag_ = true;
			capturing_flag_ = false;
			capture_timer_.start();
			ui.pushButton_capture_continuous->setIcon(QIcon(":/dexforce_camera_gui/image/video_stop.png"));
			//开始连续采集
			ui.pushButton_capture_one_frame->setDisabled(true);
			ui.pushButton_calibrate_external_param->setDisabled(true);
			ui.pushButton_test_accuracy->setDisabled(true);
		}
		else
		{
			addLogMessage(u8"请先连接相机");
		}

	}


	//capture_thread_ = new QThread(this);
	//capture_thread_->start();

	//capture_timer_ = new QTimer(0);
	//capture_timer_->setInterval(100);
	//capture_timer_->moveToThread(capture_thread_);
	//connect(capture_timer_, SIGNAL(timeout()), this, SLOT(workSlot()), Qt::DirectConnection);
	//connect(capture_thread_, SIGNAL(started()), capture_timer_, SLOT(start()));


	//disconnect(capture_timer_, SIGNAL(timeout()), this, SLOT(workSlot()));
	//disconnect(capture_thread_, SIGNAL(started()), capture_timer_, SLOT(start()));
	//capture_timer_->stop();
	//delete capture_timer_;
}

/*********************************************************************************************************************************/
//HDR显示

//void CameraCaptureGui::do_checkBox_toggled_bilateral_filter(bool state)
//{
//	if (state)
//	{
//		firmware_config_param_.use_bilateral_filter = 1;
//	}
//	else
//	{
//		firmware_config_param_.use_bilateral_filter = 0;
//	}
//
//	if (connected_flag_)
//	{
//		qDebug() << "bilateral_filter_param_d: " << firmware_config_param_.bilateral_filter_param_d;
//		DfSetParamBilateralFilter(firmware_config_param_.use_bilateral_filter, firmware_config_param_.bilateral_filter_param_d);
//
//	}
//}

void CameraCaptureGui::do_comboBox_activated_ip(int index)
{ 
	ui.lineEdit_ip->setText(device_ip_list_[index]);
}

void CameraCaptureGui::do_checkBox_toggled_over_exposure(bool state)
{
	renderBrightnessImage(brightness_map_); 
	showImage();
}

void CameraCaptureGui::do_checkBox_toggled_depth_filter(bool state)
{
	if (state)
	{
		ui.doubleSpinBox_depth_filter->setEnabled(true);
		firmware_config_param_.use_depth_filter = 1;
	}
	else
	{
		ui.doubleSpinBox_depth_filter->setDisabled(true);
		firmware_config_param_.use_depth_filter = 0;
	}


	updateDepthFilter(firmware_config_param_.use_depth_filter, firmware_config_param_.depth_filter_threshold);
}

void CameraCaptureGui::do_checkBox_toggled_hdr(bool state)
{
	processing_gui_settings_data_.Instance().use_hdr_model = state;
	if (connected_flag_)
	{
		if (state)
		{

			bool changed = manyExposureParamHasChanged();

			if (changed)
			{
				if (start_timer_flag_)
				{
					stopCapturingOneFrameBaseThread();
					updateManyExposureParam();
					do_pushButton_capture_continuous();
				}
				else
				{

					updateManyExposureParam();
				}

			}
		}

	}

}


/*****************************************************************************************************************************/
//显示相关
void CameraCaptureGui::do_QRadioButton_toggled_brightness(bool state)
{
	if (state)
	{
		radio_button_flag_ = SELECT_BRIGHTNESS_FLAG_;
		//qDebug() << "state: " << radio_button_flag_;
		showImage();
	}
}

void CameraCaptureGui::do_QRadioButton_toggled_color_depth(bool state)
{
	if (state)
	{
		radio_button_flag_ = SELECT_COLOR_DEPTH_FLAG_;
		//qDebug() << "state: " << radio_button_flag_;
		showImage();
	}
}

void CameraCaptureGui::do_QRadioButton_toggled_gray_depth(bool state)
{
	if (state)
	{
		radio_button_flag_ = SELECT_HEIGHT_MAP_FLAG_;
		//qDebug() << "state: " << radio_button_flag_;
		showImage();
	}
}

void CameraCaptureGui::do_QRadioButton_toggled_calibration(bool state)
{
	if (state)
	{
		radio_button_flag_ = SELECT_CALIBRATE_RAW_FLAG_;
		//qDebug() << "state: " << radio_button_flag_;
		showImage();
	}
}

void CameraCaptureGui::do_QRadioButton_toggled_generate_brightness_default(bool state)
{
	if (state)
	{
		generate_brightness_model_ = GENERATE_BRIGHTNESS_DEFAULT_;
		updateGenerateBrightnessParam();
	}
}

void CameraCaptureGui::do_QRadioButton_toggled_generate_brightness_illumination(bool state)
{
	if (state)
	{
		generate_brightness_model_ = GENERATE_BRIGHTNESS_ILLUMINATION_;
		updateGenerateBrightnessParam();
	}
}

void CameraCaptureGui::do_QRadioButton_toggled_generate_brightness_darkness(bool state)
{
	if (state)
	{
		generate_brightness_model_ = GENERATE_BRIGHTNESS_DARKNESS_;
		updateGenerateBrightnessParam();
	}
}

void CameraCaptureGui::do_spin_generate_brightness_exposure_changed(int val)
{
	generate_brightness_exposure_ = val;

	updateGenerateBrightnessParam();
}

bool CameraCaptureGui::showImage()
{
	switch (radio_button_flag_)
	{
	case 1:
	{
		setImage(render_image_brightness_);
	}
	break;

	case 2:
	{
		setImage(render_image_color_height_);
	}
	break;

	case 3:
	{
		setImage(render_image_color_depth_);
	}
	break;

	case 4:
	{
		setImage(render_image_raw_);
	}
	break;

	default:
		break;
	}

	return true;
}

bool CameraCaptureGui::setImage(cv::Mat img)
{
	cv::Mat show_img = img.clone();

	if (img.empty())
	{
		return false;
	}

	QImage qimg;

	if (show_img.channels() == 3)//RGB Img
	{
		cv::Mat Rgb;
		cv::cvtColor(show_img, Rgb, cv::COLOR_BGR2RGB);//颜色空间转换
		//qimg = QImage((const uchar*)(Rgb.data), Rgb.cols, Rgb.rows, Rgb.step, QImage::Format_RGB888);

		qimg = QImage(show_img.data, show_img.cols, show_img.rows, show_img.step, QImage::Format_RGB888);

	}
	else//Gray Img
	{
		if (CV_8UC1 == show_img.type())
		{
			qimg = QImage((const uchar*)(show_img.data), show_img.cols, show_img.rows, show_img.cols * show_img.channels(), QImage::Format_Indexed8);
		}
		else if (CV_32FC1 == show_img.type())
		{

			show_img.convertTo(show_img, CV_8U);

			qimg = QImage((const uchar*)(show_img.data), show_img.cols, show_img.rows, show_img.cols * show_img.channels(), QImage::Format_Indexed8);

		}

	}



	QPixmap pixmap_ = QPixmap::fromImage(qimg);


	ui.label_image->setPixmap(QPixmap::fromImage(qimg));
	ui.label_image->setPixmap(pixmap_.scaled(ui.label_image->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));  // 保持比例 & 平滑缩放(无锯齿)
	//ui.label_image->setScaledContents(true);

	return true;
}


/************************************************************************************************************************************/

bool CameraCaptureGui::bilinearInterpolationFeaturePoints(std::vector<cv::Point2f> feature_points, std::vector<cv::Point3f>& point_3d, cv::Mat point_cloud)
{
	if (point_cloud.empty())
		return false;


	Calibrate_Function calib_function;
	calib_function.setBoardMessage(board_message_);
	std::vector<cv::Mat> point_cloud_channels;
	cv::split(point_cloud, point_cloud_channels);

	point_3d.clear();

	for (int i = 0; i < feature_points.size(); i++)
	{
		cv::Point2f p_0 = feature_points[i];
		cv::Point3f f_p_inter;

		f_p_inter.x = calib_function.Bilinear_interpolation(p_0.x, p_0.y, point_cloud_channels[0]);
		f_p_inter.y = calib_function.Bilinear_interpolation(p_0.x, p_0.y, point_cloud_channels[1]);
		f_p_inter.z = calib_function.Bilinear_interpolation(p_0.x, p_0.y, point_cloud_channels[2]);

		point_3d.push_back(f_p_inter);
	}


	return true;
}

/*********************************************************************************************************************/
