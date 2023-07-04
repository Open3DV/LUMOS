#ifdef _WIN32  
#include <windows.h>
#elif __linux 
#include <cstring>
#include <stdio.h> 
#define fopen_s(pFile,filename,mode) ((*(pFile))=fopen((filename),  (mode)))==NULL
#endif 
#include <fstream>
#include <QMessageBox>
#include <QLabel>
#include <QFileDialog>
#include <QDateTime>
#include <iostream>
#include <sys/stat.h>
#include "../firmware/protocol.h"
#include "update_firmware_gui.h"
#include "update_opencam3d.h"

UpdateFirmwareGui::UpdateFirmwareGui(QWidget* parent)
	: QDialog(parent)
{
	ui.setupUi(this);

	connect(ui.pushButton_select, SIGNAL(clicked()), this, SLOT(do_pushButton_select()));
	connect(ui.pushButton_update, SIGNAL(clicked()), this, SLOT(do_pushButton_update()));
}

UpdateFirmwareGui::~UpdateFirmwareGui()
{
}


void UpdateFirmwareGui::setCameraIp(QString ip)
{
	camera_ip = ip;
}

void UpdateFirmwareGui::do_pushButton_select()
{
	fileName = QFileDialog::getOpenFileName(this, u8"固件升级", u8".", u8"camera_server");

	if (fileName.isEmpty()) {
		print_log(u8"未选择文件");
	}
	else {
		ui.lineEdit_path->setText(fileName);
		print_log(u8"已选择文件");
	}
}

void UpdateFirmwareGui::do_pushButton_update()
{
	UpdateOnDropped(on_dropped);

	int ret = UpdateConnect(camera_ip.toStdString().c_str());
	if (ret == DF_FAILED)
	{
		print_log(u8"Ip: "+ camera_ip);
		print_log(u8"UpdateConnect failed");
		return;
	}

	// ----------------------------------------------------------
	// 1. Kill the camera_server
	print_log(u8"1. Terminate the camera device service...");

	int feedback = 0;
	KillCameraServer(feedback);
	if (feedback != 1010) {
		print_log(u8"Kill camera_server failed");
		return;
	}

	char log[100] = "";
	sprintf(log, "KillCameraServer: %d", feedback);
	print_log(log);

	// ----------------------------------------------------------
	// 2. Transform the local update camera_server file to camera
	print_log(u8"2. Write the update file into device...");

	std::string str = fileName.toStdString();
	const char* file_name = str.c_str();

	FILE* fw;
	if (fopen_s(&fw, file_name, "rb") != 0)
	{
		print_log(u8"Load file: fail...");
		return;
	}

	fseek(fw, 0, SEEK_END);						// point to file tail
	int file_size = ftell(fw);
	sprintf(log, "File size: %d", file_size);
	print_log(log);

	char* pOrg = new char[file_size];

	fseek(fw, 0, SEEK_SET);						// point to file head
#ifdef _WIN32  
	fread_s(pOrg, file_size, 1, file_size, fw);
#elif __linux
	fread(pOrg, file_size, file_size, fw);
#endif 
	fclose(fw);

	ret = GetCameraServer(pOrg, file_size);

	delete[] pOrg;

	if (ret != DF_SUCCESS) {
		print_log(u8"Update camera_server: fail...");
		return;
	} else {
		print_log(u8"Update camera_server: success...");
	}

	// ----------------------------------------------------------
	// 3. Add firmware permission with -- chmod +x camera_server
	print_log(u8"3. Add the executable permission...");
	feedback = 0;
	ChmodCameraServer(feedback);
	if (feedback != 2020) {
		print_log(u8"Chmod camera_server failed");
		return;
	}

	sprintf(log, "Chmod camera_server: %d", feedback);
	print_log(log);

	// ----------------------------------------------------------
	// 4. Reboot the camera device
	print_log(u8"4. Reboot the device...");
	feedback = 0;
	RebootDevice(feedback);

	UpdateDisconnect();
}

void UpdateFirmwareGui::print_log(QString str)
{
	QString StrCurrentTime = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss.zzz");
	QString log = StrCurrentTime + " " + str;

	ui.textBrowser_log->append(log);
	ui.textBrowser_log->repaint();
}
