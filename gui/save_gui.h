#pragma once

#include <QDialog>
#include "ui_save_gui.h"

enum class SaveDataType
{
	Origin = 0,
	Undistort = 1,
};

class SaveGui : public QDialog
{
	Q_OBJECT

public:
	SaveGui(QWidget *parent = nullptr);
	~SaveGui();

	void setDataType(SaveDataType type);
	
	void getDateType(SaveDataType& type);

	bool isHideUi();

private slots:  
	void do_pushButton_ok();

	void do_pushButton_cancel();

private:
	Ui::SaveGuiClass ui;

	SaveDataType data_class_ = SaveDataType::Origin;
	bool hide_ui_ = false;
};
