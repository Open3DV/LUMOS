#include "save_gui.h"

SaveGui::SaveGui(QWidget *parent)
	: QDialog(parent)
{
	ui.setupUi(this);

	connect(ui.pushButton_ok, SIGNAL(clicked()), this, SLOT(do_pushButton_ok()));
	connect(ui.pushButton_cancel, SIGNAL(clicked()), this, SLOT(do_pushButton_cancel()));
}

SaveGui::~SaveGui()
{

}

void SaveGui::setDataType(SaveDataType type)
{
	switch (type)
	{
	case SaveDataType::Origin:
	{
		ui.radioButton_orgin->setChecked(true);
	}
		break;
	case SaveDataType::Undistort:
	{
		ui.radioButton_undistort->setChecked(true);
	}
		break;
	default:
		break;
	}

	data_class_ = type;
}

void SaveGui::getDateType(SaveDataType& type)
{
	type = data_class_;
}

bool SaveGui::isHideUi()
{
	return hide_ui_;
}

void SaveGui::do_pushButton_ok()
{
	if (ui.radioButton_orgin->isChecked())
	{
		data_class_ = SaveDataType::Origin;
	}

	if (ui.radioButton_undistort->isChecked())
	{
		data_class_ = SaveDataType::Undistort;
	}

	if (ui.checkBox_show->isChecked())
	{
		hide_ui_ = true;
	}
	else
	{
		hide_ui_ = false;
	}

	this->accept();
}

void SaveGui::do_pushButton_cancel()
{
	 
	this->reject();
}