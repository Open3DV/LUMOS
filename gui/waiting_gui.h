#pragma once

#include <QWidget>
#include "ui_waiting_gui.h"

class WaitingGui : public QWidget
{
	Q_OBJECT

public:
	WaitingGui(QWidget *parent = nullptr);
	~WaitingGui();

private:
	Ui::WaitingGuiClass ui;
};
