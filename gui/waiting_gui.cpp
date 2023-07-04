#include "waiting_gui.h"
#include <QMovie>


WaitingGui::WaitingGui(QWidget *parent)
	: QWidget(parent)
{
	ui.setupUi(this);


    this->setWindowFlags(Qt::CustomizeWindowHint | Qt::FramelessWindowHint);

    QMovie* movie;
    movie = new QMovie(":\\dexforce_camera_gui\\image\\load.gif");
    ui.label->setMovie(movie);
    movie->start();

    ui.label->setScaledContents(true);
    ui.label->setStyleSheet("background-color: transparent;");
 
}

WaitingGui::~WaitingGui()
{}
