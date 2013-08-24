#include "ctrl_state_display.h"
#include <iostream>
#include <QProgressDialog>


QStateDisplay::QStateDisplay()
{
}

void QStateDisplay::spawnBar(QString title)
{
    bar_ = new QProgressDialog(title, "", 0, 100);
    bar_->setCancelButton(NULL);
    bar_->setVisible(true);
}

void QStateDisplay::stateUpdate(const boost::any &any)
{
    std::pair<int,int> current = boost::any_cast<std::pair<int,int> >(any);
    std::cerr << current.first << " " << current.second << std::endl;

    int progress = ((double) current.first / (double) current.second) * 100;
    bar_->setValue(progress);
}

void QStateDisplay::despawnBar()
{
    bar_->deleteLater();
}
