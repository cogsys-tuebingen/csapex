#include "ctrl_state_display.h"
#include <iostream>
#include <QProgressDialog>
#include <algorithm>

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

    if(current.first == 0 && current.second == 0) {
        bar_->setValue(0);
        bar_->setRange(0,0);
        return;
    }

    int progress = ((double) current.first) / std::max((double) current.second - 1, 1.0) * 100;
    bar_->setValue(progress);
}

void QStateDisplay::despawnBar()
{
    bar_->deleteLater();
}
