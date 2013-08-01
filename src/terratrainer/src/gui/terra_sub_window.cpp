#include "terra_sub_window.h"

SubWindow::SubWindow(QWidget *parent, Qt::WindowFlags flags) :
    QMainWindow(parent, flags)
{
}

void SubWindow::closeEvent(QCloseEvent *event)
{
    QMainWindow::closeEvent(event);
    Q_EMIT visible(false);
}
