/// HEADER
#include <csapex/view/port.h>

using namespace csapex;

Port::Port()
    : refresh_style_sheet_(false)
{
    setFixedSize(16, 16);
}

void Port::paintEvent(QPaintEvent *e)
{
    if(refresh_style_sheet_) {
        refresh_style_sheet_ = false;
        setStyleSheet(styleSheet());
    }
    QFrame::paintEvent(e);
}

void Port::refreshStylesheet()
{
    refresh_style_sheet_ = true;
}
