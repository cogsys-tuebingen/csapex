/// HEADER
#include "ctrl_toolpanel.h"

/// UI
#include <ui_terra_toolbar.h>

/// SYSTEM
#include <QMainWindow>
#include <QComboBox>
#include <QPainter>
#include <cmath>

CtrlToolPanel::CtrlToolPanel(QMainWindow *tool_panel, CMPCoreBridge::Ptr bridge) :
    tool_bar_(tool_panel),
    bridge_(bridge),
    zoom_(100.0)
{
    BlackPen.setColor(Qt::black);
    BlackPen.setWidth(1);
}

void CtrlToolPanel::setupUI(Ui::ToolPanel *ui)
{
    class_selection_ = ui->classes;
    button_compute_  = ui->compile;
    button_trash_    = ui->trash;
}

void CtrlToolPanel::zoomIn()
{
    zoom_ += 12.5;
    snapZoom();
    Q_EMIT zoom(zoom_);
}

void CtrlToolPanel::zoomOut()
{
    zoom_ -= 12.5;
    snapZoom();
    Q_EMIT zoom(zoom_);
}

void CtrlToolPanel::zoomReset()
{
    Q_EMIT zoom(zoom_);
}

void CtrlToolPanel::zoomUpdate(double factor)
{
    zoom_ = factor;
}

void CtrlToolPanel::classChangend(int index)
{
    if(index == -1)
        return;

    QVariant v = class_selection_->itemData(index);
    Q_EMIT classSelected(v.toInt());
}

void CtrlToolPanel::buttonMov(bool checked)
{
    if(checked) {
        Q_EMIT uncheckAdd(false);
        Q_EMIT uncheckDel(false);
        Q_EMIT uncheckSel(false);
    }
}

void CtrlToolPanel::buttonAdd(bool checked)
{
    if(checked){
        Q_EMIT uncheckMov(false);
        Q_EMIT uncheckDel(false);
        Q_EMIT uncheckSel(false);
    }
}

void CtrlToolPanel::buttonDel(bool checked)
{
    if(checked) {
        Q_EMIT uncheckAdd(false);
        Q_EMIT uncheckMov(false);
        Q_EMIT uncheckSel(false);
    }
}

void CtrlToolPanel::buttonSel(bool checked)
{
    if(checked) {
        Q_EMIT uncheckAdd(false);
        Q_EMIT uncheckMov(false);
        Q_EMIT uncheckDel(false);
    }
}

void CtrlToolPanel::buttonCompute()
{
    button_compute_->setDisabled(true);
    button_trash_->setDisabled(true);

    Q_EMIT compute();
}

void CtrlToolPanel::computationFinished()
{
    button_compute_->setEnabled(true);
    button_trash_->setEnabled(true);
}

void CtrlToolPanel::classUpdate()
{
    class_selection_->clear();
    std::vector<int> ids = bridge_->getClassIDs();
    foreach(int id, ids) {
        class_selection_->addItem(QIcon(renderColorIcon(id)), QString::number(id), QVariant(id));
    }
}

void CtrlToolPanel::snapZoom()
{
    int step = std::floor(zoom_ / 12.5 + 0.5);
    zoom_ = 12.5 * step;
}

QPixmap CtrlToolPanel::renderColorIcon(const int class_ID)
{
    QColor   color = bridge_->getColorByClass(class_ID);
    QPixmap  pixmap(60,20);
    QPainter paintr(&pixmap);
    paintr.setBrush(color);
    paintr.setPen(BlackPen);
    paintr.drawRect(0,0,59,19);
    return pixmap;
}
