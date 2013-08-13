/// HEADER
#include "ctrl_toolpanel.h"

/// UI
#include <ui_terra_toolbar.h>

/// SYSTEM
#include <QMainWindow>
#include <QComboBox>
#include <QPainter>
#include <QDoubleSpinBox>
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
    featu_selection_ = ui->features;
    button_compute_  = ui->compile;
    button_trash_    = ui->trash;
    size_            = ui->sizeBox;
    button_tree_     = ui->showTree;
    button_grid_     = ui->showGrid;
    button_add_      = ui->addBoxes;
    button_mov_      = ui->movBoxes;
    button_sel_      = ui->selBoxes;
    button_del_      = ui->delBoxes;
}

void CtrlToolPanel::sync()
{
    Q_EMIT featuSelected(featu_selection_->currentText());
    if(class_selection_->count() > 1)
        Q_EMIT classSelected(class_selection_->currentIndex());
    Q_EMIT boxSize(size_->value());
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

void CtrlToolPanel::classChanged(int index)
{
    if(index == -1)
        return;

    QVariant v = class_selection_->itemData(index);
    Q_EMIT classSelected(v.toInt());
}

void CtrlToolPanel::featuChanged(int index)
{
    Q_EMIT featuSelected(featu_selection_->currentText());
}

void CtrlToolPanel::buttonMov()
{
        Q_EMIT uncheckMov(true);
        Q_EMIT uncheckAdd(false);
        Q_EMIT uncheckDel(false);
        Q_EMIT uncheckSel(false);
}

void CtrlToolPanel::buttonAdd()
{
        Q_EMIT uncheckAdd(true);
        Q_EMIT uncheckMov(false);
        Q_EMIT uncheckDel(false);
        Q_EMIT uncheckSel(false);
}

void CtrlToolPanel::buttonDel()
{
        Q_EMIT uncheckDel(true);
        Q_EMIT uncheckAdd(false);
        Q_EMIT uncheckMov(false);
        Q_EMIT uncheckSel(false);
}

void CtrlToolPanel::buttonSel()
{
        Q_EMIT uncheckSel(true);
        Q_EMIT uncheckAdd(false);
        Q_EMIT uncheckMov(false);
        Q_EMIT uncheckDel(false);
}

void CtrlToolPanel::buttonComp()
{
    Q_EMIT featuSelected(featu_selection_->currentText());
    Q_EMIT compute();
}

void CtrlToolPanel::trainingFinished()
{
    button_compute_->setEnabled(true);
    button_trash_->setEnabled(true);
}

void CtrlToolPanel::feedbackFinished()
{

}

void CtrlToolPanel::classifierLoaded()
{
    class_selection_->clear();
    std::vector<int> ids = bridge_->getClassIDs();
    foreach(int id, ids) {
        class_selection_->addItem(QIcon(renderColorIcon(id)), QString::number(id), QVariant(id));
    }
}


void CtrlToolPanel::classAdded(int id)
{
    class_selection_->addItem(QIcon(renderColorIcon(id)), QString::number(id), QVariant(id));

    QVariant v = class_selection_->itemData(class_selection_->currentIndex());
    Q_EMIT classSelected(v.toInt());
}

void CtrlToolPanel::classRemoved(int id)
{
    int item_index = class_selection_->findData(QVariant(id));
    if(item_index != -1) {
        class_selection_->removeItem(item_index);
    }
}

void CtrlToolPanel::classUpdated(int oldID, int newID)
{
    int item_index = class_selection_->findData(QVariant(oldID));
    if(item_index != -1) {
        class_selection_->setItemData(item_index, QVariant(newID));
        class_selection_->setItemText(item_index, QString::number(newID));
    }
}

void CtrlToolPanel::colorUpdate(int id)
{
    int item_index = class_selection_->findData(QVariant(id));
    if(item_index != -1) {
        class_selection_->setItemIcon(item_index, QIcon(renderColorIcon(id)));
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
