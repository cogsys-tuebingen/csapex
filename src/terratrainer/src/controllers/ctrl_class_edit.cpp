/// HEADER
#include "ctrl_class_edit.h"

/// COMPONENT
#include <ui_terra_classes_window.h>

/// SYSTEM
#include <QTableWidgetItem>
#include <QAbstractTableModel>
#include <QMainWindow>
#include <QPainter>
#include <QLineEdit>
#include <QPushButton>
#include <QColorDialog>

CtrlClassEdit::CtrlClassEdit(QMainWindow *class_window,  CMPCoreBridge::Ptr bridge) :
    class_window_(class_window),
    bridge_(bridge)
{
}

CtrlClassEdit::~CtrlClassEdit()
{
}

void CtrlClassEdit::setupUI(Ui::TerraClasses *class_content)
{
    /// REQUIRED GUI ELEMENTS
    class_table_    = class_content->tableClasses;
    accecpt_button_ = class_content->addClass;
    class_ID_       = class_content->classID;
    class_name_     = class_content->className;
    color_combo_    = class_content->classColor;
    current_row_    = -1;
    entered_id_     = -1;
    selected_id_    = -1;
    entered_info_    = "";

    /// EVENT HANDLING
    class_ID_->installEventFilter(this);
    class_name_->installEventFilter(this);

    /// RENDERING
    BlackPen.setColor(Qt::black);
    BlackPen.setWidth(1);

    bridge_->extendPallete(QColor(Qt::yellow));
    bridge_->extendPallete(QColor(Qt::blue));
    bridge_->extendPallete(QColor(Qt::green));
    bridge_->extendPallete(QColor(Qt::red));
    color_combo_->addItem(QIcon(renderColorIcon(0)), "");
    color_combo_->addItem(QIcon(renderColorIcon(1)), "");
    color_combo_->addItem(QIcon(renderColorIcon(2)), "");
    color_combo_->addItem(QIcon(renderColorIcon(3)), "");

    color_combo_->addItem("...");
    colorIndex(0);
}

void CtrlClassEdit::cellClicked(int row, int col)
{
    if(row != current_row_) {
        current_row_ = row;
        loadSelection();
        accecpt_button_->setIcon(QIcon(":/buttons/checkmark.png"));
        Q_EMIT enableDel(true);
        Q_EMIT enableAdd(true);
    } else {
        resetEdit();
    }
}

void CtrlClassEdit::nameEdited(QString text)
{
    entered_info_ = text;
}

void CtrlClassEdit::IDEdited(QString id)
{
    /// RESTRICT TO NUMBERS
    QRegExp rx("[^0-9\\.]");
    id.replace(rx, "");
    class_ID_->setText(id);

    entered_id_ = id.toInt();
}

void CtrlClassEdit::accept()
{
    if(class_ID_->text() == "" || entered_info_ == "" || !uniqueID()) {
        return;
    }

    if(current_row_ != -1) {
        saveEntry();
    } else {
        newEntry();
        resetEdit();
    }
}

void CtrlClassEdit::remove()
{
    if(current_row_ != -1) {
        class_table_->removeRow(current_row_);
        bridge_->removeClass(selected_id_);
        resetEdit();
    }
}

void CtrlClassEdit::colorIndex(int index)
{
    if(index == color_combo_->count() - 1) {
        class_window_->setDisabled(true);
        bridge_->extendPallete(QColorDialog::getColor());
        color_combo_->setItemText(index, "");
        color_combo_->setItemIcon(index, QIcon(renderColorIcon(index)));
        color_combo_->addItem("...");
        class_window_->setEnabled(true);
    }
    entered_color_ = index;
}

void CtrlClassEdit::classesLoaded()
{
    /// CLEAN UP
    while (class_table_->rowCount() > 0)
    {
        class_table_->removeRow(0);
    }
    resetEdit();

    /// LOAD
    std::vector<int> classes = bridge_->getClassIDs();
    for(std::vector<int>::iterator it = classes.begin() ; it != classes.end() ; it++) {
        newTableEntry(*it, bridge_->getColorID(*it), bridge_->getInfo(*it));
    }
}

bool CtrlClassEdit::eventFilter(QObject *obj, QEvent *event)
{
    if(event->type() == QEvent::FocusIn && entered_id_ == -1 && entered_info_ == "") {
        event->accept();
        class_ID_->setText("");
        class_name_->setText("");
        Q_EMIT enableAdd(true);
    }
    if(event->type() == QEvent::FocusOut && entered_id_ == -1 && entered_info_ == "") {
        event->accept();
        class_ID_->setText("ID");
        class_name_->setText("NAME");
        Q_EMIT enableAdd(false);
    }

    return QObject::eventFilter(obj, event);
}

void CtrlClassEdit::loadSelection()
{
    QList<QTableWidgetItem*> selection = class_table_->selectedItems();
    selected_id_   = selection[0]->text().toInt();
    entered_id_    = selected_id_;
    entered_color_ = bridge_->getColorID(entered_id_);
    entered_info_  = selection[2]->text();
    color_combo_->setCurrentIndex(entered_color_);

    class_ID_->setText(QString::number(entered_id_));
    class_name_->setText(entered_info_);

}

void CtrlClassEdit::resetEdit()
{
    current_row_  = -1;
    entered_info_ = "";
    entered_id_   = -1;
    selected_id_  = -1;
    accecpt_button_->setIcon(QIcon(":/buttons/plus.png"));
    class_ID_->setText("ID");
    class_name_->setText("NAME");
    class_table_->clearSelection();
    color_combo_->setCurrentIndex(0);
    colorIndex(0);
    Q_EMIT enableDel(false);
    Q_EMIT enableAdd(false);
}

void CtrlClassEdit::newTableEntry(int classID, int color, QString info)
{
    /// ROW CONTENT
    QTableWidgetItem *id   = new QTableWidgetItem(QString::number(classID));
    QTableWidgetItem *name = new QTableWidgetItem();
    QTableWidgetItem *icon = new QTableWidgetItem();

    icon->setData(Qt::DecorationRole, renderColorIcon(color).scaled(50, 50, Qt::KeepAspectRatio, Qt::SmoothTransformation));
    name->setText(info);
    /// INSERT
    class_table_->insertRow(class_table_->rowCount());

    /// SET DATA
    int row = class_table_->rowCount() - 1;
    class_table_->setItem(row, 0, id);
    class_table_->setItem(row, 1, icon);
    class_table_->setItem(row, 2, name);
}

void CtrlClassEdit::saveEntry()
{
    QList<QTableWidgetItem*> selection = class_table_->selectedItems();

    /// ID
    selection[0]->setText(QString::number(entered_id_));
    /// COLOR
    selection[1]->setData(Qt::DecorationRole, renderColorIcon(entered_color_).scaled(50, 50, Qt::KeepAspectRatio, Qt::SmoothTransformation));
    /// NAME
    selection[2]->setText(entered_info_);

    if(entered_id_ != selected_id_) {
        bridge_->updateClass(selected_id_, entered_id_);
        selected_id_ = entered_id_;
    }
    if(entered_color_ != bridge_->getColorID(entered_id_))
        bridge_->updateColor(selected_id_, entered_color_);

    if(entered_info_ != "" && entered_info_ != bridge_->getInfo(entered_id_))
        bridge_->updateInfo(entered_id_, entered_info_);
}

void CtrlClassEdit::newEntry()
{
    newTableEntry(entered_id_, entered_color_, entered_info_);
    bridge_->addClass(entered_id_, entered_color_);
    bridge_->addInfo(entered_id_, entered_info_);
}

bool CtrlClassEdit::uniqueID()
{
    if(selected_id_ == entered_id_)
        return true;

    bool unique = true;
    unique &= entered_id_ != -1;

    int row_count = class_table_->rowCount();
    for(int i = 0 ; i < row_count ; i++) {
        QTableWidgetItem *item = class_table_->item(i, 0);
        unique &= entered_id_ != item->text().toInt();
    }
    return unique;
}

QPixmap CtrlClassEdit::renderColorIcon(const int pal_index)
{
    QColor   color = bridge_->getColor(pal_index);
    QPixmap  pixmap(60,20);
    QPainter paintr(&pixmap);
    paintr.setBrush(color);
    paintr.setPen(BlackPen);
    paintr.drawRect(0,0,59,19);
    return pixmap;
}
