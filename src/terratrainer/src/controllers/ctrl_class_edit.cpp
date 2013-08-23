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
#include <QColor>
#include <QVariant>

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


    /// EVENT HANDLING
    class_ID_->installEventFilter(this);
    class_name_->installEventFilter(this);

    /// RENDERING
    BlackPen.setColor(Qt::black);
    BlackPen.setWidth(1);

    palette_.push_back(QColor(Qt::yellow));
    palette_.push_back(QColor(Qt::blue));
    palette_.push_back(QColor(Qt::green));
    palette_.push_back(QColor(Qt::red));

    color_combo_->addItem(QIcon(renderColorIcon(0)), "");
    color_combo_->addItem(QIcon(renderColorIcon(1)), "");
    color_combo_->addItem(QIcon(renderColorIcon(2)), "");
    color_combo_->addItem(QIcon(renderColorIcon(3)), "");

    color_combo_->addItem("...");
    current_row_    = -1;
    entered_id_     = -1;
    selected_id_    = -1;
    entered_info_   = "";
    entered_color_  = 0;
}

void CtrlClassEdit::write(YAML::Emitter &emitter) const
{
    emitter << YAML::Key << "CLASSES" << YAML::Value;
    emitter << YAML::BeginSeq;
    for(int i = 0 ; i < class_table_->rowCount() ; i++) {
        int     class_id    = class_table_->item(i, 0)->data(Qt::UserRole).toInt();
        int     class_color = class_table_->item(i, 1)->data(Qt::UserRole).toInt();
        QString class_info  = class_table_->item(i, 2)->text();
        emitter << YAML::BeginMap;
        emitter << YAML::Key << "id"   << YAML::Value << class_id;
        emitter << YAML::Key << "color"<< YAML::Value << class_color;
        emitter << YAML::Key << "info" << YAML::Value << class_info.toUtf8().constData();
        emitter << YAML::EndMap;
    }
    emitter << YAML::EndSeq;
    emitter << YAML::Key << "CLASSES_PALETTE" << YAML::Value;
    emitter << YAML::BeginSeq << YAML::Flow;
    for(std::vector<QColor>::const_iterator it = palette_.begin() ; it != palette_.end() ; it++) {
        emitter << it->red();
        emitter << it->green();
        emitter << it->blue();
    }
    emitter << YAML::EndSeq;
}

void CtrlClassEdit::read(const YAML::Node &document)
{
    while (class_table_->rowCount() > 0)
    {
        class_table_->removeRow(0);
    }
    resetEdit();

    palette_.clear();

    try {
        const YAML::Node &colors = document["CLASSES_PALETTE"];
        int limit = 0;
        for(YAML::Iterator it = colors.begin() ; it != colors.end() && limit < colors.size(); it++, limit++) {
            int r,g,b;
            (*it) >> r; it++;
            (*it) >> g; it++;
            (*it) >> b;
            palette_.push_back(QColor(r, g, b));
        }

        const YAML::Node &data = document["CLASSES"];

        for(YAML::Iterator it = data.begin() ; it != data.end() ; it++) {
            int class_id;
            int color;
            std::string info;

            (*it)["id"] >> class_id;
            (*it)["color"] >> color;
            (*it)["info"]  >> info;

            tableNewEntry(class_id, color, info.c_str());
            bridge_->classAdd(class_id, palette_[color]);
        }
    } catch (YAML::Exception e) {
        std::cerr << "Class Editor cannot read config : '" << e.what() <<"' !" << std::endl;
    }

    resetEdit();
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

void CtrlClassEdit::editInfo(QString text)
{
    entered_info_ = text;
}

void CtrlClassEdit::editId(QString id)
{
    /// RESTRICT TO NUMBERS
    QRegExp rx("[^0-9\\.]");
    id.replace(rx, "");
    class_ID_->setText(id);

    entered_id_ = id.toInt();
}

void CtrlClassEdit::accept()
{
    if(class_ID_->text() == "" || entered_info_ == "" || !uniqueIDCheck()) {
        return;
    }

    if(current_row_ != -1) {
        tableSaveEntry();
    } else {
        tableNewEntry(entered_id_, entered_color_, entered_info_);
        bridge_->classAdd(entered_id_, palette_[entered_color_]);
        resetEdit();
    }
}

void CtrlClassEdit::remove()
{
    if(current_row_ != -1) {
        class_table_->removeRow(current_row_);
        bridge_->classRemove(selected_id_);
        resetEdit();
    }
}

void CtrlClassEdit::colorIndex(int index)
{
    if(index == color_combo_->count() - 1) {
        class_window_->setDisabled(true);
        QColor color = QColorDialog::getColor();
        palette_.push_back(color);
        color_combo_->setItemText(index, "");
        color_combo_->setItemIcon(index, QIcon(renderColorIcon(index)));
        color_combo_->addItem("...");
        class_window_->setEnabled(true);
    }
    entered_color_ = index;
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
    entered_color_ = selection[1]->data(Qt::UserRole).toInt();
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
    entered_color_ = 0;
    Q_EMIT enableDel(false);
    Q_EMIT enableAdd(false);
}

void CtrlClassEdit::tableNewEntry(int classID, int color, QString info)
{
    /// ROW CONTENT
    QTableWidgetItem *id   = new QTableWidgetItem(QString::number(classID));
    QTableWidgetItem *name = new QTableWidgetItem();
    QTableWidgetItem *icon = new QTableWidgetItem();

    id->setData(Qt::UserRole, classID);
    icon->setData(Qt::DecorationRole, renderColorIcon(color).scaled(50, 50, Qt::KeepAspectRatio, Qt::SmoothTransformation));
    icon->setData(Qt::UserRole, color);
    name->setText(info);
    /// INSERT
    class_table_->insertRow(class_table_->rowCount());

    /// SET DATA
    int row = class_table_->rowCount() - 1;
    class_table_->setItem(row, 0, id);
    class_table_->setItem(row, 1, icon);
    class_table_->setItem(row, 2, name);
}

void CtrlClassEdit::tableSaveEntry()
{
    QList<QTableWidgetItem*> selection = class_table_->selectedItems();

    /// ID
    selection[0]->setText(QString::number(entered_id_));
    /// COLOR
    selection[1]->setData(Qt::DecorationRole, renderColorIcon(entered_color_).scaled(50, 50, Qt::KeepAspectRatio, Qt::SmoothTransformation));
    /// NAME
    selection[2]->setText(entered_info_);

    if(entered_id_ != selected_id_) {
        bridge_->classUpdate(selected_id_, entered_id_);
        selected_id_ = entered_id_;
    }
    if(palette_[entered_color_] != bridge_->colorGet(entered_id_))
        bridge_->colorUpdate(selected_id_, palette_[entered_color_]);
}

bool CtrlClassEdit::uniqueIDCheck()
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

QPixmap CtrlClassEdit::renderColorIcon(int color)
{
    QPixmap  pixmap(60,20);
    QPainter paintr(&pixmap);
    paintr.setBrush(palette_[color]);
    paintr.setPen(BlackPen);
    paintr.drawRect(0,0,59,19);
    return pixmap;
}
