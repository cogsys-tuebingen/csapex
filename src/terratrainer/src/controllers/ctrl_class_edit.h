#ifndef CTRL_CLASS_EDIT_H
#define CTRL_CLASS_EDIT_H

/// COMPONENT
#include <controllers/ctrl_cmpcore_bridge.h>
#include "controller.hpp"

/// SYSTEM
#include <QObject>
#include <QColor>
#include <QIcon>
#include <QPen>
#include <boost/shared_ptr.hpp>
#include <map>

/// DECLARATIONS
class QTableWidget;
class QMainWindow;
class QPushButton;
class QLineEdit;
class QComboBox;
namespace Ui {
class TerraClasses;
}

namespace YAML {
class Emitter;
class Node;
}


class CtrlClassEdit : public QObject, public Controller
{
    Q_OBJECT

public:
    typedef boost::shared_ptr<CtrlClassEdit> Ptr;

    CtrlClassEdit(QMainWindow *class_window, CMPCoreBridge::Ptr bridge);
    virtual ~CtrlClassEdit();

    void setupUI(Ui::TerraClasses *class_content);
    void write(YAML::Emitter &emitter) const;
    void read(const YAML::Node &document);

Q_SIGNALS:
    void enableDel(bool enabled);
    void enableAdd(bool enabled);

public Q_SLOTS:
    void cellClicked(int row, int col);
    void editInfo(QString text);
    void editId(QString id);
    void accept();
    void remove();
    void colorIndex(int index);

protected:
    bool eventFilter(QObject *obj, QEvent *event);

private:
    CMPCoreBridge::Ptr      bridge_;

    QTableWidget           *class_table_;
    QMainWindow            *class_window_;
    QPushButton            *accecpt_button_;
    QLineEdit              *class_ID_;
    QLineEdit              *class_name_;
    QComboBox              *color_combo_;
    std::vector<QColor>     palette_;

    int                     current_row_;
    int                     entered_id_;
    QString                 entered_info_;
    int                     entered_color_;
    int                     selected_id_;

    QPen                    BlackPen;

    /// INTERNAL INTERACTION
    void    loadSelection();
    void    resetEdit();
    void    tableNewEntry(int classID, int color, QString info);
    void    tableSaveEntry();
    bool    uniqueIDCheck();

    /// GRAPHICAL FUNCTIONS
    QPixmap renderColorIcon(int color);
};

#endif // CTRL_CLASS_EDIT_H
