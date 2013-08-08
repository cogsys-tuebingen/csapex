#ifndef CTRL_CLASS_EDIT_H
#define CTRL_CLASS_EDIT_H

/// COMPONENT
#include <controllers/ctrl_cmpcore_bridge.h>

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


class CtrlClassEdit : public QObject
{
    Q_OBJECT

public:
    typedef boost::shared_ptr<CtrlClassEdit> Ptr;

    CtrlClassEdit(QMainWindow *class_window, CMPCoreBridge::Ptr bridge);
    virtual ~CtrlClassEdit();

    void setupUI(Ui::TerraClasses *class_content);

Q_SIGNALS:
    void enableDel(bool enabled);
    void enableAdd(bool enabled);

public Q_SLOTS:
    void cellClicked(int row, int col);
    void nameEdited(QString text);
    void IDEdited(QString id);
    void accept();
    void remove();
    void colorIndex(int index);
    void classesLoaded();

protected:
    bool eventFilter(QObject *obj, QEvent *event);

private:
    CMPCoreBridge::Ptr  bridge_;

    QTableWidget           *class_table_;
    QMainWindow            *class_window_;
    QPushButton            *accecpt_button_;
    QLineEdit              *class_ID_;
    QLineEdit              *class_name_;
    QComboBox              *color_combo_;

    int                     current_row_;
    int                     entered_id_;
    QString                 entered_info_;
    int                     entered_color_;
    int                     selected_id_;

    QPen                    BlackPen;

    /// INTERNAL INTERACTION
    void    loadSelection();
    void    resetEdit();
    void    newTableEntry(int classID, int color, QString info);
    void    saveEntry();
    void    newEntry();
    bool    uniqueID();

    /// GRAPHICAL FUNCTIONS
    QPixmap renderColorIcon(const int pal_index);
};

#endif // CTRL_CLASS_EDIT_H
