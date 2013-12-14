#ifndef BOX_DIALOG_H
#define BOX_DIALOG_H

/// SYSTEM
#include <QDialog>
#include <QLineEdit>

namespace csapex
{

class BoxDialog : public QDialog
{
    Q_OBJECT

public:
    BoxDialog(QWidget *parent = 0, Qt::WindowFlags f = 0);

    std::string getName();

private:
    void makeUI();

private:
    QLineEdit * name_edit_;
};

}

#endif // BOX_DIALOG_H
