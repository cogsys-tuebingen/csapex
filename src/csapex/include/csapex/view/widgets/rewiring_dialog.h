#ifndef REWIRING_DIALOG_H
#define REWIRING_DIALOG_H

#include <QDialog>

namespace csapex
{

class RewiringDialog : public QDialog
{
    Q_OBJECT

public:
    RewiringDialog(QWidget *parent = 0, Qt::WindowFlags f = 0);

    std::string getName();

private Q_SLOTS:
    void finish();

private:
    void makeUI();
};

}


#endif // REWIRING_DIALOG_H
