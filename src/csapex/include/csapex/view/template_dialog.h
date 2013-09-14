#ifndef TEMPLATE_DIALOG_H
#define TEMPLATE_DIALOG_H

/// SYSTEM
#include <QDialog>
#include <QLineEdit>

namespace csapex
{

class TemplateDialog : public QDialog
{
    Q_OBJECT

public:
    TemplateDialog(QWidget *parent = 0, Qt::WindowFlags f = 0);

    std::string getName();

private Q_SLOTS:
    void checkName(const QString& name_edit_);

private:
    void makeUI();

private:
    QString style_;
    QLineEdit * name_edit_;
};

}

#endif // TEMPLATE_DIALOG_H
