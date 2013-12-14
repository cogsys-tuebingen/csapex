/// HEADER
#include <csapex/view/box_dialog.h>

/// COMPONENT
#include <csapex/manager/template_manager.h>

/// SYSTEM
#include <QIcon>
#include <QVBoxLayout>
#include <QFormLayout>
#include <QDialogButtonBox>
#include <QLabel>
#include <QCompleter>
#include <QAbstractItemModel>

using namespace csapex;

BoxDialog::BoxDialog(QWidget *parent, Qt::WindowFlags f)
    : QDialog(parent, f)
{
    makeUI();
}

void BoxDialog::makeUI()
{
    setWindowIcon(QIcon(":/plugin.png"));
    setWindowTitle("Create Box");

    QVBoxLayout* layout = new QVBoxLayout;
    setLayout(layout);

    QLabel *lbl=new QLabel("Type");
    name_edit_ = new QLineEdit;
    lbl->setBuddy(name_edit_);

    QAbstractItemModel* model = BoxManager::instance().listAvailableBoxedObjects();

    QCompleter *completer = new QCompleter(this);
    completer->setCompletionMode(QCompleter::PopupCompletion);
    completer->setCompletionColumn(0);
    completer->setMaxVisibleItems(10);
    completer->setCaseSensitivity(Qt::CaseInsensitive);
    completer->setModel(model);

    name_edit_->setCompleter(completer);

    layout->addWidget(lbl);
    layout->addWidget(name_edit_);

    QDialogButtonBox* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    layout->addWidget(buttons);
    connect(buttons, SIGNAL(accepted()), this, SLOT(accept()));
    connect(buttons, SIGNAL(rejected()), this, SLOT(reject()));

    delete model;
}

std::string BoxDialog::getName()
{
    return name_edit_->text().toStdString();
}

