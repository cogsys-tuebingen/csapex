/// HEADER
#include <csapex/view/template_dialog.h>

/// COMPONENT
#include <csapex/manager/template_manager.h>

/// SYSTEM
#include <QIcon>
#include <QVBoxLayout>
#include <QFormLayout>
#include <QDialogButtonBox>

using namespace csapex;

TemplateDialog::TemplateDialog(QWidget *parent, Qt::WindowFlags f)
    : QDialog(parent, f)
{
    makeUI();
}

void TemplateDialog::makeUI()
{
    setWindowIcon(QIcon(":/group.png"));
    setWindowTitle("Template Options");

    style_ = QString("QLineEdit {\n") +
            "  background-color: white;\n" +
            "  border: 1px solid black;\n" +
            "}\n" +
            "\n" +
            "QLineEdit[error=\"true\"] {\n" +
            "  background-color: rgb(230, 50, 50);\n" +
            "  border: 1px solid rgb(180, 0, 0);\n" +
            "}";

    QVBoxLayout* layout = new QVBoxLayout;
    setLayout(layout);

    QFormLayout* form = new QFormLayout;
    layout->addLayout(form);

    form->setRowWrapPolicy(QFormLayout::DontWrapRows);
    form->setFieldGrowthPolicy(QFormLayout::FieldsStayAtSizeHint);
    form->setFormAlignment(Qt::AlignHCenter | Qt::AlignTop);
    form->setLabelAlignment(Qt::AlignLeft);

    name_edit_ = new QLineEdit;
    name_edit_->setStyleSheet(style_);
    form->addRow(tr("&Name:"), name_edit_);
    QObject::connect(name_edit_, SIGNAL(textChanged(QString)), this, SLOT(checkName(QString)));

    QDialogButtonBox* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    layout->addWidget(buttons);
    connect(buttons, SIGNAL(accepted()), this, SLOT(accept()));
    connect(buttons, SIGNAL(rejected()), this, SLOT(reject()));

}

std::string TemplateDialog::getName()
{
    return name_edit_->text().toStdString();
}

void TemplateDialog::checkName(const QString &name)
{
    bool was = name_edit_->property("error").toBool();
    bool exists = TemplateManager::instance().templateExists(name.toStdString());
    bool empty = name.length() == 0;
    bool error = exists || empty;

    if(error != was)  {
        name_edit_->setProperty("error", error);
        if(empty) {
            name_edit_->setToolTip("Please enter a template name");
        } else if(exists) {
            name_edit_->setToolTip(QString("template name <b>") + name + "</b> already exists");
        } else {
            name_edit_->setToolTip("");
        }
        name_edit_->style()->unpolish(name_edit_);
        name_edit_->style()->polish(name_edit_);
    }
}
