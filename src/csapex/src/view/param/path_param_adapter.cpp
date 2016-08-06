/// HEADER
#include <csapex/view/param/path_param_adapter.h>

/// PROJECT
#include <csapex/view/utility/qwrapper.h>
#include <csapex/view/node/parameter_context_menu.h>
#include <csapex/view/utility/qt_helper.hpp>
#include <csapex/utility/assert.h>
#include <csapex/utility/type.h>
#include <csapex/command/update_parameter.h>

/// SYSTEM
#include <QPointer>
#include <QBoxLayout>
#include <QFileDialog>
#include <QPushButton>
#include <QLabel>
#include <QLineEdit>
#include <iostream>

using namespace csapex;

PathParameterAdapter::PathParameterAdapter(param::PathParameter::Ptr p)
    : ParameterAdapter(std::dynamic_pointer_cast<param::Parameter>(p)), path_p_(p)
{

}

QWidget* PathParameterAdapter::setup(QBoxLayout* layout, const std::string& display_name)
{
    QPointer<QLineEdit> path = new QLineEdit(path_p_->as<std::string>().c_str());
    QPointer<QPushButton> select = new QPushButton("select");

    QHBoxLayout* sub = new QHBoxLayout;

    sub->addWidget(path);
    sub->addWidget(select);

    layout->addLayout(QtHelper::wrap(display_name, sub, context_handler));

    // ui change -> model
    QObject::connect(path.data(), &QLineEdit::returnPressed, [this, path]() {
        if(!path_p_ || !path) {
            return;
        }
        auto path_str = path->text().toStdString();
        command::UpdateParameter::Ptr update_parameter = std::make_shared<command::UpdateParameter>(AUUID(p_->getUUID()), path_str);
        executeCommand(update_parameter);
    });

    QObject::connect(select.data(), &QPushButton::clicked, [this, path](){
        if(!path_p_) {
            return;
        }

        QString filter = QString::fromStdString(path_p_->filter());
        if(filter.isEmpty()) {
            filter = "All files (*.*)";
        }

        int flags = QFileDialog::DontUseNativeDialog;
        bool is_file = path_p_->isFile();

        QString dir(path_p_->as<std::string>().c_str());
        if(dir.startsWith("file://", Qt::CaseInsensitive)) {
            dir = dir.replace("file://", "", Qt::CaseInsensitive);
        }

        QString path;
        if(path_p_->isOutput()) {
            if(is_file) {
                path = QFileDialog::getSaveFileName((QWidget*) 0, path_p_->name().c_str(), dir, filter, (QString*) 0, (QFlags<QFileDialog::Option>) flags);
            } else {
                path = QFileDialog::getExistingDirectory((QWidget*) 0, path_p_->name().c_str(), dir, (QFlags<QFileDialog::Option>) flags);
            }
        } else {
            if(is_file) {
                path = QFileDialog::getOpenFileName((QWidget*) 0, path_p_->name().c_str(), dir, filter, (QString*) 0, (QFlags<QFileDialog::Option>) flags);
            } else {
                path = QFileDialog::getExistingDirectory((QWidget*) 0, path_p_->name().c_str(), dir, (QFlags<QFileDialog::Option>) flags);
            }
        }

        if(!path.isEmpty()) {
            path_p_->set(path.toStdString());
        }
    });

    // model change -> ui
    connectInGuiThread(p_->parameter_changed, [this, path](param::Parameter*){
        if(!path_p_ || !path) {
            return;
        }
        path->setText(QString::fromStdString(path_p_->as<std::string>()));
    });

    return nullptr;
}
