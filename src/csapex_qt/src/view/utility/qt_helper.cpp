/// HEADER
#include <csapex/view/utility/qt_helper.hpp>

/// COMPONENT
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/factory/message_factory.h>
#include <csapex/command/meta.h>
#include <csapex/utility/assert.h>
#include <csapex/param/parameter.h>

/// SYSTEM
#include <iostream>
#include <QLabel>

using namespace qt_helper;
using namespace csapex;

QWidget* QtHelper::wrapLayout(QBoxLayout *l, QWidget *parent)
{
    QWidget *container = new QWidget(parent);
    container->setLayout(l);
    return container;
}

QHBoxLayout* QtHelper::wrap(const std::string& txt, QWidget* widget,
                            csapex::ContextMenuHandler *context_handler,
                            csapex::param::Parameter* p) {

    QHBoxLayout* internal_layout = new QHBoxLayout;

    QLabel* label = new QLabel(txt.c_str());
    if(context_handler) {
        label->setContextMenuPolicy(Qt::CustomContextMenu);
        context_handler->setParent(label);
        QObject::connect(label, &QLabel::customContextMenuRequested, [label, context_handler](QPoint local){
            context_handler->showContextMenu(label, local);
        });
    }
    internal_layout->addWidget(label);
    internal_layout->addWidget(widget);

    if(p) {
        for(int i = 0; i < internal_layout->count(); ++i) {
            QWidget* child = internal_layout->itemAt(i)->widget();
            child->setProperty("parameter", QVariant::fromValue(static_cast<void*>(static_cast<csapex::param::Parameter*>(p))));
        }
    }

    return internal_layout;
}

QHBoxLayout* QtHelper::wrap(const std::string& txt, QLayout* layout,
                            csapex::ContextMenuHandler *context_handler,
                            csapex::param::Parameter* p) {

    QHBoxLayout* internal_layout = new QHBoxLayout;

    QLabel* label = nullptr;
    if(p && !p->description().empty()) {
        label = new QLabel(QString::fromStdString(txt) + "<img src=':/help.png' />");
    } else {
        label = new QLabel(QString::fromStdString(txt));
    }
    if(context_handler) {
        label->setContextMenuPolicy(Qt::CustomContextMenu);
        context_handler->setParent(label);
        QObject::connect(label, &QLabel::customContextMenuRequested, [label, context_handler](QPoint local){
            context_handler->showContextMenu(label, local);
        });
    }
    internal_layout->addWidget(label);
    internal_layout->addLayout(layout);

    if(p) {
        for(int i = 0; i < internal_layout->count(); ++i) {
            QWidget* child = internal_layout->itemAt(i)->widget();
            if(child) {
                child->setProperty("parameter", QVariant::fromValue(static_cast<void*>(static_cast<csapex::param::Parameter*>(p))));
            }
        }
    }

    return internal_layout;
}

void QtHelper::clearLayout(QLayout* layout) {
    if(!layout) {
        return;
    }
    QLayoutItem* item;
    while((item = layout->takeAt(0)) != nullptr) {
        if(item->layout()) {
            clearLayout(item->layout());
        }
        if(item->widget()) {
            item->widget()->deleteLater();
        }
        delete item;
    }

}
