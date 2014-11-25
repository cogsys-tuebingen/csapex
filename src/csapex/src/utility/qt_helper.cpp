/// HEADER
#include <csapex/utility/qt_helper.hpp>

/// COMPONENT
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/msg/message_factory.h>
#include <csapex/view/port.h>
#include <csapex/command/meta.h>
#include <csapex/utility/assert.h>

/// SYSTEM
#include <iostream>

using namespace qt_helper;
using namespace csapex;

void QSleepThread::sleep(unsigned long t) {
    currentThread()->sleep(t);
}
void QSleepThread::msleep(unsigned long t) {
    currentThread()->msleep(t);
}
void QSleepThread::usleep(unsigned long t) {
    currentThread()->usleep(t);
}

QWidget* QtHelper::wrapLayout(QBoxLayout *l, QWidget *parent)
{
    QWidget *container = new QWidget(parent);
    container->setLayout(l);
    return container;
}

QHBoxLayout* QtHelper::wrap(const std::string& txt, QWidget* widget,
                            csapex::ContextMenuHandler *context_handler) {

    QHBoxLayout* internal_layout = new QHBoxLayout;

    QLabel* label = new QLabel(txt.c_str());
    if(context_handler) {
        label->setContextMenuPolicy(Qt::CustomContextMenu);
        context_handler->setParent(label);
        QObject::connect(label, SIGNAL(customContextMenuRequested(QPoint)), context_handler, SLOT(showContextMenu(QPoint)));
    }
    internal_layout->addWidget(label);
    internal_layout->addWidget(widget);

    return internal_layout;
}

QHBoxLayout* QtHelper::wrap(const std::string& txt, QLayout* layout,
                            csapex::ContextMenuHandler *context_handler ) {

    QHBoxLayout* internal_layout = new QHBoxLayout;

    QLabel* label = new QLabel(txt.c_str());
    if(context_handler) {
        label->setContextMenuPolicy(Qt::CustomContextMenu);
        context_handler->setParent(label);
        QObject::connect(label, SIGNAL(customContextMenuRequested(QPoint)), context_handler, SLOT(showContextMenu(QPoint)));
    }
    internal_layout->addWidget(label);
    internal_layout->addLayout(layout);

    return internal_layout;
}

void QtHelper::clearLayout(QLayout* layout) {
    QLayoutItem* item;
    while((item = layout->takeAt(0)) != NULL) {
        if(item->layout()) {
            clearLayout(item->layout());
        }
        if(item->widget()) {
            item->widget()->deleteLater();
        }
        delete item;
    }

}
