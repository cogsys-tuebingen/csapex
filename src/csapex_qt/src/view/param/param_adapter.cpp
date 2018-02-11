/// HEADER
#include <csapex/view/param/param_adapter.h>

/// PROJECT
#include <csapex/utility/assert.h>
#include <csapex/view/node/parameter_context_menu.h>

/// SYSTEM
#include <QApplication>
#include <QThread>

using namespace csapex;

namespace {
void assertGuiThread()
{
    apex_assert_hard(QThread::currentThread() == QApplication::instance()->thread());
}

void assertNotGuiThread()
{
    apex_assert_hard(QThread::currentThread() != QApplication::instance()->thread());
}
}

ParameterAdapter::ParameterAdapter(param::Parameter::Ptr p)
    : p_(p)
{
    apex_assert_hard(p);

    qRegisterMetaType<std::function<void()>>("std::function<void()>");

    QObject::connect(this, &ParameterAdapter::modelCallback,
                     this, &ParameterAdapter::executeModelCallback,
                     Qt::QueuedConnection);

    context_handler = new ParameterContextMenu(p);
    QObject::connect(this, &ParameterAdapter::customContextMenuRequested,
                     context_handler, &ParameterContextMenu::showContextMenu);

}

ParameterAdapter::~ParameterAdapter()
{
}

void ParameterAdapter::doSetup(QBoxLayout *layout, const std::string &display_name)
{
    QWidget* main_widget = setup(layout, display_name);

    if(context_handler) {
        setupContextMenu(context_handler);
        if(main_widget) {
            context_handler->setParent(main_widget);
        }
    }
}

void ParameterAdapter::executeModelCallback(std::function<void()> cb)
{
    assertGuiThread();
    cb();
}

void ParameterAdapter::setupContextMenu(ParameterContextMenu */*context_handler*/)
{

}

void ParameterAdapter::disconnect()
{
    connections.clear();
}

/// MOC
#include "../../../include/csapex/view/param/moc_param_adapter.cpp"
