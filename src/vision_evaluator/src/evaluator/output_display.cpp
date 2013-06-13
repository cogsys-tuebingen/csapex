/// HEADER
#include "output_display.h"

/// COMPONENT
#include "registration.hpp"

/// PROJECT
#include <designer/box.h>
#include <designer/connector_in.h>

STATIC_INIT(OutputDisplay, generic, {
                SelectorProxy::ProxyConstructor c;\
                c.setName("Output Display");\
                c.setConstructor(boost::lambda::bind(boost::lambda::new_ptr<SelectorProxyImp<OutputDisplay> >(), \
                boost::lambda::_1, (QWidget*) NULL)); \
                SelectorProxy::registerProxy(c);\
            });

using namespace vision_evaluator;

OutputDisplay::OutputDisplay()
    : input_(NULL), view_(new QGraphicsView)
{
}

void OutputDisplay::fill(QBoxLayout *layout)
{
    if(input_ == NULL) {
        input_ = new ConnectorIn(box_);
        box_->addInput(input_);

        layout->addWidget(view_);
    }
}
