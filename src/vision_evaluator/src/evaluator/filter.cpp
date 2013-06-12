/// HEADER
#include "filter.h"

/// PROJECT
#include <designer/connector_in.h>
#include <designer/connector_out.h>
#include <designer/box.h>

/// SYSTEM
#include <QLabel>

using namespace vision_evaluator;


Filter::Filter()
    : input_(NULL), output_(NULL)
{
    setName("unnamed filter");
}

Filter::~Filter()
{
}

void Filter::fill(QBoxLayout *parent)
{
    if(input_ == NULL) {
//        parent->addWidget(new QLabel("Filter Filter!!!11eins"));

        input_ = new ConnectorIn(box_);
        box_->addInput(input_);

        output_ = new ConnectorOut(box_);
        box_->addOutput(output_);

        insert(parent);
    }
}
