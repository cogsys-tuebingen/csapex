/// HEADER
#include "file_importer.h"

/// COMPONENT
#include "registration.hpp"

/// PROJECT
#include <designer/box.h>
#include <designer/connector_out.h>

/// SYSTEM
#include <QLabel>

STATIC_INIT(FileImporter, generic, {
                SelectorProxy::ProxyConstructor c;\
                c.setName("File Importer");\
                c.setConstructor(boost::lambda::bind(boost::lambda::new_ptr<SelectorProxyImp<FileImporter> >(), \
                    boost::lambda::_1, (QWidget*) NULL)); \
                SelectorProxy::registerProxy(c);\
});

using namespace vision_evaluator;

FileImporter::FileImporter()
    : output_(NULL)
{
}

void FileImporter::fill(QBoxLayout *layout)
{
    if(output_ == NULL) {
        layout->addWidget(new QLabel("File Importer"));
        output_ = new ConnectorOut(box_);
        box_->addOutput(output_);
    }
}
