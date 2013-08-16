/// PROJECT
#include <csapex/csapex_testsuite.h>

/// SYSTEM
#include <QApplication>

using namespace csapex;

int main(int argc, char** argv)
{
    QApplication app(argc, argv);

    Box* box_ = new Box(new Dummy);

    ConnectorOut* output_ = new ConnectorOut(box_, 0);
    output_->setLabel("Unknown");

    box_->addOutput(output_);

    delete box_;
}


