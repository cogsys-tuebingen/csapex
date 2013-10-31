/// HEADER
#include <csapex/view/designer.h>

/// COMPONENT
#include <csapex/model/node.h>
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex/view/box.h>
#include <csapex/command/dispatcher.h>
#include <csapex/view/design_board.h>
#include <csapex/manager/box_manager.h>
#include "ui_designer.h"
#include <csapex/utility/qt_helper.hpp>

using namespace csapex;

Designer::Designer(CommandDispatcher *dispatcher, QWidget* parent)
    : QWidget(parent), ui(new Ui::Designer), dispatcher_(dispatcher), menu(NULL)
{
    ui->setupUi(this);

    ui->splitter->setStretchFactor(0, 0);
    ui->splitter->setStretchFactor(1, 1);

    designer_board = new DesignBoard(dispatcher);
    ui->scrollArea->setWidget(designer_board);
}

Designer::~Designer()
{
}

bool Designer::eventFilter(QObject*, QEvent*)
{
    return false;
}

void Designer::keyPressEvent(QKeyEvent* e)
{
    designer_board->keyPressEvent(e);
}

void Designer::keyReleaseEvent(QKeyEvent* e)
{
    designer_board->keyReleaseEvent(e);
}

void Designer::resizeEvent(QResizeEvent*)
{
    if(menu == NULL) {
        reloadBoxMenues();
    }
}

void Designer::reset()
{
    designer_board->reset();
}

void Designer::addBox(Box *box)
{
    designer_board->addBoxEvent(box);
}

void Designer::removeBox(Box *box)
{
    designer_board->removeBoxEvent(box);
}


void Designer::stateChangedEvent()
{
    designer_board->refresh();
}

void Designer::updateDebugInfo()
{
    Graph::Ptr graph = dispatcher_->getGraph();
    std::vector<Box*> selected;
    boost::function<void(Box*)> append = boost::bind(&std::vector<Box*>::push_back, &selected, _1);
    graph->foreachBox(append, boost::bind(&Box::isSelected, _1));

    debug->clear();

    foreach (Box* box, selected) {
        Node* node = box->getNode();
        QObject::connect(node, SIGNAL(stateChanged()), this, SLOT(updateDebugInfo()));
        debug->addTopLevelItem(node->createDebugInformation());
    }

    debug->expandAll();

}

void Designer::reloadBoxMenues()
{
    if(ui->boxes->layout()) {
        QtHelper::clearLayout(ui->boxes->layout());
        QtHelper::clearLayout(ui->debug->layout());
    } else {
        ui->boxes->setLayout(new QVBoxLayout);
        ui->debug->setLayout(new QVBoxLayout);
    }

    menu = new QTreeWidget;
    menu->setHeaderLabel("Type");
    menu->setColumnCount(1);
    BoxManager::instance().insertAvailableBoxedObjects(menu);
    ui->boxes->layout()->addWidget(menu);


    debug = new QTreeWidget();
    QStringList labels;
    labels.append("Property");
    labels.append("Value");
    debug->setHeaderLabels(labels);
    debug->setColumnCount(2);
    ui->debug->layout()->addWidget(debug);

    Graph::Ptr graph = dispatcher_->getGraph();
    QObject::connect(graph.get(), SIGNAL(selectionChanged()), this, SLOT(updateDebugInfo()));
}

void Designer::enableGrid(bool grid)
{
    designer_board->enableGrid(grid);
}
