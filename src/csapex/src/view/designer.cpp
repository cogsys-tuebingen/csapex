/// HEADER
#include <csapex/view/designer.h>

/// COMPONENT
#include <csapex/command/dispatcher.h>
#include <csapex/core/drag_io.h>
#include <csapex/core/settings.h>
#include <csapex/manager/box_manager.h>
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex/model/node.h>
#include <csapex/utility/qt_helper.hpp>
#include <csapex/view/box.h>
#include <csapex/view/design_board.h>
#include <csapex/view/widget_controller.h>
#include "ui_designer.h"
#include <utils_param/parameter_factory.h>

/// SYSTEM
#include <QScrollBar>

using namespace csapex;

Designer::Designer(Settings& settings, Graph::Ptr graph, CommandDispatcher *dispatcher, WidgetControllerPtr widget_ctrl, DesignBoard* board, QWidget* parent)
    : QWidget(parent), ui(new Ui::Designer), settings_(settings), graph_(graph), dispatcher_(dispatcher), widget_ctrl_(widget_ctrl), box_selection_menu(NULL), is_init_(false)
{
    ui->setupUi(this);

    ui->splitter->setStretchFactor(0, 0);
    ui->splitter->setStretchFactor(1, 1);

    designer_board = board;
    ui->scrollArea->setWidget(designer_board);

    if(settings_.knows("grid")) {
        enableGrid(settings_.get<bool>("grid"));
    }

    if(settings_.knows("grid-lock")) {
        lockToGrid(settings_.get<bool>("grid-lock"));
    }

    QObject::connect(dispatcher, SIGNAL(stateChanged()), this, SLOT(updateUndoInfo()));
}

Designer::~Designer()
{
}

void Designer::deleteSelected()
{
    command::Meta::Ptr del(new command::Meta("delete selected"));

    if(widget_ctrl_->box_selection_.countSelected() != 0) {
        del->add(widget_ctrl_->box_selection_.deleteSelectedCommand());
    }
    if(widget_ctrl_->connection_selection_.countSelected() != 0) {
        del->add(widget_ctrl_->connection_selection_.deleteSelectedCommand());
    }

    if(del->commands() != 0) {
        dispatcher_->execute(del);
    }
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
    if(box_selection_menu == NULL) {
        reloadBoxMenues();
    }

    if(!is_init_) {
        QList<int> sizes = ui->splitter->sizes();

        if(sizes[0] > 0 && sizes[1] > 0) {
            is_init_ = true;
            int w = 0;
            sizes[1] = sizes[1] + (sizes[0] - w);
            sizes[0] = w;
            ui->splitter->setSizes(sizes);
        }
    }
}

void Designer::setView(int sx, int sy)
{
    designer_board->setView(sx, sy);
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
    if(!debug_tabs->isVisible()) {
        return;
    }

    std::vector<Box*> selected;
    boost::function<void(Box*)> append = boost::bind(&std::vector<Box*>::push_back, &selected, _1);
    widget_ctrl_->foreachBox(append, boost::bind(&Box::isSelected, _1));

    box_info->clear();

    foreach (Box* box, selected) {
        Node* node = box->getNode();
        QObject::connect(node, SIGNAL(stateChanged()), this, SLOT(updateDebugInfo()));
        QObject::connect(node, SIGNAL(modelChanged()), this, SLOT(updateDebugInfo()));
        box_info->addTopLevelItem(node->createDebugInformation());
    }

    QTreeWidgetItemIterator it(box_info);
    while (*it) {
        QTreeWidgetItem* item = *it;
        bool expand = item->data(0, Qt::UserRole).toBool();

        int depth = 0;
        while(item->parent()) {
            item = item->parent();
            ++depth;
        }

        if(depth <= 1 || expand) {
            QTreeWidgetItem* item = *it;
            do {
                box_info->expandItem(item);
                item = item->parent();
            }  while(item);
        }
        ++it;
    }

    for(int i = 0; i < box_info->depth(); ++i) {
        box_info->resizeColumnToContents(i);
    }
}

void Designer::updateUndoInfo()
{
    undo_stack->clear();
    redo_stack->clear();

    dispatcher_->populateDebugInfo(undo_stack, redo_stack);

    undo_stack->expandAll();
    redo_stack->expandAll();
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

    box_selection_menu = new QTreeWidget;
    box_selection_menu->setHeaderLabel("Type");
    box_selection_menu->setColumnCount(1);
    BoxManager::instance().insertAvailableNodeTypes(box_selection_menu);
    ui->boxes->layout()->addWidget(box_selection_menu);

    box_info = new QTreeWidget;
    QStringList box_info_labels;
    box_info_labels.append("Property");
    box_info_labels.append("Value");
    box_info->setHeaderLabels(box_info_labels);
    box_info->setColumnCount(2);

    undo_stack = new QTreeWidget;
    QStringList undo_labels;
    undo_labels.append("Command");
    undo_labels.append("Type");
    undo_stack->setHeaderLabels(undo_labels);
    undo_stack->setColumnCount(2);

    redo_stack = new QTreeWidget;
    QStringList redo_labels;
    redo_labels.append("Command");
    redo_labels.append("Type");
    redo_stack->setHeaderLabels(redo_labels);
    redo_stack->setColumnCount(2);

    QWidget* undo_redo = new QWidget;
    undo_redo->setLayout(new QVBoxLayout);
    undo_redo->layout()->addWidget(new QLabel("Undo stack"));
    undo_redo->layout()->addWidget(undo_stack);
    undo_redo->layout()->addWidget(new QLabel("Redo stack"));
    undo_redo->layout()->addWidget(redo_stack);

    debug_tabs = new QTabWidget;
    debug_tabs->addTab(box_info, "Box Information");
    debug_tabs->addTab(undo_redo, "Undo/Redo Stack");
    ui->debug->layout()->addWidget(debug_tabs);

    designer_board->setFocus();
}

bool Designer::isGridEnabled() const
{
    return settings_.get<bool>("grid", false);
}

bool Designer::isGridLockEnabled() const
{
    return settings_.get<bool>("grid-lock", false);
}

void Designer::enableGrid(bool grid)
{
    if(!settings_.knows("grid")) {
        settings_.add(param::ParameterFactory::declareBool("grid", grid));
    }

    settings_.set("grid", grid);

    designer_board->enableGrid(grid);
    Q_EMIT gridEnabled(grid);

}

void Designer::lockToGrid(bool lock)
{
    if(!settings_.knows("grid-lock")) {
        settings_.add(param::ParameterFactory::declareBool("grid-lock", lock));
    }

    settings_.set("grid-lock", lock);

    DragIO::lock = lock;
    Q_EMIT gridLockEnabled(lock);
}
