/// HEADER
#include <csapex/view/designer/graph_view_context_menu.h>

/// COMPONENT
#include <csapex/core/csapex_core.h>
#include <csapex/model/node_worker.h>
#include <csapex/model/node_state.h>
#include <csapex/model/subgraph_node.h>
#include <csapex/model/graph_facade.h>
#include <csapex/scheduling/thread_group.h>
#include <csapex/scheduling/thread_pool.h>
#include <csapex/view/designer/graph_view.h>
#include <csapex/view/utility/clipboard.h>
#include <csapex/view/utility/snippet_list_generator.h>
#include <csapex/view/utility/node_list_generator.h>
#include <csapex/view/csapex_view_core.h>
#include <csapex/view/node/box.h>
#include <csapex/view/node/note_box.h>
#include <csapex/view/utility/context_menu_handler.h>

/// SYSTEM
#include <sstream>
#include <QMenu>

using namespace csapex;

GraphViewContextMenu::GraphViewContextMenu(GraphView &view)
    : view_(view)
{

}

void GraphViewContextMenu::showGlobalMenu(const QPoint& global_pos)
{
    QMenu menu;

    QAction q_copy("copy", &menu);
    q_copy.setIcon(QIcon(":/copy.png"));
    q_copy.setEnabled(!view_.getSelectedBoxes().empty());
    menu.addAction(&q_copy);

    QAction q_paste("paste", &menu);
    q_paste.setIcon(QIcon(":/paste.png"));
    q_paste.setEnabled(ClipBoard::canPaste());
    menu.addAction(&q_paste);

    menu.addSeparator();

    QAction add_note("create sticky note", &menu);
    add_note.setIcon(QIcon(":/note.png"));
    menu.addAction(&add_note);

    QAction add_subgraph("create subgraph", &menu);
    add_subgraph.setIcon(QIcon(":/group.png"));
    menu.addAction(&add_subgraph);

    QMenu add_node("create node");
    add_node.setIcon(QIcon(":/plugin.png"));
    NodeListGenerator node_generator(view_.core_.getNodeFactory(), *view_.view_core_.getNodeAdapterFactory());
    node_generator.insertAvailableNodeTypes(&add_node);
    menu.addMenu(&add_node);

    QMenu add_snippet("add snippet");
    add_snippet.setIcon(QIcon(":/snippet.png"));
    SnippetListGenerator snippet_generator(view_.core_.getSnippetFactory());
    snippet_generator.insertAvailableSnippets(&add_snippet);
    menu.addMenu(&add_snippet);

    QAction* selectedItem = menu.exec(global_pos);

    if(selectedItem) {
        if(selectedItem == &q_copy) {
            view_.copySelected();

        } else if(selectedItem == &q_paste) {
            view_.paste();

        } else if(selectedItem == &add_note) {
            view_.startPlacingBox("csapex::Note", nullptr);

        } else if(selectedItem == &add_subgraph) {
            view_.startPlacingBox("csapex::Graph", nullptr);

        } else {
            // else it must be an insertion
            auto data = selectedItem->data().value<QPair<QString, QString>>();
            std::string mime = data.first.toStdString();
            std::string type = data.second.toStdString();

            view_.createNodes(global_pos, type, mime);
        }
    }
}


void GraphViewContextMenu::showSelectionMenu(const QPoint& global_pos)
{
    std::stringstream title;
    if(view_.selected_boxes_.size() == 1) {
        title << "Node: " << view_.selected_boxes_.front()->getNodeWorker()->getUUID().getShortName();
    } else {
        title << view_.selected_boxes_.size() << " selected nodes";
    }

    QMenu menu;
    std::map<QAction*, std::function<void()> > handler;

    ContextMenuHandler::addHeader(menu, title.str());


    QAction* copy = new QAction("copy", &menu);
    copy->setIcon(QIcon(":/copy.png"));
    copy->setIconVisibleInMenu(true);
    handler[copy] = std::bind(&GraphView::copySelected, &view_);
    menu.addAction(copy);

    QAction* paste = new QAction("paste", &menu);
    paste->setIcon(QIcon(":/paste.png"));
    paste->setEnabled(false);
    menu.addAction(paste);

    menu.addSeparator();

    bool has_minimized = false;
    bool has_maximized = false;
    bool has_muted = false;
    bool has_unmuted = false;
    bool has_enabled = false;
    bool has_disabled = false;
    bool has_box = false;
    bool has_note = false;
    bool has_pipeline = false;
    bool has_sequential = false;

    std::map<int, bool> has_log_level;
    for(int i = 0; i <=3; ++i) {
        has_log_level[i] = false;
    }

    for(NodeBox* box : view_.selected_boxes_) {
        bool minimized = box->isMinimizedSize();
        has_minimized |= minimized;
        has_maximized |= !minimized;

        const auto& state = box->getNodeHandle()->getNodeState();

        has_log_level[state->getLoggerLevel()] = true;

        bool muted = state->isMuted();
        has_muted |= muted;
        has_unmuted |= !muted;

        bool enabled = box->getNodeWorker()->isProcessingEnabled();
        has_enabled|= enabled;
        has_disabled |= !enabled;

        bool note = dynamic_cast<NoteBox*>(box);
        has_note |= note;
        has_box |= !note;

        ExecutionMode mode = state->getExecutionMode();
        has_pipeline |= mode == ExecutionMode::PIPELINING;
        has_sequential |= mode == ExecutionMode::SEQUENTIAL;
    }


    if(has_box) {
        if(has_disabled){
            QAction* enable = new QAction("enable", &menu);
            enable->setIcon(QIcon(":/checkbox_checked.png"));
            enable->setIconVisibleInMenu(true);
            handler[enable] = std::bind(&GraphView::enableSelection, &view_, true);
            menu.addAction(enable);
        }
        if(has_enabled) {
            QAction* disable = new QAction("disable", &menu);
            disable->setIcon(QIcon(":/checkbox_unchecked.png"));
            disable->setIconVisibleInMenu(true);
            handler[disable] = std::bind(&GraphView::enableSelection, &view_, false);
            menu.addAction(disable);
        }

        QMenu* log_level_menu = menu.addMenu(QIcon(":/logger_level.png"), "set logger level");
        {
            auto add_level = [this, &handler, &menu, log_level_menu, &has_log_level](const std::string& name, int level) {
                QAction* log_level_action = new QAction(name.c_str(), &menu);
                handler[log_level_action] = std::bind(&GraphView::setLoggerLevel, &view_, level);
                log_level_menu->addAction(log_level_action);
                log_level_action->setCheckable(true);
                log_level_action->setChecked(has_log_level[level]);
            };

            add_level("Debug", 0);
            add_level("Info", 1);
            add_level("Warning", 2);
            add_level("Error", 3);
        }
        menu.addMenu(log_level_menu);

        menu.addSeparator();

        if(has_unmuted) {
            QAction* max = new QAction("mute", &menu);
            max->setIcon(QIcon(":/muted.png"));
            max->setIconVisibleInMenu(true);
            handler[max] = std::bind(&GraphView::muteBox, &view_, true);
            menu.addAction(max);
        }
        if(has_muted){
            QAction* min = new QAction("unmute", &menu);
            min->setIcon(QIcon(":/unmuted.png"));
            min->setIconVisibleInMenu(true);
            handler[min] = std::bind(&GraphView::muteBox, &view_, false);
            menu.addAction(min);
        }

        menu.addSeparator();

        if(has_minimized) {
            QAction* max = new QAction("maximize", &menu);
            max->setIcon(QIcon(":/maximize.png"));
            max->setIconVisibleInMenu(true);
            handler[max] = std::bind(&GraphView::minimizeBox, &view_, false);
            menu.addAction(max);
        }
        if(has_maximized){
            QAction* min = new QAction("minimize", &menu);
            min->setIcon(QIcon(":/minimize.png"));
            min->setIconVisibleInMenu(true);
            handler[min] = std::bind(&GraphView::minimizeBox, &view_, true);
            menu.addAction(min);
        }

        QAction* flip = new QAction("flip sides", &menu);
        flip->setIcon(QIcon(":/flip.png"));
        flip->setIconVisibleInMenu(true);
        handler[flip] = std::bind(&GraphView::flipBox, &view_);
        menu.addAction(flip);


        menu.addSeparator();


        QAction* type_pipeline = new QAction("pipeline", &menu);
        type_pipeline->setCheckable(true);
        type_pipeline->setChecked(has_pipeline && !has_sequential);
        menu.addAction(type_pipeline);
        handler[type_pipeline] = std::bind(&GraphView::setExecutionMode, &view_, ExecutionMode::PIPELINING);

        QAction* type_sequence = new QAction("sequential", &menu);
        type_sequence->setCheckable(true);
        type_sequence->setChecked(!has_pipeline && has_sequential);
        menu.addAction(type_sequence);
        handler[type_sequence] = std::bind(&GraphView::setExecutionMode, &view_, ExecutionMode::SEQUENTIAL);

        menu.addSeparator();

        bool threading = !view_.core_.getSettings().get("threadless", false);
        QMenu* thread_menu = menu.addMenu(QIcon(":/thread_group.png"), "thread grouping");
        thread_menu->setEnabled(threading);

        if(thread_menu->isEnabled()) {
            QAction* private_thread = new QAction("private thread", &menu);
            private_thread->setIcon(QIcon(":/thread_group_none.png"));
            private_thread->setIconVisibleInMenu(true);
            handler[private_thread] = std::bind(&GraphView::usePrivateThreadFor, &view_);
            thread_menu->addAction(private_thread);

            QAction* default_thread = new QAction("default thread", &menu);
            default_thread->setIcon(QIcon(":/thread_group.png"));
            default_thread->setIconVisibleInMenu(true);
            handler[default_thread] = std::bind(&GraphView::useDefaultThreadFor, &view_);
            thread_menu->addAction(default_thread);

            thread_menu->addSeparator();

            QMenu* choose_group_menu = new QMenu("thread group", &menu);


            ThreadPool* thread_pool = view_.graph_facade_->getThreadPool();

            std::vector<ThreadGroupPtr> thread_groups = thread_pool->getGroups();
            for(std::size_t i = 0; i < thread_groups.size(); ++i) {
                const ThreadGroup& group = *thread_groups[i];

                if(group.id() == ThreadGroup::PRIVATE_THREAD) {
                    continue;
                }

                std::stringstream ss;
                ss << "(" << group.id() << ") " << group.getName();
                QAction* switch_thread = new QAction(QString::fromStdString(ss.str()), &menu);
                switch_thread->setIcon(QIcon(":/thread_group.png"));
                switch_thread->setIconVisibleInMenu(true);
                handler[switch_thread] = std::bind(&GraphView::switchToThread, &view_, group.id());
                choose_group_menu->addAction(switch_thread);
            }

            choose_group_menu->addSeparator();

            QAction* new_group = new QAction("new thread group", &menu);
            new_group->setIcon(QIcon(":/thread_group_add.png"));
            new_group->setIconVisibleInMenu(true);
            //        handler[new_group] = std::bind(&ThreadPool::createNewThreadGroupFor, &thread_pool_,  box->getNodeWorker());
            handler[new_group] = std::bind(&GraphView::createNewThreadGroupFor, &view_);

            choose_group_menu->addAction(new_group);

            thread_menu->addMenu(choose_group_menu);
        }

        //    QAction* term = new QAction("terminate thread", &menu);
        //    term->setIcon(QIcon(":/stop.png"));
        //    term->setIconVisibleInMenu(true);
        //    handler[term] = std::bind(&NodeBox::killContent, box);
        //    menu.addAction(term);

        menu.addSeparator();


        bool has_profiling = false;
        bool has_not_profiling = false;
        for(NodeBox* box : view_.selected_boxes_) {
            bool p = box->getNodeWorker()->isProfiling();
            has_profiling |= p;
            has_not_profiling |= !p;
        }
        if(has_profiling) {
            QAction* prof = new QAction("stop profiling", &menu);
            prof->setIcon(QIcon(":/stop_profiling.png"));
            prof->setIconVisibleInMenu(true);
            handler[prof] = std::bind(&GraphView::showProfiling, &view_, false);
            menu.addAction(prof);
        }
        if(has_not_profiling){
            QAction* prof = new QAction("start profiling", &menu);
            prof->setIcon(QIcon(":/profiling.png"));
            prof->setIconVisibleInMenu(true);
            handler[prof] = std::bind(&GraphView::showProfiling, &view_, true);
            menu.addAction(prof);
        }

        if(view_.selected_boxes_.size() == 1) {
            QAction* info = new QAction("get information", &menu);
            info->setIcon(QIcon(":/help.png"));
            info->setIconVisibleInMenu(true);
            handler[info] = std::bind(&NodeBox::getInformation, view_.selected_boxes_.front());
            menu.addAction(info);
        }

        menu.addSeparator();

    }

    QAction* set_color = new QAction("set color", &menu);
    set_color->setIcon(QIcon(":/color_wheel.png"));
    set_color->setIconVisibleInMenu(true);
    handler[set_color] = std::bind(&GraphView::chooseColor, &view_);
    menu.addAction(set_color);

    QAction* grp = new QAction("group", &menu);
    grp->setIcon(QIcon(":/group.png"));
    grp->setIconVisibleInMenu(true);
    handler[grp] = std::bind(&GraphView::groupSelected, &view_);
    menu.addAction(grp);

    QAction* ungrp = new QAction("ungroup", &menu);
    ungrp->setIcon(QIcon(":/ungroup.png"));
    handler[ungrp] = std::bind(&GraphView::ungroupSelected, &view_);
    ungrp->setIconVisibleInMenu(true);

    bool is_graph = false;
    if(view_.selected_boxes_.size() == 1) {
        NodeBox* box = view_.selected_boxes_.front();
        is_graph = dynamic_cast<SubgraphNode*>(box->getNode());
    }

    ungrp->setEnabled(is_graph);
    menu.addAction(ungrp);

    menu.addSeparator();

    QAction* snippet = new QAction("define snippet", &menu);
    snippet->setIcon(QIcon(":/snippet_add.png"));
    snippet->setIconVisibleInMenu(true);
    handler[snippet] = std::bind(&GraphView::makeSnippetFromSelected, &view_);
    menu.addAction(snippet);

    QAction* morph = new QAction("change node type", &menu);
    morph->setIcon(QIcon(":/pencil.png"));
    morph->setIconVisibleInMenu(true);
    handler[morph] = std::bind(&GraphView::morphNode, &view_);
    menu.addAction(morph);

    menu.addSeparator();

    QAction* del = new QAction("delete", &menu);
    del->setIcon(QIcon(":/close.png"));
    del->setIconVisibleInMenu(true);
    handler[del] = std::bind(&GraphView::deleteBox, &view_);
    menu.addAction(del);

    QAction* selectedItem = menu.exec(global_pos);

    if(selectedItem) {
        handler[selectedItem]();
    }
}
