/*
 * filter_manager.cpp
 *
 *  Created on: Mar 28, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

/// HEADER
#include "filter_manager.h"

/// COMPONENT
#include "../qt_helper.hpp"

/// SYSTEM
#include <boost/bind.hpp>
#include <utils/LibUtil/QtCvImageConverter.h>

using namespace vision_evaluator;

FilterManager::FilterManager()
    : PluginManager<Filter>("vision_evaluator::Filter"), group(NULL), group_layout(NULL), active_filters(NULL), available(NULL)
{
}

void FilterManager::insert(QBoxLayout* layout)
{
    if(!pluginsLoaded()) {
        reload();
    }

    parent_layout = layout;
    group = new QGroupBox("Filters");
    layout->addWidget(group);

    group_layout = new QVBoxLayout;
    group->setLayout(group_layout);

    active_filters = new QListWidget;

    active_filters->setDragEnabled(true);
    active_filters->setAcceptDrops(true);
    active_filters->setMovement(QListView::Snap);
    active_filters->setDragDropMode(QAbstractItemView::InternalMove);
    active_filters->setDefaultDropAction(Qt::MoveAction);

    active_filters->setSizePolicy(QSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding));
    active_filters->setMinimumSize(QSize(32, 0));
    active_filters->setMaximumSize(QSize(3200, 250));

    group_layout->addWidget(active_filters);

    available = new QComboBox;
    available->addItem("Add Filter");
    available->addItem("----------");

    group_layout->addWidget(available);

    options = new QGroupBox("Options");
    options_layout = new QVBoxLayout;
    options->setLayout(options_layout);
    group_layout->addWidget(options);

    for(std::map<std::string, Constructor>::const_iterator it = availableClasses().begin(); it != availableClasses().end(); ++it) {
        available->addItem(it->second.getName().c_str());
    }

    QObject::connect(available, SIGNAL(currentIndexChanged(int)), this, SLOT(add_filter(int)));
    QObject::connect(available, SIGNAL(activated(int)), this, SLOT(add_filter(int)));

    active_filters->installEventFilter(new ListSignalHandler(Qt::Key_Delete,
                                       boost::bind(&FilterManager::delete_filter, this),
                                       boost::bind(&FilterManager::reorder_filters, this)));

    QObject::connect(active_filters, SIGNAL(currentRowChanged(int)), this, SLOT(select_row(int)));

    refresh_filters();
}

void FilterManager::add_filter(int index)
{
    available->setCurrentIndex(0);

    if(index < 2) {
        // text info in box
        return;
    }

    Filter::Ptr filter = availableClasses(index - 2)();

    QObject::connect(filter.get(), SIGNAL(filter_changed()), this, SIGNAL(filter_changed()));

    active.push_back(filter);

    std::cout << "add filter: " << filter->getName() << std::endl;

    refresh_filters();

    filter_changed();
}

void FilterManager::delete_filter()
{
    unsigned idx = active_filters->currentIndex().row();

    if(idx > active.size()) {
        return;
    }

    active.erase(active.begin() + idx);

    refresh_filters();

    filter_changed();
}

void FilterManager::select_row(int index)
{
    QtHelper::clearLayout(options_layout);

    delete options_layout;
    options_layout = new QVBoxLayout;

    options->setLayout(options_layout);

    if(index < 0) {
        return;
    }

    active[index]->insert(options_layout);
}

void FilterManager::refresh_filters()
{
    std::cout << "refresh filters" << std::endl;
    active_filters->clear();

    //    active_filters->setMinimumHeight(active_filters->sizeHintForRow(0));
    //    active_filters->setFixedHeight(active.size() * active_filters->sizeHintForRow(0));
    for(unsigned i = 0; i < active.size(); ++i) {
        Filter::Ptr e = active[i];

        QListWidgetItem* item = new QListWidgetItem(e->getName().c_str());
        item->setData(1, i);
        //item->setFlags(Qt::ItemIsUserCheckable | Qt::ItemIsEnabled | Qt::ItemIsSelectable | Qt::ItemIsDragEnabled | Qt::ItemIsDropEnabled);
        active_filters->addItem(item);
    }

    int l,t,r,b;
    active_filters->getContentsMargins(&l, &t, &r, &b);
    int row_height =  active_filters->sizeHintForRow(0) + b;
    active_filters->setFixedHeight(active.size() * row_height);

    parent_layout->update();
}

void FilterManager::reorder_filters()
{
    bool change_happened = false;

    std::vector<Filter::Ptr> tmp;
    for(int row = 0; row < active_filters->model()->rowCount(); ++row) {
        QListWidgetItem* item = active_filters->item(row);
        int ref = item->data(1).toInt();

        if(ref != row) {
            change_happened = true;
        }

        tmp.push_back(active[ref]);
    }

    if(change_happened) {
        active = tmp;
        refresh_filters();
    }
}

void FilterManager::filterCopy(cv::Mat img, cv::Mat mask)
{
    filter(img, mask);
}

void FilterManager::filter(cv::Mat& img, cv::Mat& mask)
{
    if(!active.empty()) {
        for(unsigned i = 0; i < active.size(); ++i) {
            active[i]->filter(img, mask);
        }
    }

    Q_EMIT outputMat(img, mask);
    Q_EMIT display_request(QtCvImageConverter::Converter<QImage, QSharedPointer>::mat2QImage(img));
}
