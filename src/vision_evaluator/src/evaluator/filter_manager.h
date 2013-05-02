/*
 * filter_manager.h
 *
 *  Created on: Mar 28, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#ifndef FILTER_MANAGER_H
#define FILTER_MANAGER_H

/// COMPONENT
#include "filter.h"
#include "generic_manager.hpp"

/// SYSTEM
#include <QComboBox>
#include <QGroupBox>
#include <QLayout>
#include <QListWidget>
#include <QWidget>
#include <QKeyEvent>

#define REGISTER_FILTER(class_name)\
    REGISTER_GENERIC(FilterManager, class_name)

namespace vision_evaluator
{
class FilterManager : public GenericManager<Filter>
{
    Q_OBJECT

public:
    FilterManager();

    virtual void insert(QBoxLayout* parent);
    virtual void filter(cv::Mat& img, cv::Mat& mask);

private Q_SLOTS:
    void add_filter(int index);
    void select_row(int index);
    void delete_filter();
    void refresh_filters();
    void reorder_filters();
    void filterCopy(cv::Mat img, cv::Mat mask);

Q_SIGNALS:
    void outputMat(cv::Mat, cv::Mat);
    void display_request(const QSharedPointer<QImage>);

private:
    QBoxLayout* parent_layout;

    QGroupBox* group;
    QBoxLayout* group_layout;

    QGroupBox* options;
    QBoxLayout* options_layout;

    QListWidget* active_filters;
    QComboBox* available;

    std::vector<Filter::Ptr> active;
};






class ListSignalHandler : public QObject
{
    Q_OBJECT

public:
    ListSignalHandler(int key, boost::function<void()> remove, boost::function<void()> reorder)
        : key(key), remove(remove), reorder(reorder)
    {}

protected:
    bool eventFilter(QObject* obj, QEvent* event) {
        if(event->type() == QEvent::ChildRemoved) {
            reorder();

        } else if(event->type() == QEvent::KeyPress) {
            QKeyEvent* keyEvent = static_cast<QKeyEvent*>(event);
            if(keyEvent->key() == key) {
                remove();
                return true;
            }
        }
        // standard event processing
        return QObject::eventFilter(obj, event);
    }

private:
    int key;
    boost::function<void()> remove;
    boost::function<void()> reorder;
};


} /// NAMESPACE

#endif // FILTER_MANAGER_H
