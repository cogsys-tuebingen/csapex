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

/// SYSTEM
#include <boost/signals2.hpp>
#include <boost/lambda/bind.hpp>
#include <boost/lambda/construct.hpp>
#include <QComboBox>
#include <QGroupBox>
#include <QLayout>
#include <QListWidget>
#include <QWidget>
#include <QKeyEvent>

#define REGISTER_FILTER(class_name)\
    namespace vision_evaluator { \
    class _____FILTER_##class_name##_registrator { \
        static _____FILTER_##class_name##_registrator instance; \
        _____FILTER_##class_name##_registrator () {\
            std::cout << "register filter instance " << #class_name << std::endl; \
                FilterManager::Constructor constructor; \
                constructor.name = #class_name; \
                constructor.constructor = boost::lambda::new_ptr<class_name>(); \
                vision_evaluator::FilterManager::available_filters.push_back(constructor); \
        } \
    };\
    _____FILTER_##class_name##_registrator _____FILTER_##class_name##_registrator::instance;\
    }

namespace vision_evaluator
{
class FilterManager : public Filter
{
    Q_OBJECT

public:
    struct Constructor {
        std::string name;
        boost::function<Filter*()> constructor;

        Filter::Ptr operator () (){
            Filter::Ptr res(constructor());
            res->setName(name);
            assert(res.get() != NULL);
            return res;
        }
    };
    static std::vector<FilterManager::Constructor> available_filters;

public:
    FilterManager();

    virtual void insert(QBoxLayout* parent);
    virtual void filter(cv::Mat img, cv::Mat mask);

private Q_SLOTS:
    void add_filter(int index);
    void select_row(int index);
    void delete_filter();
    void refresh_filters();
    void reorder_filters();

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
