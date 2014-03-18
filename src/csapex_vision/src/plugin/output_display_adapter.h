#ifndef OUTPUD_DISPLAY_ADAPTER_H
#define OUTPUD_DISPLAY_ADAPTER_H

/// PROJECT
#include <csapex/view/node_adapter.h>
#include <csapex/view/node_adapter_builder.h>

/// COMPONENT
#include "output_display.h"

/// SYSTEM
#include <QGraphicsView>

namespace csapex {

class OutputDisplayAdapter : public QObject, public NodeAdapter
{
    Q_OBJECT

public:
    OutputDisplayAdapter(OutputDisplay *node);

    virtual Memento::Ptr getState() const;
    virtual void setState(Memento::Ptr memento);

    virtual void setupUi(QBoxLayout* layout);

public Q_SLOTS:
    void display(QSharedPointer<QImage> img);
    void setAsync(int a);
    void fitInView();

Q_SIGNALS:
    void displayRequest(QSharedPointer<QImage> img);

protected:
    bool eventFilter(QObject* o, QEvent* e);

    OutputDisplay* wrapped_;

    struct State : public Memento {
        int width;
        int height;

        State()
            : width(300), height(300)
        {}

        virtual void writeYaml(YAML::Emitter& out) const {
            out << YAML::Key << "width" << YAML::Value << width;
            out << YAML::Key << "height" << YAML::Value << height;
        }
        virtual void readYaml(const YAML::Node& node) {
            node["width"] >> width;
            node["height"] >> height;
        }
    };


private:
    QSize last_size_;
    State state;

    QGraphicsPixmapItem* pixmap_;

    QGraphicsView* view_;

    QImage empty;
    QPainter painter;

    bool down_;
    QPoint last_pos_;
};

}

#endif // OUTPUD_DISPLAY_ADAPTER_H
