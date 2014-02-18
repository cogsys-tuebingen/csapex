#ifndef OUTPUT_DISPLAY_H
#define OUTPUT_DISPLAY_H

/// PROJECT
#include <csapex/model/boxed_object.h>

/// SYSTEM
#include <QGraphicsView>

namespace csapex
{

class ConnectorIn;

class OutputDisplay : public BoxedObject
{
    Q_OBJECT

public:
    OutputDisplay();
    virtual ~OutputDisplay();

    virtual void fill(QBoxLayout* layout);

    virtual void enable();
    virtual void disable();
    virtual void connectorChanged();

    virtual Memento::Ptr getState() const;
    virtual void setState(Memento::Ptr memento);

public Q_SLOTS:
    void process();
    void display(QSharedPointer<QImage> img);
    void setAsync(int a);
    void fitInView();

Q_SIGNALS:
    void displayRequest(QSharedPointer<QImage> img);

protected:
    bool eventFilter(QObject* o, QEvent* e);

private:

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

    QSize last_size_;
    State state;
    ConnectorIn* input_;
    QGraphicsPixmapItem* pixmap_;

    QGraphicsView* view_;

    QImage empty;
    QPainter painter;

    bool down_;
    QPoint last_pos_;
};

}

#endif // OUTPUT_DISPLAY_H
