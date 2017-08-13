#ifndef CPU_AFFINITY_WIDGET_H
#define CPU_AFFINITY_WIDGET_H

/// PROJECT
#include <csapex/utility/cpu_affinity.h>
#include <csapex/utility/utility_fwd.h>
#include <csapex/model/observer.h>

/// SYSTEM
#include <QWidget>
#include <QMetaType>
#include <QPointF>
#include <QVector>
#include <memory>

namespace csapex
{


class CpuAffinityRenderer
{
public:
    enum EditMode { Editable, ReadOnly };

    CpuAffinityRenderer();
    CpuAffinityRenderer(const CpuAffinity &cpu_affinity);

    void paint(QPainter *painter, const QRect &rect,
               const QPalette &palette, EditMode mode) const;
    QSize sizeHint() const;
    CpuAffinity &getCpuAffinity();

private:
    CpuAffinity cpu_affinity_;
};


class CpuAffinityWidget : public QWidget, public Observer
{
    Q_OBJECT

public:
    CpuAffinityWidget(const CpuAffinityPtr &affinity, QWidget *parent = 0);

    QSize sizeHint() const override;
    void setRenderer(const CpuAffinityRenderer &renderer) {
        renderer_ = renderer;
    }
    CpuAffinityRenderer getWidget() { return renderer_; }

//signals:
//    void editingFinished();

protected:
    void paintEvent(QPaintEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;

private:
    int cpuAtPosition(int x);

    CpuAffinityPtr affinity_;
    CpuAffinityRenderer renderer_;

    bool modifying_;
    bool modify_enabled_;
};

}

Q_DECLARE_METATYPE(csapex::CpuAffinityRenderer)

#endif // CPU_AFFINITY_WIDGET_H
