/// HEADER
#include <csapex/view/widgets/cpu_affinity_widget.h>

/// PROJECT
#include <csapex/utility/cpu_affinity.h>

/// SYSTEM
#include <QtWidgets>

using namespace csapex;



const int PaintingScaleFactor = 20;

CpuAffinityRenderer::CpuAffinityRenderer(const CpuAffinity& cpu_affinity)
    : cpu_affinity_(cpu_affinity)
{
}

CpuAffinityRenderer::CpuAffinityRenderer()
{

}

CpuAffinity& CpuAffinityRenderer::getCpuAffinity()
{
    return cpu_affinity_;
}

QSize CpuAffinityRenderer::sizeHint() const
{
    return PaintingScaleFactor * QSize(cpu_affinity_.getNumCpus(), 1);
}

void CpuAffinityRenderer::paint(QPainter *painter, const QRect &rect,
                       const QPalette &palette, EditMode mode) const
{
    painter->save();

    painter->setRenderHint(QPainter::Antialiasing, true);
    painter->setPen(Qt::NoPen);

    if (mode == Editable) {
        painter->setBrush(palette.highlight());
    } else {
        painter->setBrush(palette.foreground());
    }

    int yOffset = (rect.height() - PaintingScaleFactor) / 2;
    painter->translate(rect.x(), rect.y() + yOffset);
//    painter->scale(PaintingScaleFactor, PaintingScaleFactor);

    QTextOption opt(Qt::AlignVCenter | Qt::AlignCenter);
//    opt.setUseDesignMetrics(true);
//    opt.setWrapMode(QTextOption::WrapAnywhere);

    double margin = 0.1 * PaintingScaleFactor;
    double width = 1.0 * PaintingScaleFactor;

    double real_width = width - 2*margin;

    QFont font = painter->font();
    font.setPixelSize(real_width * 0.75);
    painter->setFont(font);

    for (unsigned i = 0; i < cpu_affinity_.getNumCpus(); ++i) {
        QRectF rect(margin, margin, real_width, real_width);

        QPen pen(QBrush(QColor::fromRgb(0,0,0)), 0.1, Qt::SolidLine);

        QBrush brush;
        if (cpu_affinity_.isCpuUsed(i)) {
            brush.setColor(QColor::fromRgb(50, 255, 100));
        } else {
            brush.setColor(QColor::fromRgb(255, 50, 100));
        }
        if (mode == Editable) {
            brush.setColor(brush.color().light(120));
            brush.setStyle(Qt::Dense1Pattern);
            pen.setStyle(Qt::DotLine);
        } else {
            brush.setStyle(Qt::SolidPattern);
        }

        painter->setPen(pen);
        painter->setBrush(brush);

        painter->drawRect(rect);

        painter->drawText(rect, QString::number(i), opt);
        painter->translate(width, 0.0);
    }

    painter->restore();
}







CpuAffinityWidget::CpuAffinityWidget(const CpuAffinityPtr& affinity, QWidget *parent)
    : QWidget(parent),
      affinity_(affinity),
      renderer_(*affinity_),
      modifying_(false)
{
    setMouseTracking(true);
    setAutoFillBackground(true);

    observe(affinity_->affinity_changed, [this](const CpuAffinity*) {
       update();
    });
}

QSize CpuAffinityWidget::sizeHint() const
{
    return renderer_.sizeHint();
}

void CpuAffinityWidget::paintEvent(QPaintEvent *)
{
    QPainter painter(this);
    renderer_.paint(&painter, rect(), this->palette(),
                       CpuAffinityRenderer::Editable);
}

void CpuAffinityWidget::mouseMoveEvent(QMouseEvent *event)
{
    if(modifying_) {
        int cpu = cpuAtPosition(event->pos().x());
        if(renderer_.getCpuAffinity().isCpuUsed(cpu) != modify_enabled_) {
            renderer_.getCpuAffinity().toggleCpu(cpu);
            update();
        }
    }
}

void CpuAffinityWidget::mousePressEvent(QMouseEvent* event)
{
    modifying_ = true;
    int cpu = cpuAtPosition(event->pos().x());
    renderer_.getCpuAffinity().toggleCpu(cpu);
    update();

    modify_enabled_ = renderer_.getCpuAffinity().isCpuUsed(cpu);
}

void CpuAffinityWidget::mouseReleaseEvent(QMouseEvent* event)
{
    modifying_ = false;
}

int CpuAffinityWidget::cpuAtPosition(int x)
{
    int numcpus = renderer_.getCpuAffinity().getNumCpus();
    int cpu = (x / (renderer_.sizeHint().width() / numcpus));
    if (cpu < 0 || cpu >= numcpus)
        return -1;

    return cpu;
}

/// MOC
#include "../../../include/csapex/view/widgets/moc_cpu_affinity_widget.cpp"
