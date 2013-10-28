/// HEADER
#include <csapex/view/profiling_widget.h>

/// COMPONENT
#include <csapex/view/box.h>
#include <csapex/model/node.h>
#include <csapex/model/node_worker.h>

/// SYSTEM
#include <QPainter>

using namespace csapex;

ProfilingWidget::ProfilingWidget(QWidget *parent, Box *box)
    : QWidget(parent), box_(box), node_worker_(box->getNode()->getNodeWorker())
{
    w_ = 200;
    h_ = 75;

    setFixedSize(w_, h_);

    reposition(box_);

    connect(box_, SIGNAL(destroyed()), this, SLOT(close()));
    connect(box_, SIGNAL(destroyed()), this, SLOT(deleteLater()));
    connect(box_, SIGNAL(moved(Box*,int,int)), this, SLOT(reposition(Box*)));
}

void ProfilingWidget::reposition(Box *box)
{
    move(box->pos() + QPoint(0, box->height()));
}

void ProfilingWidget::paintEvent(QPaintEvent *)
{
    QPainter p(this);

    QRect rect = contentsRect().adjusted(0,0,-1,-1);

    p.setBrush(QBrush(QColor(200, 200, 255)));
    p.setPen(QPen(Qt::black));
    p.setOpacity(0.6);

    p.drawRect(rect);

    int padding = 5;

    int left = padding + 50;
    int right = w_ - padding;
    int up = padding;
    int bottom = h_ - padding;

    double content_width = right - left - 2 * padding;
    double indiv_width = content_width / node_worker_->timer_history_length_;
    int content_height = bottom - up - 2 * padding;

    p.setPen(QPen(Qt::black));

    // x-axis
    p.drawLine(left, bottom, right, bottom);

    // y-axis
    p.drawLine(left, bottom, left, up);

    size_t max = node_worker_->timer_history_length_;
    int n = std::min(max, node_worker_->timer_history_.size());

    if(n > 0) {
        int maxt = *std::max_element(node_worker_->timer_history_.begin(), node_worker_->timer_history_.end());

        std::stringstream txt;
        txt << maxt << " ms";


        QFont font;
        font.setPixelSize(12);
        p.setFont(font);

        QFontMetrics metrics(font);

        int dy = metrics.height();

        QTextOption opt(Qt::AlignRight);

        p.setPen(QPen(QColor(20, 20, 20)));
        p.drawText(QRect(0, up, left -padding, dy), txt.str().c_str(), opt);
        p.drawText(QRect(0, bottom - dy, left - padding, dy), "0 ms", opt);

        double maxt_f = std::max(1, maxt);

        double x = left + padding + (max - n) * indiv_width;
        for(int i = 0; i < n; ++i) {
            int time = node_worker_->timer_history_[i];

            double f = time / maxt_f;
            assert(0.0 <= f);
            assert(f <= 1.0);
            int height = f * content_height;

            int r = 255 * f;
            int g = 255 * (1-f);
            int b = 128;
            p.setBrush(QBrush(QColor(r, g, b)));
            p.drawRect(x, bottom - height, indiv_width, height);

            x += indiv_width;
        }
    }
}
