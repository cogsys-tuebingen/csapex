#ifndef DESIGNER_STYLEABLE_H
#define DESIGNER_STYLEABLE_H

/// SYSTEM
#include <QColor>

namespace csapex
{
class DesignerStyleable
{
public:
    QColor lineColor() const
    {
        return line_color_;
    }
    QColor lineColorError() const
    {
        return line_error_color_;
    }
    QColor lineColorBlocked() const
    {
        return line_blocked_color_;
    }
    QColor lineColorDisabled() const
    {
        return line_disabled_color_;
    }
    int lineWidth() const
    {
        return line_width_;
    }

    void setLineColor(const QColor& c)
    {
        line_color_ = c;
    }
    void setLineColorError(const QColor& c)
    {
        line_error_color_ = c;
    }
    void setLineColorBlocked(const QColor& c)
    {
        line_blocked_color_ = c;
    }
    void setLineColorDisabled(const QColor& c)
    {
        line_disabled_color_ = c;
    }
    void setLineWidth(int width)
    {
        line_width_ = width;
    }

protected:
    // CSS settable
    QColor line_color_;
    QColor line_error_color_;
    QColor line_blocked_color_;
    QColor line_disabled_color_;
    int line_width_;
};
}

#endif // DESIGNER_STYLEABLE_H

