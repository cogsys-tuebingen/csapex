/// HEADER
#include <csapex/view/widgets/notification_widget.h>

/// COMPONENT
#include <csapex/view/designer/designer_scene.h>
#include <csapex/view/designer/graph_view.h>
#include <csapex/view/node/box.h>

/// SYSTEM
#include <QPainter>
#include <QStyleOption>
#include <QLabel>
#include <QMouseEvent>
#include <QEvent>
#include <QTextStream>
#include <QTimer>
#include <QPropertyAnimation>
#include <QGraphicsOpacityEffect>
#include <QPushButton>

using namespace csapex;

NotificationWidget::NotificationWidget(const Notification &notification, QWidget *parent)
    : QWidget(parent), timer_(nullptr), notification_(notification)
{
    timer_ = new QTimer;
    timer_->setSingleShot(true);

    QObject::connect(timer_, &QTimer::timeout, this, &NotificationWidget::fadeout);

    setCursor(Qt::PointingHandCursor);


    eff = new QGraphicsOpacityEffect(this);
    eff->setOpacity(0.9);
    this->setGraphicsEffect(eff);


    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    setMinimumSize(50, 30);
    setMaximumSize(500, 100);

    setAutoFillBackground(false);

    QGridLayout* layout = new QGridLayout;
    setLayout(layout);

    icon_ = new QLabel;
    layout->addWidget(icon_, 0, 0);

    label_ = new QLabel;
    layout->addWidget(label_, 0, 1);

    QPushButton* btn = new QPushButton;
    btn->setIcon(QIcon(":/hide.png"));
    layout->addWidget(btn, 0, 2);

    QObject::connect(btn, &QPushButton::clicked, [this](){
        fadeout();
    });

    icon_->installEventFilter(this);
    label_->installEventFilter(this);

    setNotification(notification);

    setMouseTracking(true);
}

NotificationWidget::~NotificationWidget()
{
    delete timer_;
}

void NotificationWidget::shutdown()
{
    hide();
    deleteLater();
}

void NotificationWidget::fadeout()
{
    timeout();

    delete timer_;
    timer_ = nullptr;

    QPropertyAnimation *a = new QPropertyAnimation(graphicsEffect(),"opacity");
    a->setDuration(1000);
    a->setStartValue(eff->opacity());
    a->setEndValue(0);
    a->setEasingCurve(QEasingCurve::OutBack);
    a->start(QPropertyAnimation::DeleteWhenStopped);
    connect(a, &QPropertyAnimation::finished, this, &NotificationWidget::shutdown);
}

bool NotificationWidget::eventFilter(QObject *, QEvent * e)
{
    if(e->type() == QEvent::MouseButtonRelease) {
        e->ignore();
        return true;

    } else {
        return false;
    }
}

void NotificationWidget::mouseReleaseEvent(QMouseEvent *)
{
    activated(notification_.auuid);
}

void NotificationWidget::paintEvent(QPaintEvent* event)
{
    QPainter painter(this);

    QStyleOption opt;
    opt.init(this);
    style()->drawPrimitive(QStyle::PE_PanelButtonBevel, &opt, &painter, this);
}

void NotificationWidget::setNotification(const Notification &notification)
{
    bool is_error = notification.error != ErrorState::ErrorLevel::NONE;
    bool was_error = notification_.error != ErrorState::ErrorLevel::NONE;

    notification_ = notification;

    if(is_error) {
        QString s;
        QTextStream ss(&s);
        ss << "<b>" << QString::fromStdString(notification_.auuid.getFullName()) << "</b>:<br /> " << QString::fromStdString(notification_.message);
        label_->setText(s);

        if(notification.error == ErrorState::ErrorLevel::ERROR) {
            icon_->setText("<img src=':error.png' />");
        } else {
            icon_->setText("<img src=':help.png' />");
        }

    } else if(!was_error) {
        icon_->setText("<img src=':error.png' />");
    }

    layout()->activate();
    adjustSize();

    timer_->stop();
    if(notification_.error == ErrorState::ErrorLevel::NONE) {
        timer_->start(1 * 1000);
    }
}

/// MOC
#include "../../../include/csapex/view/widgets/moc_notification_widget.cpp"
