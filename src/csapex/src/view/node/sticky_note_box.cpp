/// HEADER
#include <csapex/view/node/sticky_note_box.h>

/// COMPONENT
#include <csapex/nodes/sticky_note.h>
#include <csapex/model/node_state.h>
#include <csapex/view/designer/graph_view.h>

/// SYSTEM
#include <QStyle>
#include <QStyleOption>
#include <QStylePainter>
#include <QSizeGrip>
#include <QVBoxLayout>
#include <QTextEdit>
#include <QResizeEvent>

using namespace csapex;


StickyNoteBox::StickyNoteBox(Settings &settings, NodeFacadePtr node_facade_, QIcon icon, GraphView *parent)
    : NodeBox(settings, node_facade_, icon, parent),
      default_bg_color_(255, 220, 100),
      border_color_difference_(120)
{
}


StickyNoteBox::~StickyNoteBox()
{
}

void StickyNoteBox::paintEvent(QPaintEvent* e)
{
    if(!adapter_) {
        return;
    }

    bool focused = property("focused").toBool();

    QStylePainter p(this);

    int line_width = 2;
    int snap_width = 16;

    QRect r = rect().adjusted(line_width/2, line_width/2, -line_width/2, -line_width/2);

    QPen pen (palette().foreground(), line_width, focused ? Qt::DashLine : Qt::SolidLine,
              Qt::RoundCap, Qt::BevelJoin);
    p.setPen(pen);

    QPoint a = r.bottomRight() - QPoint(snap_width, 0);
    QPoint b = r.bottomRight() - QPoint(0, snap_width);
    QPoint c = r.bottomRight() - QPoint(snap_width, snap_width);

    QPolygon outline ({r.topLeft(), r.topRight(), b, a, r.bottomLeft(), r.topLeft()});
    QPolygon snap ({a, b, c, a});

    QPainterPath outline_path;
    outline_path.addPolygon(outline);

    QPainterPath snap_path;
    snap_path.addPolygon(snap);

    QColor col = default_bg_color_;

    {
        int r, g, b;
        node_facade_->getNodeState()->getColor(r, g, b);
        if(r >= 0 && g >= 0 && b >= 0) {
            col = QColor(r,g,b);
        }
    }

    if(focused) {
        bool light = (col.lightness() > 128);
        col = light ? col.darker(120) : col.lighter(120);
    }

    p.fillPath(outline_path, QBrush(col));
    p.fillPath(snap_path, QBrush(col.dark()));

    p.drawPath(outline_path);
    p.drawPath(snap_path);

}

void StickyNoteBox::resizeEvent(QResizeEvent *e)
{
    if(node_facade_->hasParameter("w")) {
        node_facade_->setParameter("w", width());
    }

    if(node_facade_->hasParameter("h")) {
        node_facade_->setParameter("h", height());
    }
}

void StickyNoteBox::construct()
{
    setFocusPolicy(Qt::ClickFocus);
    setWindowFlags(Qt::SubWindow);
    setAutoFillBackground(false);
    setAttribute( Qt::WA_TranslucentBackground, true );
    setAttribute(Qt::WA_NoSystemBackground, true);

    QGridLayout* layout = new QGridLayout;
    layout->setContentsMargins(0,0,0,0);

    grip_ = new QSizeGrip(this);
    grip_->installEventFilter(this);

    edit_ = new QTextEdit;
    edit_->setMinimumSize(16, 16);

    layout->addWidget(edit_, 0, 0, 1, 1);

    layout->addWidget(grip_, 1, 1, 1 ,1, Qt::AlignBottom | Qt::AlignRight);
    setLayout(layout);
}

void StickyNoteBox::init()
{
    std::string text = node_facade_->readParameter<std::string>("text");
    edit_->setText(QString::fromStdString(text));

    stopResize();

    NodeBox::init();

    node_facade_->parameters_changed.connect([this](){
        edit_->setText(QString::fromStdString(node_facade_->readParameter<std::string>("text")));
    });

    QObject::connect(edit_, &QTextEdit::textChanged, [this](){
        node_facade_->setParameter("text", edit_->toPlainText().toStdString());
    });


    NodeState* state = node_facade_->getNodeState().get();
    observer_.observeQueued<StickyNoteBox>(state->color_changed, this, static_cast<void (StickyNoteBox::*)()>(&NodeBox::changeColor));
    observer_.observeQueued<StickyNoteBox>(state->pos_changed, this, static_cast<void (StickyNoteBox::*)()>(&NodeBox::updatePosition));

    updateStylesheetColor();
    updateVisualsRequest();
}

void StickyNoteBox::startResize()
{
    setMinimumSize(40, 40);
    setMaximumSize(10000, 10000);
}
void StickyNoteBox::stopResize()
{
    if(node_facade_->hasParameter("w") && node_facade_->hasParameter("h")) {
        int w = node_facade_->readParameter<int>("w");
        int h = node_facade_->readParameter<int>("h");
        setFixedSize(std::max(40, w), std::max(40, h));
    } else {
        setFixedSize(40, 40);
    }
}


void StickyNoteBox::updateComponentInformation(GraphFacade *graph)
{

}

void StickyNoteBox::updateThreadInformation()
{

}

void StickyNoteBox::updateFrequencyInformation()
{

}

void StickyNoteBox::setSelected(bool selected)
{
    setProperty("focused",selected);
    refreshStylesheet();
}

void StickyNoteBox::refreshTopLevelStylesheet()
{
    style()->polish(this);
    update();
}


void StickyNoteBox::updateStylesheetColor()
{
    NodeStatePtr state = node_facade_->getNodeState();

    int r, g, b;
    state->getColor(r, g, b);

    QString style = parent_ ? parent_->styleSheet() : styleSheet();

    QColor text_color = Qt::black;
    QColor border_color = default_bg_color_.darker(border_color_difference_);
    if(r >= 0 && g >= 0 && b >= 0) {
        QColor background(r,g,b);

        bool light = (background.lightness() > 128);
        text_color = light ? Qt::black: Qt::white;
        border_color = light ? background.darker(border_color_difference_): background.lighter(border_color_difference_);
    }

    style += "csapex--StickyNoteBox QTextEdit { ";
    style += "color: rgb(" + QString::number(text_color.red()) + ", " +
            QString::number(text_color.green()) + ", " +
            QString::number(text_color.blue()) + ") !important;";
    style += "border-right: 1px dotted rgb(" + QString::number(border_color.red()) + ", " +
            QString::number(border_color.green()) + ", " +
            QString::number(border_color.blue()) + ") !important;";
    style += "border-bottom: 1px dotted rgb(" + QString::number(border_color.red()) + ", " +
            QString::number(border_color.green()) + ", " +
            QString::number(border_color.blue()) + ") !important;";
    style += "}";

    setStyleSheet(style);
}

/// MOC
#include "../../../include/csapex/view/node/moc_sticky_note_box.cpp"
