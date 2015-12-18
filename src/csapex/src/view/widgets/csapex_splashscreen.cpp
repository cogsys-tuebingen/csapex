/// HEADER
#include <csapex/view/widgets/csapex_splashscreen.h>

/// PROJECT
#include <csapex/info.h>

/// SYSTEM
#include <QPixmap>
#include <QPainter>

using namespace csapex;

CsApexSplashScreen::CsApexSplashScreen(QWidget */*parent*/) :
    QSplashScreen(QPixmap(":/apex_splash.png"))

{
    this->setCursor(Qt::BusyCursor);
}

void CsApexSplashScreen::drawContents(QPainter *painter)
{
    painter->translate(width() - 320, 50);

    QFont font = painter->font();
    font.setPixelSize(20);
    font.setBold(true);
    painter->setFont(font);
    painter->setPen(QPen(QColor(47, 68, 78).light()));

    QPointF pos(0, 0);

    painter->drawText(pos, QString("cs::APEX ") + csapex::info::CSAPEX_VERSION.c_str());
    pos.setY(pos.y() + 20);

    font.setPixelSize(10);

    painter->drawText(pos, QString("     (") + csapex::info::GIT_COMMIT_HASH.c_str() + "  / " + csapex::info::GIT_BRANCH.c_str() + ")");
    pos.setY(pos.y() + 16);

    font.setBold(false);
    painter->setFont(font);

#if __GNUC__
    painter->drawText(pos, QString("Based on QT") + QT_VERSION_STR + " and GCC " + __VERSION__);
#else
    painter->drawText(pos, QString("Based on QT") + QT_VERSION_STR);
#endif
    pos.setY(pos.y() + 12);

    painter->drawText(pos, QString("Built on ") + __DATE__ + " at " + __TIME__);
    pos.setY(pos.y() + 16);

    font.setPixelSize(10);
    painter->setFont(font);

    painter->drawText(pos, QString("The program is provided AS IS with NO WARRANTY "));
    pos.setY(pos.y() + 12);
    painter->drawText(pos, QString("OF ANY KIND, INCLUDING THE WARRANTY OF DESIGN, "));
    pos.setY(pos.y() + 12);
    painter->drawText(pos, QString("MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE."));

    pos.setX(pos.x() + 30);
    pos.setY(pos.y() + 40);

    font.setPixelSize(16);
    font.setBold(true);
    painter->setFont(font);

    painter->setPen(QPen(QColor(47, 68, 78)));

    painter->drawText(pos, message());

//    QSplashScreen::drawContents(painter);
}

/// MOC
#include "../../../include/csapex/view/widgets/moc_csapex_splashscreen.cpp"
