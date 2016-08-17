#ifndef CSAPEX_QT_EXPORT_H
#define CSAPEX_QT_EXPORT_H

#include <QtCore/qglobal.h>

#include <csapex/csapex_qt_export.h>

#if WIN32
#if defined(csapex_qt_EXPORTS)
   #define CSAPEX_QT_EXPORT Q_DECL_EXPORT
#else
   #define CSAPEX_QT_EXPORT Q_DECL_IMPORT
#endif
#else
   #define CSAPEX_QT_EXPORT
#endif

#endif // CSAPEX_QT_EXPORT_H

