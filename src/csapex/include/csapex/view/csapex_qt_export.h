#ifndef CSAPEX_QT_EXPORT_H
#define CSAPEX_QT_EXPORT_H

#if WIN32
#include <QtCore/qglobal.h>

#include "csapex_qt_export_cmake.h"

#if defined(CSAPEX_QT_EXPORT_CMAKE)
#  define CSAPEX_QT_EXPORT Q_DECL_EXPORT
#else
#  define CSAPEX_QT_EXPORT Q_DECL_IMPORT
#endif

#endif

#endif // CSAPEX_QT_EXPORT_H

