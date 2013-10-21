#ifndef DISPLAYABLE_H
#define DISPLAYABLE_H

/// COMPONENT
#include <csapex/csapex_fwd.h>

/// SYSTEM
#include <string>
#include <QWidget>
#include <QMutex>

namespace csapex
{

class Displayable
{
public:
    enum ErrorLevel {
        EL_ERROR, EL_WARNING
    };

public:
    virtual void setError(bool e, const std::string& msg = "", ErrorLevel level = EL_ERROR);
    void setErrorSilent(bool e, const std::string& msg = "", ErrorLevel level = EL_ERROR);

    bool isError() const;
    ErrorLevel errorLevel() const;
    std::string errorMessage() const;

    virtual void setBox(Box* box);
    Box* getBox() const;

protected:
    Displayable();
    virtual ~Displayable();

    virtual void errorEvent(bool error, ErrorLevel level);

protected:
    mutable QMutex mutex;
    Box* box_;
    bool error_;
    std::string error_msg_;
    ErrorLevel level_;
};

}

#endif // DISPLAYABLE_H
