#ifndef DISPLAYABLE_H
#define DISPLAYABLE_H

/// SYSTEM
#include <string>
#include <QWidget>

namespace vision_evaluator {

class Box;

class Displayable : public QWidget
{
public:
    virtual void setError(bool e, const std::string& msg = "");
    bool isError() const;

    virtual void setBox(Box* box);
    Box* getBox() const;

protected:
    Displayable();

    virtual void errorEvent(bool error);

protected:
    Box* box_;
    bool error_;
};

}

#endif // DISPLAYABLE_H
