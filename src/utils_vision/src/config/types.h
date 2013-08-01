#ifndef TYPES_H
#define TYPES_H

/// PROJECT
#include <common/global.hpp>

/// SYSTEM
#include <exception>
#include <string>
#include <map>

namespace Types
{
template <class T>
class Parameter : public T
{
public:
    typedef typename T::ID SubType;
    class IllegalException : public std::exception {};

    static SubType read(const std::string& type);
    static std::string write(SubType type);
    static std::string write(int type);

protected:
    static Parameter<T>& instance() {
        static Parameter<T> instance;
        return instance;
    }

    std::string writeName(int type) {
        return write(static_cast<SubType>(type));
    }
    std::string writeName(SubType type) {
        return map[type];
    }

    SubType readName(const std::string& type);

    Parameter() {
        T::initialize(map);
    }
    std::map<SubType, std::string> map;
};

/**
 * @brief The Strategy class represents symbolic constants
 */
class StrategyImp
{
public:
    enum ID {
        FIRST, NAIVE=0, BIN, BAG, BOW,
        COUNT
    };
protected:
    void initialize(std::map<ID, std::string>& map);
};

typedef Parameter<StrategyImp> Strategy;
}
#endif // TYPES_H
