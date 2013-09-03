#ifndef MESSAGES_H
#define MESSAGES_H
#include <iostream>
#include <sstream>
/**
 * @namespace This namespace contains method to generate console output.
 */
namespace msg_out {
/**
 * @brief Put error message onto console.
 * @param msg   the message
 * @param tag   the software components tag
 */
inline void error(const std::string& msg, const std::string &tag = "")
{

#if defined(__UNIX__) || defined(__APPLE__)
    std::cerr << "\033[1;31m " << "ERROR\t" << "\033[0m";
#else
    std::cerr << "ERROR\t";
#endif

    if(tag != "")
        std::cerr << tag << ": ";
    std::cerr << msg << std::endl;
}
/**
 * @brief Put warn message onto console.
 * @param msg   the  message
 * @param tag   the software components tag
 */
inline void warn(const std::string& msg, const std::string &tag = "")
{
#if defined(__UNIX__) || defined(__APPLE__)
    std::cout << "\033[1;33m " << "WARN\t" << "\033[0m";
#else
    std::cout << "WARN\t";
#endif

    if(tag != "")
        std::cout << tag << ": ";
    std::cout << msg << std::endl;
}
/**
 * @brief Put an info message onto console.
 * @param msg   the message
 * @param tag   the software components tag
 */
inline void info(const std::string& msg, const std::string &tag = "")
{

#if defined(__UNIX__) || defined(__APPLE__)
    std::cout << "\033[1;32m " << "INFO\t" << "\033[0m";
#else
    std::cout << "INFO\t";
#endif

    if(tag != "")
        std::cout << tag << ": ";

    std::cout << msg << std::endl;

}

/**
 * @brief Convert several data types to string.
 * @param   the value to convert
 */
template <class T>
inline std::string to_string(const T &value)
{
    std::stringstream s;
    s << value;
    return s.str();
}
}


#endif // MESSAGES_H
