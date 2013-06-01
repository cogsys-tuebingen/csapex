/// HEADER
#include "logger.h"

/// SYSTEM
#include <iostream>

Logger Logger::INFO(std::cout, "\033[1;32m[INFO] \033[0;32m", "\033[0m");
Logger Logger::WARN(std::cout, "\033[1;33m[WARN] \033[0;33m", "\033[0m");
Logger Logger::ERROR(std::cerr, "\033[1;31m[ERROR] \033[0;31m", "\033[0m");
Logger Logger::FATAL(std::cerr, "\033[1;31m[FATAL] ", "\033[0m");

Logger::Logger(std::ostream& out, const std::string& prefix, const std::string& suffix)
    : out_(out), prefix_(prefix), suffix_(suffix)
{
}

std::ostream& Logger::getOut()
{
    return out_ << prefix_;
}

void Logger::endl()
{
    out_ << suffix_ << std::endl;
}
