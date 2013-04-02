/// HEADER
#include "db_strategy_decorator.h"

DatabaseStrategyDecorator::DatabaseStrategyDecorator(Ptr decorated)
    : DatabaseStrategyInterface(*decorated), decorated(decorated), TAG(0xDEADBEEF)
{
    decorated->getDatabase();
}

DatabaseStrategyDecorator::~DatabaseStrategyDecorator()
{

}

Database* DatabaseStrategyDecorator::getDatabase()
{
    return decorated->getDatabase();
}

void DatabaseStrategyDecorator::train(Frame::Ptr frame)
{
    decorated->train(frame);
}

void DatabaseStrategyDecorator::addValidationExample(Frame::Ptr frame)
{
    decorated->addValidationExample(frame);
}

MatchablePose* DatabaseStrategyDecorator::getPoseByAngle(const double yaw, int* index) const
{
    return decorated->getPoseByAngle(yaw, index);
}

void DatabaseStrategyDecorator::validate()
{
    decorated->validate();
}

bool DatabaseStrategyDecorator::load()
{
    return decorated->load();
}

bool DatabaseStrategyDecorator::loadConfig()
{
    return decorated->loadConfig();
}

bool DatabaseStrategyDecorator::save()
{
    return decorated->save();
}

void DatabaseStrategyDecorator::clear()
{
    decorated->clear();
}

MatchablePose* DatabaseStrategyDecorator::detect(Frame::Ptr frame, cv::Rect& out_roi, double* score_out)
{
    return decorated->detect(frame, out_roi, score_out);
}

void DatabaseStrategyDecorator::dumpReference(const std::string& path)
{
    decorated->dumpReference(path);
}
