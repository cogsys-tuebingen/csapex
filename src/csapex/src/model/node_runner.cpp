/// HEADER
#include <csapex/model/node_runner.h>

/// PROJECT
#include <csapex/model/node_worker.h>
#include <csapex/scheduling/scheduler.h>
#include <csapex/scheduling/task.h>
#include <csapex/utility/assert.h>

using namespace csapex;

NodeRunner::NodeRunner(NodeWorkerPtr worker)
    : worker_(worker), scheduler_(nullptr)
{
    tick_ = std::make_shared<Task>(std::string("tick ") + worker->getUUID().getFullName(),
                                   std::bind(&NodeWorker::tick, worker),
                                   this);
    prepare_ = std::make_shared<Task>(std::string("prepare ") + worker->getUUID().getFullName(),
                                      std::bind(&NodeWorker::prepareForNextProcess, worker),
                                      this);
    process_ = std::make_shared<Task>(std::string("process ") + worker->getUUID().getFullName(),
                                      std::bind(&NodeWorker::processMessages, worker),
                                      this);
    check_ = std::make_shared<Task>(std::string("check ") + worker->getUUID().getFullName(),
                                    std::bind(&NodeWorker::checkTransitions, worker),
                                    this);
}

NodeRunner::~NodeRunner()
{
    for(boost::signals2::connection& c : connections_) {
        c.disconnect();
    }
    connections_.clear();

    if(scheduler_) {
//        detach();
    }
}

void NodeRunner::reset()
{
    worker_->reset();
}

void NodeRunner::assignToScheduler(Scheduler *scheduler)
{
    std::unique_lock<std::mutex> lock(mutex_);

    apex_assert_hard(scheduler_ == nullptr);

    scheduler_ = scheduler;

    scheduler_->add(this, remaining_tasks_);
    remaining_tasks_.clear();

    for(boost::signals2::connection& c : connections_) {
        c.disconnect();
    }
    connections_.clear();

    // node tasks
    auto cmp = worker_->messages_processed.connect([this]() {
        schedule(prepare_);
    });
    connections_.push_back(cmp);

    auto ct = worker_->tickRequested.connect([this]() {
        schedule(tick_);
    });
    connections_.push_back(ct);

    auto cp = worker_->processRequested.connect([this]() {
        schedule(process_);
    });
    connections_.push_back(cp);

    auto ctr = worker_->checkTransitionsRequested.connect([this]() {
        schedule(check_);
    });
    connections_.push_back(ctr);

    // generic task
    auto cg = worker_->executionRequested.connect([this](std::function<void()> cb) {
        schedule(std::make_shared<Task>("anonymous", cb));
    });
    connections_.push_back(cg);
}

void NodeRunner::schedule(TaskPtr task)
{
    std::unique_lock<std::mutex> lock(mutex_);
    remaining_tasks_.push_back(task);

    if(scheduler_) {
        for(TaskPtr t : remaining_tasks_) {
            scheduler_->schedule(t);
        }
        remaining_tasks_.clear();
    }
}

void NodeRunner::detach()
{
    std::unique_lock<std::mutex> lock(mutex_);

    if(scheduler_) {
        auto t = scheduler_->remove(this);
        remaining_tasks_.insert(remaining_tasks_.end(), t.begin(), t.end());
        scheduler_ = nullptr;
    }
}

UUID NodeRunner::getUUID() const
{
    return worker_->getUUID();
}

void NodeRunner::setError(const std::string &msg)
{
    std::cerr << "error happened: " << msg << std::endl;
    worker_->setError(true, msg);
}
