/// HEADER
#include <csapex/model/node_runner.h>

/// PROJECT
#include <csapex/model/node_worker.h>
#include <csapex/model/tickable_node.h>
#include <csapex/scheduling/scheduler.h>
#include <csapex/scheduling/task.h>
#include <csapex/utility/assert.h>
#include <csapex/model/node_state.h>

using namespace csapex;

NodeRunner::NodeRunner(NodeWorkerPtr worker)
    : worker_(worker), scheduler_(nullptr),
      paused_(false), ticking_(false), is_source_(false), stepping_(false), can_step_(false)
{
    NodePtr node = worker_->getNode().lock();
    is_source_ = worker->isSource();
    ticking_ = node && std::dynamic_pointer_cast<TickableNode>(node);
    if(ticking_) {
        tick_ = std::make_shared<Task>(std::string("tick ") + worker->getUUID().getFullName(),
                                       [this, worker]() {
            bool success = worker->tick();
            if(stepping_) {
                if(!success) {
                    can_step_ = true;
                } else {
                    end_step();
                }
            }
        }, this);
    }
    check_parameters_ = std::make_shared<Task>(std::string("check parameters for ") + worker->getUUID().getFullName(),
                                               std::bind(&NodeWorker::checkParameters, worker),
                                               this);
    prepare_ = std::make_shared<Task>(std::string("prepare ") + worker->getUUID().getFullName(),
                                      std::bind(&NodeWorker::prepareForNextProcess, worker),
                                      this);
    process_ = std::make_shared<Task>(std::string("process ") + worker->getUUID().getFullName(),
                                      std::bind(&NodeWorker::startProcessingMessages, worker),
                                      this);
    check_transitions_ = std::make_shared<Task>(std::string("check ") + worker->getUUID().getFullName(),
                                                std::bind(&NodeWorker::checkTransitions, worker, true),
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
    std::unique_lock<std::recursive_mutex> lock(mutex_);

    apex_assert_hard(scheduler_ == nullptr);

    scheduler_ = scheduler;

    scheduler_->add(this, remaining_tasks_);
    worker_->getNodeState()->setThread(scheduler->name(), scheduler->id());

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

    auto cp = worker_->processRequested.connect([this]() {
        schedule(process_);
    });
    connections_.push_back(cp);

    auto ctr = worker_->checkTransitionsRequested.connect([this]() {
        schedule(check_transitions_);
    });
    connections_.push_back(ctr);

    // tick?
    if(tick_) {
        auto ct = worker_->tickRequested.connect([this]() {
            if(!paused_) {
                if(!stepping_ || can_step_) {
                    can_step_ = false;
                    schedule(tick_);
                }
            }
        });
        connections_.push_back(ct);
    }

    // parameter change
    auto check = worker_->parametersChanged.connect([this]() {
        schedule(check_parameters_);
    });
    connections_.push_back(check);

    schedule(check_parameters_);


    // generic task
    auto cg = worker_->executionRequested.connect([this](std::function<void()> cb) {
            schedule(std::make_shared<Task>("anonymous", cb));
});
    connections_.push_back(cg);
}

void NodeRunner::schedule(TaskPtr task)
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
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
    std::unique_lock<std::recursive_mutex> lock(mutex_);

    if(scheduler_) {
        auto t = scheduler_->remove(this);
        remaining_tasks_.insert(remaining_tasks_.end(), t.begin(), t.end());
        scheduler_ = nullptr;
    }
}

bool NodeRunner::isPaused() const
{
    return paused_;
}

void NodeRunner::setPause(bool pause)
{
    paused_ = pause;
}

void NodeRunner::setSteppingMode(bool stepping)
{
    stepping_ = stepping;
    can_step_ = false;
}

void NodeRunner::step()
{
    if(is_source_ && worker_->isProcessingEnabled()) {
        begin_step();
        can_step_ = true;
    } else {
        can_step_ = false;
        end_step();
    }
}

bool NodeRunner::isStepping() const
{
    return stepping_;
}

bool NodeRunner::isStepDone() const
{
    if(!ticking_) {
        return true;
    }
    return !can_step_;
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
