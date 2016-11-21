/// HEADER
#include <csapex/model/node_runner.h>

/// PROJECT
#include <csapex/model/node_handle.h>
#include <csapex/model/node_worker.h>
#include <csapex/model/tickable_node.h>
#include <csapex/scheduling/scheduler.h>
#include <csapex/scheduling/task.h>
#include <csapex/utility/assert.h>
#include <csapex/model/node_state.h>
#include <csapex/utility/thread.h>

/// SYSTEM
#include <memory>
#include <iostream>

using namespace csapex;

NodeRunner::NodeRunner(NodeWorkerPtr worker)
    : worker_(worker), scheduler_(nullptr),
      paused_(false), ticking_(false), is_source_(false), stepping_(false), can_step_(false),
      tick_thread_running_(false), guard_(-1)
{
    NodeHandlePtr handle = worker_->getNodeHandle();
    NodePtr node = handle->getNode().lock();
    is_source_ = handle->isSource();
    ticking_ = node && std::dynamic_pointer_cast<TickableNode>(node);

    check_parameters_ = std::make_shared<Task>(std::string("check parameters for ") + handle->getUUID().getFullName(),
                                               std::bind(&NodeWorker::checkParameters, worker),
                                               0,
                                               this);
    try_process_ = std::make_shared<Task>(std::string("check ") + handle->getUUID().getFullName(),
                                          [this]()
    {
        if(worker_->tryProcess()) {
            measureFrequency();
        }
    }, 0, this);

    if(ticking_) {
        tick_ = std::make_shared<Task>(std::string("tick ") + handle->getUUID().getFullName(),
                                       [this]() {
            tick();
        }, 0, this);
    }
}

NodeRunner::~NodeRunner()
{
    stopObserving();

    if(scheduler_) {
        //        detach();
    }

    stopTickThread();

    guard_ = 0xDEADBEEF;
}

void NodeRunner::measureFrequency()
{
    worker_->getNodeHandle()->getRate().tick();
}

void NodeRunner::reset()
{
    worker_->reset();
}

void NodeRunner::assignToScheduler(Scheduler *scheduler)
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);

    apex_assert_hard(scheduler_ == nullptr);
    apex_assert_hard(scheduler != nullptr);

    scheduler_ = scheduler;

    scheduler_->add(shared_from_this(), remaining_tasks_);
    worker_->getNodeHandle()->getNodeState()->setThread(scheduler->getName(), scheduler->id());

    remaining_tasks_.clear();

    stopObserving();

    // signals
    observe(scheduler->scheduler_changed, [this](){
        NodeHandlePtr nh = worker_->getNodeHandle();
        nh->getNodeState()->setThread(scheduler_->getName(), scheduler_->id());
    });

    // node tasks
    observe(worker_->try_process_changed, [this]() {
        try_process_->setPriority(worker_->getSequenceNumber());
        schedule(try_process_);
    });

    // parameter change
    observe(worker_->getNodeHandle()->parameters_changed, [this]() {
        schedule(check_parameters_);
    });

    schedule(check_parameters_);


    // generic task
    observe(worker_->getNodeHandle()->execution_requested, [this](std::function<void()> cb) {
                schedule(std::make_shared<Task>("anonymous", cb, 0, this));
            });

    if(ticking_ && !tick_thread_running_) {
        // TODO: get rid of this!
        ticking_thread_ = std::thread([this]() {
            csapex::thread::set_name((std::string("T") + worker_->getUUID().getShortName()).c_str());
            tick_thread_running_ = true;

            tick_thread_stop_ = false;
            tickLoop();

            tick_thread_running_ = false;
        });
    }
}


Scheduler* NodeRunner::getScheduler() const
{
    return scheduler_;
}

void NodeRunner::scheduleTick()
{
    if(!paused_) {
        if(!stepping_ || can_step_) {
            can_step_ = false;
            schedule(tick_);
        }
    }
}

void NodeRunner::tick()
{
    apex_assert_hard(guard_ == -1);

    bool success = worker_->tick();
    if(stepping_) {
        if(!success) {
            can_step_ = true;
        } else {
            end_step();
        }
    }

    if(success) {
        measureFrequency();

        NodeHandlePtr handle = worker_->getNodeHandle();
        NodePtr node = handle->getNode().lock();
        auto ticker = std::dynamic_pointer_cast<TickableNode>(node);

        if(ticker->isImmediate()) {
            scheduleTick();
        }
    }
}

void NodeRunner::tickLoop()
{
    NodePtr node =  worker_->getNode();
    auto ticker = std::dynamic_pointer_cast<TickableNode>(node);

    while(!tick_thread_stop_) {
        if(!ticker->isImmediate() || !tick_->isScheduled()) {
            scheduleTick();
        }

        ticker->keepUpRate();
    }
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

void NodeRunner::stopTickThread()
{
    if(tick_thread_running_) {
        tick_thread_stop_ = true;
        ticking_thread_.join();
    }
}

void NodeRunner::detach()
{
    stopTickThread();

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
