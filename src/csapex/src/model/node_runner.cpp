/// HEADER
#include <csapex/model/node_runner.h>

/// PROJECT
#include <csapex/model/node_handle.h>
#include <csapex/model/node_worker.h>
#include <csapex/model/node.h>
#include <csapex/scheduling/scheduler.h>
#include <csapex/scheduling/task.h>
#include <csapex/utility/assert.h>
#include <csapex/model/node_state.h>
#include <csapex/utility/thread.h>
#include <csapex/model/subgraph_node.h>

/// SYSTEM
#include <memory>
#include <iostream>

using namespace csapex;

NodeRunner::NodeRunner(NodeWorkerPtr worker)
    : worker_(worker), scheduler_(nullptr),
      paused_(false), stepping_(false), can_step_(0), step_done_(false),
      guard_(-1),
      waiting_for_execution_(false),
      waiting_for_step_(false)
{
    NodeHandlePtr handle = worker_->getNodeHandle();

    handle->getNodeState()->max_frequency_changed->connect([this](){
        NodeHandlePtr handle = worker_->getNodeHandle();
        max_frequency_ = handle->getNodeState()->getMaximumFrequency();

        if(max_frequency_ > 0.0) {
            handle->getRate().setFrequency(max_frequency_);
        } else {
            handle->getRate().setFrequency(0.0);
        }
    });
    max_frequency_ = handle->getNodeState()->getMaximumFrequency();
    handle->getRate().setFrequency(max_frequency_);

    check_parameters_ = std::make_shared<Task>(std::string("check parameters for ") + handle->getUUID().getFullName(),
                                               std::bind(&NodeWorker::checkParameters, worker),
                                               0,
                                               this);
    execute_ = std::make_shared<Task>(std::string("check ") + handle->getUUID().getFullName(),
                                      [this]()
    {
        execute();
    }, 0, this);
}

NodeRunner::~NodeRunner()
{
    stopObserving();

    if(scheduler_) {
        detach();
    }

    guard_ = 0xDEADBEEF;
}

void NodeRunner::measureFrequency()
{
    if(worker_->isProcessingEnabled()) {
        worker_->getNodeHandle()->getRate().tick();
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
        scheduleProcess();
    });


    observe(worker_->messages_processed, [this](){
        measureFrequency();
        step_done_ = true;
        //TRACE worker_->getNode()->ainfo << "end step" << std::endl;
        end_step();
    });

    // parameter change
    observe(worker_->getNodeHandle()->parameters_changed, [this]() {
        schedule(check_parameters_);
    });

    // processing enabled change
    observe(worker_->getNodeHandle()->getNodeState()->enabled_changed, [this]() {
        if(!worker_->getNodeHandle()->getNodeState()->isEnabled()) {
            waiting_for_execution_ = false;
        }
    });

    schedule(check_parameters_);


    // generic task
    observe(worker_->getNodeHandle()->execution_requested, [this](std::function<void()> cb) {
        schedule(std::make_shared<Task>("anonymous", cb, 0, this));
    });
}


Scheduler* NodeRunner::getScheduler() const
{
    return scheduler_;
}

void NodeRunner::scheduleProcess()
{
    apex_assert_hard(guard_ == -1);
    if(!paused_) {
        bool source = worker_->getNodeHandle()->isSource();
        if(!source || !stepping_ || can_step_) {
            //execute_->setPriority(std::max<long>(0, worker_->getSequenceNumber()));
            //if(worker_->canExecute()) {
                if(!waiting_for_execution_) {
                    schedule(execute_);
                }
            //}
        }
    }
}

void NodeRunner::execute()
{
    if(stepping_ && can_step_ <= 0) {
        return;
    }

    apex_assert_hard(guard_ == -1);
    if(worker_->canExecute()) {
        if(max_frequency_ > 0.0) {
            const Rate& rate = worker_->getNodeHandle()->getRate();
            double f = rate.getEffectiveFrequency();
            if(f > max_frequency_) {
                auto next_process = rate.endOfCycle();

                auto now = std::chrono::system_clock::now();

                if(next_process > now) {
                    scheduleDelayed(execute_, next_process);
                    waiting_for_execution_ = true;
                    return;
                }
            }
        }

        waiting_for_execution_ = false;

        worker_->getNodeHandle()->getRate().startCycle();

        if(stepping_) {
            apex_assert_hard(can_step_);
        }
        can_step_--;

        if(!worker_->execute()) {
            //TRACE worker_->getNode()->ainfo << "execute failed" << std::endl;
            can_step_++;
        }
    } else {
        can_step_++;
        waiting_for_execution_ = false;
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

void NodeRunner::scheduleDelayed(TaskPtr task, std::chrono::system_clock::time_point time)
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    scheduler_->scheduleDelayed(task, time);
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
    if(!paused_) {
        scheduleProcess();
    }
}

bool NodeRunner::canStartStepping() const
{
    if(auto subgraph = std::dynamic_pointer_cast<SubgraphNode>(worker_->getNode())) {
        // if the node is an iterating sub graph, we need to wait until the current iteration is done...
        return subgraph->isIterating();
    }
    return true;
}

void NodeRunner::setSteppingMode(bool stepping)
{
    bool can_start_stepping = true;
    if(stepping) {
        if(!canStartStepping()){
            can_start_stepping = false;
            waiting_for_step_ = true;
            wait_for_step_connection_ = worker_->messages_processed.connect([this]() {
                stepping_enabled();
            });
        }
    }

    stepping_ = stepping;
    can_step_ = 0;

    if(stepping_) {
        if(can_start_stepping) {
            stepping_enabled();
        }

    } else {
        scheduleProcess();
    }
}

void NodeRunner::step()
{
    if(waiting_for_step_) {
        wait_for_step_connection_.disconnect();
    }

    can_step_++;

    step_done_ = false;
    begin_step();

    bool source = worker_->getNodeHandle()->isSource();

    bool can_process = worker_->isProcessingEnabled();
    can_process &= source || worker_->getNodeHandle()->hasConnectionsIncoming();

    if(!can_process) {
        //TRACE worker_->getNode()->ainfo << "cannot step" << std::endl;
        step_done_ = true;
        end_step();
        return;
    }

    //TRACE worker_->getNode()->ainfo << "step" << std::endl;
    if(source) {
        scheduleProcess();
    }
}

bool NodeRunner::isStepping() const
{
    return stepping_;
}

bool NodeRunner::isStepDone() const
{
    return step_done_;
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
