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

using namespace csapex;

NodeRunner::NodeRunner(NodeWorkerPtr worker)
    : worker_(worker), scheduler_(nullptr),
      paused_(false), ticking_(false), is_source_(false), stepping_(false), can_step_(false),
      tick_thread_running_(false)
{
    NodeHandlePtr handle = worker_->getNodeHandle();
    NodePtr node = handle->getNode().lock();
    is_source_ = handle->isSource();
    ticking_ = node && std::dynamic_pointer_cast<TickableNode>(node);

    check_parameters_ = std::make_shared<Task>(std::string("check parameters for ") + handle->getUUID().getFullName(),
                                               std::bind(&NodeWorker::checkParameters, worker),
                                               this);
    check_transitions_ = std::make_shared<Task>(std::string("check ") + handle->getUUID().getFullName(),
                                                std::bind(&NodeWorker::checkTransitions, worker),
                                                this);

    if(ticking_) {
        tick_ = std::make_shared<Task>(std::string("tick ") + handle->getUUID().getFullName(),
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

    if(tick_thread_running_) {
        tick_thread_stop_ = true;
        ticking_thread_.join();
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
    worker_->getNodeHandle()->getNodeState()->setThread(scheduler->name(), scheduler->id());

    remaining_tasks_.clear();

    for(boost::signals2::connection& c : connections_) {
        c.disconnect();
    }
    connections_.clear();

    // node tasks
    auto ctr = worker_->checkTransitionsRequested.connect([this]() {
        schedule(check_transitions_);
    });
    connections_.push_back(ctr);

    // parameter change
    auto check = worker_->getNodeHandle()->parametersChanged.connect([this]() {
        schedule(check_parameters_);
    });
    connections_.push_back(check);

    schedule(check_parameters_);


    // generic task
    auto cg = worker_->getNodeHandle()->executionRequested.connect([this](std::function<void()> cb) {
            schedule(std::make_shared<Task>("anonymous", cb));
});
    connections_.push_back(cg);

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

void NodeRunner::tickLoop()
{
    NodePtr node =  worker_->getNode();
    auto ticker = std::dynamic_pointer_cast<TickableNode>(node);

    using std::chrono::system_clock;
    auto next_tick = system_clock::now();

    while(!tick_thread_stop_) {
        double f = ticker->getTickFrequency();
        std::chrono::milliseconds interval(int(1000.0 / f));

        bool immediate = ticker->isImmediate();

        if(!paused_) {
            if(!stepping_ || can_step_) {
                can_step_ = false;
                schedule(tick_);
            }
        }

        next_tick += interval;

        if(next_tick > system_clock::now() && !immediate) {
            std::this_thread::sleep_until(next_tick);
        }
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
