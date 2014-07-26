/// HEADER
#include <csapex/model/node_worker.h>

/// COMPONENT
#include <csapex/model/node.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/utility/timer.h>
#include <csapex/utility/thread.h>
#include <csapex/core/settings.h>

/// SYSTEM
#include <QThread>

using namespace csapex;

NodeWorker::NodeWorker(Node* node)
    : node_(node), private_thread_(NULL),
      timer_history_pos_(-1),
      thread_initialized_(false), paused_(false), stop_(false)
{
    timer_history_.resize(Settings::timer_history_length_);
    apex_assert_hard(timer_history_.size() == Settings::timer_history_length_);
    apex_assert_hard(timer_history_.capacity() == Settings::timer_history_length_);

    apex_assert_hard(node_);

    timer_ = new QTimer();
    setTickFrequency(DEFAULT_FREQUENCY);

    QObject::connect(timer_, SIGNAL(timeout()), this, SLOT(tick()));
}

NodeWorker::~NodeWorker()
{
}

void NodeWorker::makeThread()
{
    if(!private_thread_) {
        private_thread_ = new QThread;
        connect(private_thread_, SIGNAL(finished()), private_thread_, SLOT(deleteLater()));

        moveToThread(private_thread_);

        private_thread_->start();
    }
}

void NodeWorker::stop()
{
    QMutexLocker lock(&stop_mutex_);


    QObject::disconnect(private_thread_);
    stop_ = true;

    pause(false);

    if(private_thread_) {
        private_thread_->quit();
        //        private_thread_->wait(1000);
        //        if(private_thread_->isRunning()) {
        //            std::cout << "terminate thread" << std::endl;
        //            private_thread_->terminate();
        //        }
    }
}

void NodeWorker::pause(bool pause)
{
    QMutexLocker lock(&pause_mutex_);
    paused_ = pause;
    continue_.wakeAll();
}

void NodeWorker::killExecution()
{
    // TODO: implement
}

void NodeWorker::forwardMessage(Connectable *s)
{
    {
        pause_mutex_.lock();
        while(paused_) {
            continue_.wait(&pause_mutex_);
        }
        pause_mutex_.unlock();
    }

    QMutexLocker lock(&stop_mutex_);
    if(stop_) {
        return;
    }

    Input* source = dynamic_cast<Input*> (s);
    apex_assert_hard(source);

    if(node_->isEnabled()) {
        Q_EMIT messagesReceived();

        forwardMessageSynchronized(source);
    }
}

void NodeWorker::addInput(Input *source)
{
    clearInput(source);

    connect(source, SIGNAL(enabled(bool)), this, SLOT(checkInputs()));
}

void NodeWorker::checkInputs()
{
    for(int i = 0; i < node_->countInputs(); ++i) {
        Input* source = node_->getInput(i);
        if(!source->isEnabled() && source->isBlocked()) {
            source->free();
            clearInput(source);
        }
    }
}

void NodeWorker::clearInput(Input *source)
{
    QMutexLocker lock(&message_mutex_);
    has_msg_[source] = false;
}

void NodeWorker::removeInput(Input *source)
{
    QMutexLocker lock(&message_mutex_);
    std::map<Input*, bool>::iterator it = has_msg_.find(source);

    if(it != has_msg_.end()) {
        has_msg_.erase(it);
    }
}

void NodeWorker::forwardMessageSynchronized(Input *source)
{
    typedef std::pair<Input*, bool> PAIR;
    bool can_process = true;

    {
        QMutexLocker lock(&message_mutex_);
        std::vector< boost::shared_ptr<QMutexLocker> > locks;

        // forbid async to change
        Q_FOREACH(const PAIR& pair, has_msg_) {
            Input* cin = pair.first;
            locks.push_back(cin->lockAsync());
        }

        //apex_assert_hard(!has_msg_[source]);
        if(has_msg_[source] && !source->isAsync()) {
            std::cerr << "input @" << source->getUUID().getFullName() <<
                         ": assertion failed: !has_msg_[" << source->getUUID().getFullName() << "]" << std::endl;
            apex_assert_hard(false);
        }
        if(!source->isAsync()) {
            has_msg_[source] = true;
        }

        // check if all inputs have messages
        Q_FOREACH(const PAIR& pair, has_msg_) {
            Input* cin = pair.first;
            if(!has_msg_[cin]) {
                // connector doesn't have a message
                if(cin->isAsync()) {
                    // do not wait for this input
                } else if(cin->isOptional()) {
                    if(cin->isConnected()) {
                        // c is optional and connected, so we have to wait for a message
                        return;
                    } else {
                        // c is optional and not connected, so we can proceed
                        /* do nothing */
                    }
                } else {
                    // c is mandatory, so we have to wait for a message
                    return;
                }
            }
        }

        // everything has a message here

        // check for old messages
        bool had_old_message = false;
        int highest_seq_no = -1;
        UUID highest = UUID::NONE;
        Q_FOREACH(const PAIR& pair, has_msg_) {
            Input* cin = pair.first;

            if(has_msg_[cin] && !cin->isAsync()) {
                int s = cin->sequenceNumber();
                if(s > highest_seq_no) {
                    highest_seq_no = s;
                    highest = cin->getUUID();
                }
            }
        }
        Q_FOREACH(const PAIR& pair, has_msg_) {
            Input* cin = pair.first;
            if(cin == source) {
                continue;
            }

            if(has_msg_[cin] && !cin->isAsync() && cin->sequenceNumber() != highest_seq_no) {
                std::cerr << "input @" << source->getUUID().getFullName() <<
                             ": dropping old message @" << cin->getUUID().getFullName() << " with #" << cin->sequenceNumber() <<
                             " < #" << highest_seq_no << " @" << highest.getFullName() << std::endl;
                has_msg_[cin] = false;
                had_old_message = true;
                cin->free();
            }
        }

        // if a message was dropped we can already return
        if(had_old_message) {
            return;
        }

        // now all sequence numbers must be equal!
        Q_FOREACH(const PAIR& pair, has_msg_) {
            Input* cin = pair.first;

            if(has_msg_[cin] && !cin->isAsync()) {
                if(highest_seq_no != cin->sequenceNumber()) {
                    std::cerr << "input @" << cin->getUUID().getFullName() <<
                                 ": assertion failed: highest_seq_no (" << highest_seq_no << ") == cin->seq_no (" << cin->sequenceNumber() << ")" << std::endl;
                    apex_assert_hard(false);
                }
            }
        }

        // check if one is "NoMessage"
        Q_FOREACH(const PAIR& pair, has_msg_) {
            Input* cin = pair.first;

            if(has_msg_[cin] && !cin->isAsync()) {
                if(cin->isMessage<connection_types::NoMessage>()) {
                    can_process = false;
                }
            }
        }

        // set output sequence numbers
        for(int i = 0; i < node_->countOutputs(); ++i) {
            Output* out = node_->getOutput(i);
            out->setSequenceNumber(highest_seq_no);
        }

        // reset states
        Q_FOREACH(const PAIR& pair, has_msg_) {
            Input* cin = pair.first;

            if(has_msg_[cin] && !cin->isAsync()) {
                has_msg_[cin] = false;
            }
        }
    }

    Timer::Ptr t(new Timer(node_->getUUID()));
    node_->useTimer(t.get());
    if(can_process){
        //boost::shared_ptr<QMutexLocker> lock = node_->getParamLock();
        try {
            node_->process();

        }  catch(const std::exception& e) {
            node_->setError(true, e.what());
        } catch(const std::string& s) {
            node_->setError(true, "Uncatched exception (string) exception: " + s);
        } catch(...) {
            throw;
        }

    }
    t->finish();

    // send the messages
    sendMessages();

    if(++timer_history_pos_ >= (int) Settings::timer_history_length_) {
        timer_history_pos_ = 0;
    }
    timer_history_[timer_history_pos_] = t;

    // reset all edges
    Q_FOREACH(const PAIR& pair, has_msg_) {
        Input* cin = pair.first;
        cin->free();
    }

    Q_EMIT messageProcessed();
}

void NodeWorker::sendMessages()
{
    for(int i = 0; i < node_->countOutputs(); ++i) {
        Output* out = node_->getOutput(i);
        out->sendMessages();
    }
}

void NodeWorker::setTickFrequency(double f)
{
    if(timer_->isActive()) {
        timer_->stop();
    }
    if(f == 0.0) {
        return;
    }
    timer_->setInterval(1000. / f);
    timer_->setSingleShot(false);
    timer_->start();
}

void NodeWorker::tick()
{
    {
        pause_mutex_.lock();
        while(paused_) {
            continue_.wait(&pause_mutex_);
        }
        pause_mutex_.unlock();

        QMutexLocker lock(&stop_mutex_);
        if(stop_) {
            return;
        }
    }


    if(!thread_initialized_) {
        thread::set_name(node_->getUUID().getShortName().c_str());
        thread_initialized_ = true;
    }

    if(node_->isEnabled()) {
        //boost::shared_ptr<QMutexLocker> lock = node_->getParamLock();
        node_->tick();

        // if there is a message: send!
        bool has_msg = false;
        for(int i = 0; i < node_->countOutputs(); ++i) {
            Output* out = node_->getOutput(i);
            if(out->hasMessage()) {
                has_msg = true;
            }
        }
        if(has_msg) {
            sendMessages();
        }
    }
}

void NodeWorker::checkParameters()
{
    Parameterizable::ChangedParameterList changed_params = node_->getChangedParameters();

    if(!changed_params.empty()) {
        for(Parameterizable::ChangedParameterList::iterator it = changed_params.begin(); it != changed_params.end();) {
            try {
                it->second(it->first);

            } catch(const std::exception& e) {
                it = changed_params.erase(it);
                throw;
            }

            it = changed_params.erase(it);
        }
    }
}

void NodeWorker::triggerError(bool e, const std::string &what)
{
    node_->setError(e, what);
}

