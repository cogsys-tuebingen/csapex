#include "gtest/gtest.h"

#include <csapex/utility/subprocess.h>

#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/interprocess_condition.hpp>
#include <cstdlib>
#include <scoped_allocator>
#include <boost/interprocess/containers/string.hpp>
#include <thread>
#include <condition_variable>

using namespace csapex;

class SharedMemoryTest : public ::testing::Test
{
protected:
    SharedMemoryTest()
    {
    }

    virtual ~SharedMemoryTest()
    {
        // You can do clean-up work that doesn't throw exceptions here.
    }

    // If the constructor and destructor are not enough for setting up
    // and cleaning up each test, you can define the following methods:

    void SetUp() override
    {
        // Code here will be called immediately after the constructor (right
        // before each test).
    }

    void TearDown() override
    {
        // Code here will be called immediately after each test (right
        // before the destructor).
        if(worker_thread.joinable()) {
            worker_thread.join();
        }
    }

protected:
    std::thread worker_thread;
};

struct ShmBlock
{
    ShmBlock(bool can_process)
        : can_process(can_process)
    {
    }

    boost::interprocess::interprocess_mutex m;
    boost::interprocess::interprocess_condition message_available;
    boost::interprocess::interprocess_condition message_processed;

    bool can_process;
};

struct Foo
{
    Foo()
        : value(0)
    {

    }
    int value;
};

template <template <typename> class Allocator = std::allocator>
struct MessageT
{
    template <typename T>
    using ScopedAllocator = typename std::scoped_allocator_adaptor<Allocator<T>>;

    MessageT(int v, const ScopedAllocator<uint8_t>& alloc = ScopedAllocator<uint8_t>())
        : alloc_(alloc),
          foo_alloc_(alloc),
          string_alloc_(alloc),
          int_values(alloc_), float_values(alloc_),
          string_p(alloc),
          raw_ptr(foo_alloc_.allocate(1))
    {
        int_values.push_back(v);
        float_values.push_back(v*0.5);

        raw_ptr->value = v * 20.0;

        std::string string = std::string("foobarbaz") + std::to_string(v);
        string_p.assign(string.begin(), string.end());
    }

    ~MessageT()
    {
        foo_alloc_.deallocate(raw_ptr, 1);
    }

    using shared_string = typename boost::interprocess::basic_string<char, std::char_traits<char>, ScopedAllocator<char>>;

    ScopedAllocator<uint8_t>alloc_;
    ScopedAllocator<Foo> foo_alloc_;
    ScopedAllocator<shared_string> string_alloc_;

    std::vector<int, ScopedAllocator<int>> int_values;
    std::vector<float, ScopedAllocator<float>> float_values;


    shared_string string_p;

    typename Allocator<Foo>::pointer raw_ptr;
};

//using Message = MessageT<ShmAllocator>;
using Message = MessageT<ShmAllocator>;


TEST_F(SharedMemoryTest, MessagesCanBeSentToSubprocess)
{
    using namespace boost::interprocess;
    //Remove shared memory on construction and destruction
    struct shm_remove
    {
        shm_remove() {  remove(); }
        ~shm_remove(){  remove(); }
        void remove()
        {
            shared_memory_object::remove("csapex::test_node_0::shm");
        }
    } remover;

    //Create a managed shared memory segment
    managed_shared_memory segment(create_only, "csapex::test_node_0::shm", 65536);

    ShmBlock* shm = segment.construct<ShmBlock>("shm")(false);
    ASSERT_NE(nullptr, shm);

    const ShmAllocator<int> alloc_inst (segment.get_segment_manager());

    pid_t pid = fork();

    if (pid == 0)
    {
        // CHILD
        {
            scoped_lock<interprocess_mutex> lock(shm->m);

            // wait for a message
            while(!shm->can_process) {
                shm->message_available.wait(lock);
            }

            //Get buffer local address from handle
            std::pair<Message*, managed_shared_memory::size_type> msg_data
                    = segment.find<Message> ("in/msg_0");
            Message* msg = msg_data.first;
            if(msg) {
                if(msg->int_values.size() > 0 && msg->int_values.at(0) == 23) {
                    segment.construct<Message>
                            ("out/msg_0")
                            (42, alloc_inst);
                }

            } else {
                std::cout << "child: no message received" << std::endl;
            }

            shm->message_processed.notify_all();
        }
        std::quick_exit(0);

    } else {
        // PARENT

        // notify child process
        {
            scoped_lock<interprocess_mutex> lock(shm->m);

            // create shm marker block

            // send message to child process
            Message *msg = segment.construct<Message>
                    ("in/msg_0")
                    (23, alloc_inst);
            ASSERT_NE(nullptr, msg);
            ASSERT_EQ("foobarbaz23", msg->string_p);
            ASSERT_NEAR(23*20, msg->raw_ptr->value, 0.001);

            shm->can_process = true;

            shm->message_available.notify_all();

            shm->message_processed.wait(lock);

            Message* res = segment.find<Message> ("out/msg_0").first;
            ASSERT_NE(nullptr, res);
            ASSERT_EQ(42, res->int_values.at(0));
            ASSERT_NEAR(42*0.5, res->float_values.at(0), 0.001);
            ASSERT_NEAR(42*20, res->raw_ptr->value, 0.001);
            ASSERT_EQ("foobarbaz42", res->string_p);

            //Deallocate previously allocated memory
            segment.destroy<Message>("in/msg_0");
            segment.destroy<Message>("out/msg_0");
        }

        segment.destroy<ShmBlock>("shm");

        // Check memory has been freed
        ASSERT_EQ(nullptr, segment.find<Message>("in/msg_0").first);
    }

}

TEST_F(SharedMemoryTest, EmptySubprocess)
{
    Subprocess sp("test");
    sp.fork([](){});

    SUCCEED();
}

TEST_F(SharedMemoryTest, SubprocessChannelSendsMessage)
{
    Subprocess sp("test");

    ASSERT_FALSE(sp.out.hasMessage());

    sp.fork([&sp](){
        sp.out.write({SubprocessChannel::MessageType::PARAMETER_UPDATE, "done"});
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    ASSERT_TRUE(sp.out.hasMessage());

    ASSERT_EQ(SubprocessChannel::MessageType::PARAMETER_UPDATE, sp.out.read().type);
}

TEST_F(SharedMemoryTest, WritingAndReadingIsSynchronizedInChild)
{
    Subprocess sp("test");
    sp.fork([&sp](){
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        sp.out.write({SubprocessChannel::MessageType::PARAMETER_UPDATE, "done"});
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        sp.out.write({SubprocessChannel::MessageType::PROCESS, "done"});
    });

    ASSERT_EQ(SubprocessChannel::MessageType::PARAMETER_UPDATE, sp.out.read().type);
    ASSERT_EQ(SubprocessChannel::MessageType::PROCESS, sp.out.read().type);
}

TEST_F(SharedMemoryTest, WritingAndReadingIsSynchronizedInParent)
{
    Subprocess sp("test");
    sp.fork([&sp](){
        sp.out.write({SubprocessChannel::MessageType::PARAMETER_UPDATE, "done"});
        sp.out.write({SubprocessChannel::MessageType::PROCESS, "done"});
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    ASSERT_EQ(SubprocessChannel::MessageType::PARAMETER_UPDATE, sp.out.read().type);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    ASSERT_EQ(SubprocessChannel::MessageType::PROCESS, sp.out.read().type);
}


TEST_F(SharedMemoryTest, ReadingTwiceBlocks)
{
    Subprocess sp("test");

    sp.fork([&sp](){
        sp.out.write({SubprocessChannel::MessageType::PROCESS, "done"});
        sp.out.write({SubprocessChannel::MessageType::PARAMETER_UPDATE, "param1"});
    });

    std::mutex m;
    std::condition_variable wait;

    bool thread_has_read_from_channel = false;
    SubprocessChannel::MessageType thread_result = SubprocessChannel::MessageType::NONE;

    worker_thread = std::thread([&](){
        std::unique_lock<std::mutex> lock(m);
        wait.wait(lock);
        thread_result = sp.out.read().type;
        thread_has_read_from_channel = true;
    });

    {
        SubprocessChannel::Message msg = sp.out.read();

        ASSERT_EQ(SubprocessChannel::MessageType::PROCESS, msg.type);

        wait.notify_all();

        // the thread must be blocked now
        std::this_thread::sleep_for(std::chrono::milliseconds(5));

        ASSERT_FALSE(thread_has_read_from_channel);

        ASSERT_EQ(SubprocessChannel::MessageType::PROCESS, msg.type);
        ASSERT_EQ("done", msg.toString());
    }

    worker_thread.join();

    thread_has_read_from_channel = false;

    ASSERT_EQ(SubprocessChannel::MessageType::PARAMETER_UPDATE, thread_result);
}



TEST_F(SharedMemoryTest, SubprocessChannelCommunication)
{
    Subprocess sp("test");

    sp.fork([&sp](){
        ASSERT_EQ(SubprocessChannel::MessageType::PARAMETER_UPDATE, sp.in.read().type);
        ASSERT_EQ(SubprocessChannel::MessageType::PARAMETER_UPDATE, sp.in.read().type);
        ASSERT_EQ(SubprocessChannel::MessageType::PARAMETER_UPDATE, sp.in.read().type);
        ASSERT_EQ(SubprocessChannel::MessageType::PARAMETER_UPDATE, sp.in.read().type);

        auto msg = sp.in.read();
        ASSERT_EQ(SubprocessChannel::MessageType::PROCESS, msg.type);
        ASSERT_EQ("process", msg.toString());

        sp.out.write({SubprocessChannel::MessageType::PROCESS, "done"});
        sp.out.write({SubprocessChannel::MessageType::PARAMETER_UPDATE, "param1"});
        sp.out.write({SubprocessChannel::MessageType::PARAMETER_UPDATE, "param2"});
        sp.out.write({SubprocessChannel::MessageType::PARAMETER_UPDATE, "param3"});
    });

    sp.in.write({SubprocessChannel::MessageType::PARAMETER_UPDATE, "msg1"});
    sp.in.write({SubprocessChannel::MessageType::PARAMETER_UPDATE, "msg2"});
    sp.in.write({SubprocessChannel::MessageType::PARAMETER_UPDATE, "msg3"});
    sp.in.write({SubprocessChannel::MessageType::PARAMETER_UPDATE, "msg4"});
    sp.in.write({SubprocessChannel::MessageType::PROCESS, "process"});

    {
        auto msg = sp.out.read();
        ASSERT_EQ(SubprocessChannel::MessageType::PROCESS, msg.type);
        ASSERT_EQ("done", msg.toString());
    }

    ASSERT_EQ(SubprocessChannel::MessageType::PARAMETER_UPDATE, sp.out.read().type);
    ASSERT_EQ(SubprocessChannel::MessageType::PARAMETER_UPDATE, sp.out.read().type);
    ASSERT_EQ(SubprocessChannel::MessageType::PARAMETER_UPDATE, sp.out.read().type);
}



TEST_F(SharedMemoryTest, SequentialWriteIn)
{
    Subprocess sp("test");

    sp.fork([&sp](){
        {
        SubprocessChannel::Message m = sp.in.read();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        ASSERT_EQ(SubprocessChannel::MessageType::PARAMETER_UPDATE, m.type);
        }
        {
        SubprocessChannel::Message m = sp.in.read();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        ASSERT_EQ(SubprocessChannel::MessageType::PARAMETER_UPDATE, m.type);
        }
        {
        SubprocessChannel::Message m = sp.in.read();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        ASSERT_EQ(SubprocessChannel::MessageType::PARAMETER_UPDATE, m.type);
        }
        {
        SubprocessChannel::Message m = sp.in.read();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        ASSERT_EQ(SubprocessChannel::MessageType::PARAMETER_UPDATE, m.type);
        }

        sp.out.write({SubprocessChannel::MessageType::PROCESS, "done"});
    });

    sp.in.write({SubprocessChannel::MessageType::PARAMETER_UPDATE, "msg1"});
    sp.in.write({SubprocessChannel::MessageType::PARAMETER_UPDATE, "msg2"});
    sp.in.write({SubprocessChannel::MessageType::PARAMETER_UPDATE, "msg3"});
    sp.in.write({SubprocessChannel::MessageType::PARAMETER_UPDATE, "msg4"});

    {
        auto msg = sp.out.read();
        ASSERT_EQ(SubprocessChannel::MessageType::PROCESS, msg.type);
        ASSERT_EQ("done", msg.toString());
    }
}


TEST_F(SharedMemoryTest, ExceptionDoesNotStopSubprocessChannelCommunication)
{
    Subprocess sp("test");

    sp.fork([&sp](){
        ASSERT_EQ(SubprocessChannel::MessageType::PARAMETER_UPDATE, sp.in.read().type);
        throw std::runtime_error("foo");
    });

    sp.in.write({SubprocessChannel::MessageType::PARAMETER_UPDATE, "msg1"});
    ASSERT_EQ(SubprocessChannel::MessageType::CHILD_EXIT, sp.out.read().type);
}


TEST_F(SharedMemoryTest, SubprocessLoopTerminates)
{
    {
        Subprocess sp("test");
        sp.fork([&sp](){
            while(sp.isActive()) {
                sp.in.read();
                sp.out.write({SubprocessChannel::MessageType::PARAMETER_UPDATE, "answer"});
            }
        });

        sp.in.write({SubprocessChannel::MessageType::PARAMETER_UPDATE, "msg1"});
        ASSERT_EQ(SubprocessChannel::MessageType::PARAMETER_UPDATE, sp.out.read().type);
        sp.in.write({SubprocessChannel::MessageType::PARAMETER_UPDATE, "msg1"});
        ASSERT_EQ(SubprocessChannel::MessageType::PARAMETER_UPDATE, sp.out.read().type);
        sp.in.write({SubprocessChannel::MessageType::PARAMETER_UPDATE, "msg1"});
        ASSERT_EQ(SubprocessChannel::MessageType::PARAMETER_UPDATE, sp.out.read().type);
    }
}

TEST_F(SharedMemoryTest, SigtermInChildDoesNotStopSubprocessChannelCommunication)
{
    Subprocess sp("test");

    sp.fork([&sp](){
        ASSERT_EQ(SubprocessChannel::MessageType::PARAMETER_UPDATE, sp.in.read().type);
        raise(SIGTERM);
    });

    sp.in.write({SubprocessChannel::MessageType::PARAMETER_UPDATE, "msg1"});

    auto msg = sp.out.read();
    ASSERT_EQ(SubprocessChannel::MessageType::CHILD_EXIT, msg.type);
    std::string str = std::to_string(SIGTERM);
    ASSERT_EQ(str.c_str(), msg.toString());
}


TEST_F(SharedMemoryTest, SegfaultInChildDoesNotStopSubprocessChannelCommunication)
{
    Subprocess sp("test");
    sp.fork([&sp](){
        ASSERT_EQ(SubprocessChannel::MessageType::PARAMETER_UPDATE, sp.in.read().type);
        raise(SIGSEGV);
    });

    sp.in.write({SubprocessChannel::MessageType::PARAMETER_UPDATE, "msg1"});

    auto msg = sp.out.read();
    ASSERT_EQ(SubprocessChannel::MessageType::CHILD_EXIT, msg.type);
    std::string str = std::to_string(SIGSEGV);
    ASSERT_EQ(str.c_str(), msg.toString());
}

TEST_F(SharedMemoryTest, MessageHandleActsAsRAII)
{
    Subprocess sp("test");

    ASSERT_FALSE(sp.out.hasMessage());

    sp.fork([&sp](){
        bool as_expected = true;
        for(std::size_t i = 0; i < 5; ++i){
            SubprocessChannel::Message message = sp.in.read();

            // sleep a little after reading, expect that message's data does not change
            std::this_thread::sleep_for(std::chrono::milliseconds(5));

            std::string expected_msg(std::to_string(i));
            if(expected_msg != message.toString()) {
                as_expected = false;
                std::cerr << "not equal: " << i << " " << message.data << std::endl;
            }
        }

        if(as_expected) {
            sp.out.write({SubprocessChannel::MessageType::PROCESS, "done"});
        } else {
            sp.out.write({SubprocessChannel::MessageType::SHUTDOWN, "done"});
        }
    });

    for(std::size_t i = 0; i < 5; ++i){
        std::string msg(std::to_string(i));
        sp.in.write({SubprocessChannel::MessageType::PARAMETER_UPDATE, msg});
    }

    ASSERT_EQ(SubprocessChannel::MessageType::PROCESS, sp.out.read().type);
}

