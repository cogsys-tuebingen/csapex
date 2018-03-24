#include "gtest/gtest.h"

#include <csapex/utility/subprocess.h>

#include <cstdlib>
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

struct Foo
{
    Foo()
        : value(0)
    {

    }
    int value;
};

TEST_F(SharedMemoryTest, EmptySubprocess)
{
    Subprocess sp("test");
    sp.fork([](){});
}

TEST_F(SharedMemoryTest, EmptySubprocessCanBeJoined)
{
    Subprocess sp("test");
    sp.fork([](){});

    ASSERT_EQ(0, sp.join());
}

/*
TEST_F(SharedMemoryTest, OutputCanBeGrabbed)
{
    for(int i = 0; i < 16; ++i) {
        Subprocess sp("test");
        sp.fork([](){
            std::cerr << "B";
            std::cout << "Foo" << std::endl;
            std::cerr << "ar";
        });

        sp.join();

        EXPECT_EQ("Foo\n", sp.getChildStdOut());
        EXPECT_EQ("Bar", sp.getChildStdErr());
    }
}*/


TEST_F(SharedMemoryTest, ReturnCode)
{
    Subprocess empty("test");
    empty.fork([](){});

    EXPECT_EQ(0, empty.join());

    Subprocess ret1("test");
    ret1.fork([]() -> int {
                  return 123;
              });

    EXPECT_EQ(123, ret1.join());
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
        sp.out.write({SubprocessChannel::MessageType::PROCESS_SYNC, "done"});
    });

    ASSERT_EQ(SubprocessChannel::MessageType::PARAMETER_UPDATE, sp.out.read().type);
    ASSERT_EQ(SubprocessChannel::MessageType::PROCESS_SYNC, sp.out.read().type);
}

TEST_F(SharedMemoryTest, WritingAndReadingIsSynchronizedInParent)
{
    Subprocess sp("test");
    sp.fork([&sp](){
        sp.out.write({SubprocessChannel::MessageType::PARAMETER_UPDATE, "done"});
        sp.out.write({SubprocessChannel::MessageType::PROCESS_SYNC, "done"});
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    ASSERT_EQ(SubprocessChannel::MessageType::PARAMETER_UPDATE, sp.out.read().type);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    ASSERT_EQ(SubprocessChannel::MessageType::PROCESS_SYNC, sp.out.read().type);
}


TEST_F(SharedMemoryTest, ReadingTwiceBlocks)
{
    Subprocess sp("test");

    sp.fork([&sp](){
        sp.out.write({SubprocessChannel::MessageType::PROCESS_SYNC, "done"});
        sp.out.write({SubprocessChannel::MessageType::PARAMETER_UPDATE, "param1"});
    });

    std::mutex m;
    bool worker_waiting = false;
    std::condition_variable worker_waiting_changed;

    bool message_read = false;
    std::condition_variable message_read_changed;

    bool thread_has_read_from_channel = false;
    SubprocessChannel::MessageType thread_result = SubprocessChannel::MessageType::NONE;

    worker_thread = std::thread([&](){
        std::unique_lock<std::mutex> lock(m);
        worker_waiting = true;
        worker_waiting_changed.notify_all();

        while(!message_read) {
            message_read_changed.wait(lock);
        }

        thread_result = sp.out.read().type;
        thread_has_read_from_channel = true;
    });

    {
        std::unique_lock<std::mutex> lock(m);
        while(!worker_waiting) {
            worker_waiting_changed.wait(lock);
        }

        SubprocessChannel::Message msg = sp.out.read();

        ASSERT_EQ(SubprocessChannel::MessageType::PROCESS_SYNC, msg.type);
        message_read = true;
        message_read_changed.notify_all();

        // the thread must be blocked now
        std::this_thread::sleep_for(std::chrono::milliseconds(5));

        ASSERT_FALSE(thread_has_read_from_channel);

        ASSERT_EQ(SubprocessChannel::MessageType::PROCESS_SYNC, msg.type);
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
        ASSERT_EQ(SubprocessChannel::MessageType::PROCESS_SYNC, msg.type);
        ASSERT_EQ("process", msg.toString());

        sp.out.write({SubprocessChannel::MessageType::PROCESS_SYNC, "done"});
        sp.out.write({SubprocessChannel::MessageType::PARAMETER_UPDATE, "param1"});
        sp.out.write({SubprocessChannel::MessageType::PARAMETER_UPDATE, "param2"});
        sp.out.write({SubprocessChannel::MessageType::PARAMETER_UPDATE, "param3"});
    });

    sp.in.write({SubprocessChannel::MessageType::PARAMETER_UPDATE, "msg1"});
    sp.in.write({SubprocessChannel::MessageType::PARAMETER_UPDATE, "msg2"});
    sp.in.write({SubprocessChannel::MessageType::PARAMETER_UPDATE, "msg3"});
    sp.in.write({SubprocessChannel::MessageType::PARAMETER_UPDATE, "msg4"});
    sp.in.write({SubprocessChannel::MessageType::PROCESS_SYNC, "process"});

    {
        auto msg = sp.out.read();
        ASSERT_EQ(SubprocessChannel::MessageType::PROCESS_SYNC, msg.type);
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

        sp.out.write({SubprocessChannel::MessageType::PROCESS_SYNC, "done"});
    });

    sp.in.write({SubprocessChannel::MessageType::PARAMETER_UPDATE, "msg1"});
    sp.in.write({SubprocessChannel::MessageType::PARAMETER_UPDATE, "msg2"});
    sp.in.write({SubprocessChannel::MessageType::PARAMETER_UPDATE, "msg3"});
    sp.in.write({SubprocessChannel::MessageType::PARAMETER_UPDATE, "msg4"});

    {
        auto msg = sp.out.read();
        ASSERT_EQ(SubprocessChannel::MessageType::PROCESS_SYNC, msg.type);
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
    ASSERT_EQ(SubprocessChannel::MessageType::CHILD_ERROR, sp.out.read().type);
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
    ASSERT_EQ(SubprocessChannel::MessageType::CHILD_SIGNAL, msg.type);
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
    ASSERT_EQ(SubprocessChannel::MessageType::CHILD_SIGNAL, msg.type);
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
            sp.out.write({SubprocessChannel::MessageType::PROCESS_SYNC, "done"});
        } else {
            sp.out.write({SubprocessChannel::MessageType::SHUTDOWN, "done"});
        }
    });

    for(std::size_t i = 0; i < 5; ++i){
        std::string msg(std::to_string(i));
        sp.in.write({SubprocessChannel::MessageType::PARAMETER_UPDATE, msg});
    }

    ASSERT_EQ(SubprocessChannel::MessageType::PROCESS_SYNC, sp.out.read().type);
}

