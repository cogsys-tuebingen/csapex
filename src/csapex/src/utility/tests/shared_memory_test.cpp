#include "gtest/gtest.h"

#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/interprocess_condition.hpp>
#include <cstdlib>
#include <scoped_allocator>
#include <boost/interprocess/containers/string.hpp>

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
    }
};

template <typename T>
using ShmAllocator = boost::interprocess::allocator<T, boost::interprocess::managed_shared_memory::segment_manager>;

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
