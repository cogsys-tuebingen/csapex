#include "gtest/gtest.h"

#include <csapex/utility/slim_signal.hpp>
#include <csapex/utility/delegate_bind.h>
#include <boost/signals2.hpp>
#include <type_traits>

namespace csapex
{

struct Foo
{
};

bool global_called = false;
void global_callback(int, Foo*)
{
    global_called = true;
}

class SlimSignalsTest : public ::testing::Test
{
protected:
    SlimSignalsTest()
    {
    }

    virtual ~SlimSignalsTest() {
    }

    virtual void SetUp() override {
        member_called_1 = false;
        member_called_2 = false;
    }

    virtual void TearDown() override {
    }

public:
    void member_callback(int, Foo*)
    {
        member_called_1 = true;
    }
    void member_callback2(int, Foo*)
    {
        member_called_2 = true;
    }

    bool member_called_1;
    bool member_called_2;
};

TEST_F(SlimSignalsTest, Constructor)
{
    boost::signals2::signal<void(int, Foo*)> boost_sig;
    slim_signal::Signal<void(int, Foo*)> slim_sig;

    EXPECT_TRUE(std::is_constructible<slim_signal::Signal<void(int, Foo*)>>::value);
    EXPECT_FALSE(std::is_move_constructible<slim_signal::Signal<void(int, Foo*)>>::value);
    EXPECT_FALSE(std::is_copy_constructible<slim_signal::Signal<void(int, Foo*)>>::value);
    EXPECT_FALSE(std::is_copy_assignable<slim_signal::Signal<void(int, Foo*)>>::value);
}

TEST_F(SlimSignalsTest, Connecting)
{
    int i = 0;
    Foo foo;

    /// BOOST
    {
        boost::signals2::signal<void(int, Foo*)> boost_sig;
        bool boost_called = false;
        boost_sig.connect([&](int, Foo*) {
            boost_called = true;
        });

        boost_sig(i, &foo);
        EXPECT_TRUE(boost_called);
    }

    /// SLIM
    slim_signal::Signal<void(int, Foo*)> slim_sig;

    {
        global_called = false;
        slim_sig.connect(delegate::Delegate<void(int, Foo*)>(global_callback));

        slim_sig(i, &foo);
        EXPECT_TRUE(global_called);
    }

    {
        member_called_1 = false;
        slim_sig.connect(delegate::bind(&SlimSignalsTest::member_callback, this));

        slim_sig(i, &foo);
        EXPECT_TRUE(member_called_1);
    }

    {
        member_called_1 = false;
        slim_sig.connect(delegate::Delegate<void(int, Foo*)>(this, &SlimSignalsTest::member_callback));

        slim_sig(i, &foo);
        EXPECT_TRUE(member_called_1);
    }

    {
        bool slim_called = false;
        auto cb = [&](int, Foo*) {
            slim_called = true;
        };
        std::function<void(int, Foo*)> d = cb;

        slim_sig.connect(d);

        slim_sig(i, &foo);
        EXPECT_TRUE(slim_called);
    }

    {
        bool slim_called = false;
        slim_sig.connect([&](int, Foo*) {
            slim_called = true;
        });

        slim_sig(i, &foo);
        EXPECT_TRUE(slim_called);
    }
}

TEST_F(SlimSignalsTest, MultipleConnections)
{
    int i = 0;
    Foo foo;

    /// BOOST
    {
        boost::signals2::signal<void(int, Foo*)> boost_sig;

        bool boost_called_1 = false;
        bool boost_called_2 = false;
        boost_sig.connect([&](int, Foo*) {
            boost_called_1 = true;
        });
        boost_sig.connect([&](int, Foo*) {
            boost_called_2 = true;
        });

        boost_sig(i, &foo);
        EXPECT_TRUE(boost_called_1);
        EXPECT_TRUE(boost_called_2);
    }

    /// SLIM
    {
        slim_signal::Signal<void(int, Foo*)> slim_sig;

        bool slim_called_1 = false;
        bool slim_called_2 = false;
        slim_sig.connect([&](int, Foo*) {
            slim_called_1 = true;
        });
        slim_sig.connect([&](int, Foo*) {
            slim_called_2 = true;
        });

        slim_sig(i, &foo);
        EXPECT_TRUE(slim_called_1);
        EXPECT_TRUE(slim_called_2);
    }
}

TEST_F(SlimSignalsTest, Disconnections)
{
    int i = 0;
    Foo foo;

    /// BOOST
    {
        boost::signals2::signal<void(int, Foo*)> boost_sig;

        bool called_1 = false;
        bool called_2 = false;
        auto c = boost_sig.connect([&](int, Foo*) {
            called_1 = true;
        });
        boost_sig.connect([&](int, Foo*) {
            called_2 = true;
        });

        boost_sig(i, &foo);
        EXPECT_TRUE(called_1);
        EXPECT_TRUE(called_2);

        c.disconnect();
        called_1 = false;
        called_2 = false;

        boost_sig(i, &foo);

        EXPECT_FALSE(called_1);
        EXPECT_TRUE(called_2);
    }

    /// SLIM

    {
        slim_signal::Signal<void(int, Foo*)> slim_sig;

        bool called_1 = false;
        bool called_2 = false;
        slim_signal::Connection c = slim_sig.connect([&](int, Foo*) {
            called_1 = true;
        });
        slim_sig.connect([&](int, Foo*) {
            called_2 = true;
        });

        slim_sig(i, &foo);
        EXPECT_TRUE(called_1);
        EXPECT_TRUE(called_2);

        c.disconnect();
        called_1 = false;
        called_2 = false;

        slim_sig(i, &foo);

        EXPECT_FALSE(called_1);
        EXPECT_TRUE(called_2);
    }
    {
        slim_signal::Signal<void(int, Foo*)> slim_sig;

        member_called_1 = false;
        member_called_2 = false;
        slim_signal::Connection c = slim_sig.connect(delegate::Delegate<void(int, Foo*)>(this, &SlimSignalsTest::member_callback));
        slim_sig.connect(delegate::Delegate<void(int, Foo*)>(this, &SlimSignalsTest::member_callback2));

        slim_sig(i, &foo);
        EXPECT_TRUE(member_called_1);
        EXPECT_TRUE(member_called_2);

        c.disconnect();
        member_called_1 = false;
        member_called_2 = false;

        slim_sig(i, &foo);

//        std::abort();

        EXPECT_FALSE(member_called_1);
        EXPECT_TRUE(member_called_2);
    }
}

TEST_F(SlimSignalsTest, ScopedConnections)
{
    int i = 0;
    Foo foo;

    /// BOOST
    {
        boost::signals2::signal<void(int, Foo*)> boost_sig;

        bool called_1 = false;
        bool called_2 = false;
        {
            boost::signals2::scoped_connection c(boost_sig.connect([&](int, Foo*) {
                called_1 = true;
            }));
        }

        boost::signals2::scoped_connection c(boost_sig.connect([&](int, Foo*) {
            called_2 = true;
        }));

        boost_sig(i, &foo);

        EXPECT_FALSE(called_1);
        EXPECT_TRUE(called_2);
    }

    /// SLIM
    slim_signal::Signal<void(int, Foo*)> slim_sig;

    {
        bool called_1 = false;
        bool called_2 = false;
        {
            slim_signal::ScopedConnection c = slim_sig.connect([&](int, Foo*) {
                called_1 = true;
            });
        }
        slim_signal::ScopedConnection c = slim_sig.connect([&](int, Foo*) {
            called_2 = true;
        });

        slim_sig(i, &foo);

        EXPECT_FALSE(called_1);
        EXPECT_TRUE(called_2);
    }
    {
        member_called_1 = false;
        member_called_2 = false;
        {
            slim_signal::ScopedConnection c = slim_sig.connect(delegate::Delegate<void(int, Foo*)>(this, &SlimSignalsTest::member_callback));
        }
        slim_signal::ScopedConnection c = slim_sig.connect(delegate::Delegate<void(int, Foo*)>(this, &SlimSignalsTest::member_callback2));

        slim_sig(i, &foo);

        EXPECT_FALSE(member_called_1);
        EXPECT_TRUE(member_called_2);
    }
}

TEST_F(SlimSignalsTest, Chaining)
{
    int i = 0;
    Foo foo;

    /// BOOST
    {
        boost::signals2::signal<void(int, Foo*)> boost_sig1;
        boost::signals2::signal<void(int, Foo*)> boost_sig2;

        bool called = false;

        boost_sig2.connect(boost_sig1);
        boost_sig1.connect([&](int, Foo*) {
            called = true;
        });

        boost_sig2(i, &foo);

        EXPECT_TRUE(called);
    }

    /// SLIM

    {
        slim_signal::Signal<void(int, Foo*)> slim_sig1;
        slim_signal::Signal<void(int, Foo*)> slim_sig2;

        bool called = false;

        slim_sig2.connect(slim_sig1);
        slim_sig1.connect([&](int, Foo*) {
            called = true;
        });

        slim_sig2(i, &foo);

        EXPECT_TRUE(called);
    }
}

TEST_F(SlimSignalsTest, Deletion)
{
    int i = 0;
    Foo foo;

    /// BOOST
    {
        auto boost_sig = std::make_shared<boost::signals2::signal<void(int, Foo*)> >();

        bool called = false;

        boost::signals2::scoped_connection sc(boost_sig->connect([&](int, Foo*) {
            called = true;
        }));

        (*boost_sig)(i, &foo);

        EXPECT_TRUE(called);

        boost_sig.reset();

        // EXPECT not to crash
        sc.disconnect();
    }

    /// SLIM

    {
        auto slim_sig = std::make_shared<slim_signal::Signal<void(int, Foo*)> >();

        bool called = false;

        slim_signal::ScopedConnection sc(slim_sig->connect([&](int, Foo*) {
            called = true;
        }));

        (*slim_sig)(i, &foo);

        EXPECT_TRUE(called);

        slim_sig.reset();

        // EXPECT not to crash
        sc.disconnect();
    }
}

//TEST_F(SlimSignalsTest, RecursiveDeletion)
//{
//    int i = 0;
//    Foo foo;

//    /// BOOST
//    {
//        auto boost_sig = std::make_shared<boost::signals2::signal<void(int, Foo*)> >();

//        bool called = false;

//        boost::signals2::scoped_connection sc(boost_sig->connect([&](int, Foo*) {
//            boost_sig->disconnect_all_slots();
//        }));

//        boost::signals2::scoped_connection sc2(boost_sig->connect([&](int, Foo*) {
//            called = true;
//        }));

//        (*boost_sig)(i, &foo);

//        EXPECT_FALSE(called);

//        boost_sig.reset();

//        // EXPECT not to crash
//        sc.disconnect();
//    }

//    /// SLIM

//    {
//        auto slim_sig = std::make_shared<slim_signal::Signal<void(int, Foo*)> >();

//        bool called = false;

//        slim_signal::ScopedConnection sc(slim_sig->connect([&](int, Foo*) {
//            slim_sig->disconnectAll();
//        }));
//        slim_signal::ScopedConnection sc2(slim_sig->connect([&](int, Foo*) {
//            called = true;
//        }));

//        (*slim_sig)(i, &foo);

//        EXPECT_FALSE(called);

//        slim_sig.reset();

//        // EXPECT not to crash
//        sc.disconnect();
//    }
//}

}


