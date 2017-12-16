#include "gtest/gtest.h"

#include <csapex/utility/slim_signal.hpp>
#include <csapex/utility/delegate_bind.h>
#include <boost/signals2.hpp>
#include <type_traits>

namespace csapex
{

struct Base
{
};
struct Child : public Base
{
};

bool global_called = false;
void global_callback(int, Base*)
{
    global_called = true;
}
void global_callback_child(int, Child*)
{
    global_called = true;
}
void global_callback_fewer(int)
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
        member_called_child = false;
        member_called_fewer = false;
    }

    virtual void TearDown() override {
    }

public:
    void member_callback(int, Base*)
    {
        member_called_1 = true;
    }
    void member_callback2(int, Base*)
    {
        member_called_2 = true;
    }
    void member_callback_child(int, Child*)
    {
        member_called_child = true;
    }
    void member_callback_fewer(int)
    {
        member_called_fewer = true;
    }

    bool member_called_1;
    bool member_called_2;
    bool member_called_child;
    bool member_called_fewer;
};

TEST_F(SlimSignalsTest, Constructor)
{
    boost::signals2::signal<void(int, Base*)> boost_sig;
    slim_signal::Signal<void(int, Base*)> slim_sig;

    ASSERT_TRUE(std::is_constructible<slim_signal::Signal<void(int, Base*)>>::value);
    ASSERT_FALSE(std::is_move_constructible<slim_signal::Signal<void(int, Base*)>>::value);
    ASSERT_FALSE(std::is_copy_constructible<slim_signal::Signal<void(int, Base*)>>::value);
    ASSERT_FALSE(std::is_copy_assignable<slim_signal::Signal<void(int, Base*)>>::value);
}

TEST_F(SlimSignalsTest, Connecting)
{
    int i = 0;
    Base foo;

    /// BOOST
    {
        boost::signals2::signal<void(int, Base*)> boost_sig;
        bool boost_called = false;
        boost_sig.connect([&](int, Base*) {
            boost_called = true;
        });

        boost_sig(i, &foo);
        ASSERT_TRUE(boost_called);
    }

    /// SLIM
    slim_signal::Signal<void(int, Base*)> slim_sig;

    {
        global_called = false;
        slim_sig.connect(delegate::Delegate<void(int, Base*)>(global_callback));

        slim_sig(i, &foo);
        ASSERT_TRUE(global_called);
    }

    {
        member_called_1 = false;
        slim_sig.connect(delegate::bind(&SlimSignalsTest::member_callback, this));

        slim_sig(i, &foo);
        ASSERT_TRUE(member_called_1);
    }

    {
        member_called_1 = false;
        slim_sig.connect(delegate::Delegate<void(int, Base*)>(this, &SlimSignalsTest::member_callback2));

        slim_sig(i, &foo);
        ASSERT_TRUE(member_called_2);
    }

    {
        bool slim_called = false;
        auto cb = [&](int, Base*) {
            slim_called = true;
        };
        std::function<void(int, Base*)> d = cb;

        slim_sig.connect(d);

        slim_sig(i, &foo);
        ASSERT_TRUE(slim_called);
    }

    {
        bool slim_called = false;
        slim_sig.connect([&](int, Base*) {
            slim_called = true;
        });

        slim_sig(i, &foo);
        ASSERT_TRUE(slim_called);
    }
}

TEST_F(SlimSignalsTest, UpcastingWorks)
{
    int i = 0;
    Child foo;

    /// SLIM
    slim_signal::Signal<void(int, Child*)> slim_sig;

    {
        global_called = false;
        slim_sig.connect(delegate::Delegate<void(int, Base*)>(global_callback));

        slim_sig(i, &foo);
        ASSERT_TRUE(global_called);
    }

    {
        member_called_1 = false;
        slim_sig.connect(delegate::bind(&SlimSignalsTest::member_callback, this));

        slim_sig(i, &foo);
        ASSERT_TRUE(member_called_1);
    }

    {
        member_called_2 = false;
        slim_sig.connect(delegate::Delegate<void(int, Base*)>(this, &SlimSignalsTest::member_callback2));

        slim_sig(i, &foo);
        ASSERT_TRUE(member_called_2);
    }

    {
        bool slim_called = false;
        auto cb = [&](int, Base*) {
            slim_called = true;
        };
        std::function<void(int, Base*)> d = cb;

        slim_sig.connect(d);

        slim_sig(i, &foo);
        ASSERT_TRUE(slim_called);
    }

    {
        bool slim_called = false;
        slim_sig.connect([&](int, Base*) {
            slim_called = true;
        });

        slim_sig(i, &foo);
        ASSERT_TRUE(slim_called);
    }
}



TEST_F(SlimSignalsTest, ConnectionToFewerArguments)
{
    int i = 0;
    Child foo;

    /// SLIM
    slim_signal::Signal<void(int, Child*)> slim_sig;

#if 0
    {
        global_called = false;
        slim_sig.connect(delegate::Delegate<void(int)>(global_callback_fewer));

        slim_sig(i, &foo);
        ASSERT_TRUE(global_called);
    }

    {
        member_called_fewer = false;
        slim_sig.connect(delegate::bind(&SlimSignalsTest::member_callback_fewer, this));

        slim_sig(i, &foo);
        ASSERT_TRUE(member_called_fewer);
    }

    {
        member_called_fewer = false;
        slim_sig.connect(delegate::Delegate<void(int)>(this, &SlimSignalsTest::member_callback_fewer));

        slim_sig(i, &foo);
        ASSERT_TRUE(member_called_fewer);
    }
#endif
    //test all here
    //also test connection to observable signal!
    {
        bool slim_called = false;
        slim_sig.connect([&](int) {
            slim_called = true;
        });

        slim_sig(i, &foo);
        ASSERT_TRUE(slim_called);
    }

    {
        bool slim_called = false;
        auto cb = [&](int) {
            slim_called = true;
        };
        std::function<void(int)> d = cb;

        slim_sig.connect(d);

        slim_sig(i, &foo);
        ASSERT_TRUE(slim_called);
    }

    {
        bool slim_called = false;
        auto cb = [&]() {
            slim_called = true;
        };
        std::function<void()> d = cb;

        slim_sig.connect(d);

        slim_sig(i, &foo);
        ASSERT_TRUE(slim_called);
    }

}

TEST_F(SlimSignalsTest, MultipleConnections)
{
    int i = 0;
    Base foo;

    /// BOOST
    {
        boost::signals2::signal<void(int, Base*)> boost_sig;

        bool boost_called_1 = false;
        bool boost_called_2 = false;
        boost_sig.connect([&](int, Base*) {
            boost_called_1 = true;
        });
        boost_sig.connect([&](int, Base*) {
            boost_called_2 = true;
        });

        boost_sig(i, &foo);
        ASSERT_TRUE(boost_called_1);
        ASSERT_TRUE(boost_called_2);
    }

    /// SLIM
    {
        slim_signal::Signal<void(int, Base*)> slim_sig;

        bool slim_called_1 = false;
        bool slim_called_2 = false;
        slim_sig.connect([&](int, Base*) {
            slim_called_1 = true;
        });
        slim_sig.connect([&](int, Base*) {
            slim_called_2 = true;
        });

        slim_sig(i, &foo);
        ASSERT_TRUE(slim_called_1);
        ASSERT_TRUE(slim_called_2);
    }
}

TEST_F(SlimSignalsTest, Disconnections)
{
    int i = 0;
    Base foo;

    /// BOOST
    {
        boost::signals2::signal<void(int, Base*)> boost_sig;

        bool called_1 = false;
        bool called_2 = false;
        auto c = boost_sig.connect([&](int, Base*) {
            called_1 = true;
        });
        boost_sig.connect([&](int, Base*) {
            called_2 = true;
        });

        boost_sig(i, &foo);
        ASSERT_TRUE(called_1);
        ASSERT_TRUE(called_2);

        c.disconnect();
        called_1 = false;
        called_2 = false;

        boost_sig(i, &foo);

        ASSERT_FALSE(called_1);
        ASSERT_TRUE(called_2);
    }

    /// SLIM

    {
        slim_signal::Signal<void(int, Base*)> slim_sig;

        bool called_1 = false;
        bool called_2 = false;
        slim_signal::Connection c = slim_sig.connect([&](int, Base*) {
            called_1 = true;
        });
        slim_sig.connect([&](int, Base*) {
            called_2 = true;
        });

        slim_sig(i, &foo);
        ASSERT_TRUE(called_1);
        ASSERT_TRUE(called_2);

        c.disconnect();
        called_1 = false;
        called_2 = false;

        slim_sig(i, &foo);

        ASSERT_FALSE(called_1);
        ASSERT_TRUE(called_2);
    }
    {
        slim_signal::Signal<void(int, Base*)> slim_sig;

        member_called_1 = false;
        member_called_2 = false;
        slim_signal::Connection c = slim_sig.connect(delegate::Delegate<void(int, Base*)>(this, &SlimSignalsTest::member_callback));
        slim_sig.connect(delegate::Delegate<void(int, Base*)>(this, &SlimSignalsTest::member_callback2));

        slim_sig(i, &foo);
        ASSERT_TRUE(member_called_1);
        ASSERT_TRUE(member_called_2);

        c.disconnect();
        member_called_1 = false;
        member_called_2 = false;

        slim_sig(i, &foo);

        //        std::abort();

        ASSERT_FALSE(member_called_1);
        ASSERT_TRUE(member_called_2);
    }
}

TEST_F(SlimSignalsTest, ScopedConnections)
{
    int i = 0;
    Base foo;

    /// BOOST
    {
        boost::signals2::signal<void(int, Base*)> boost_sig;

        bool called_1 = false;
        bool called_2 = false;
        {
            boost::signals2::scoped_connection c(boost_sig.connect([&](int, Base*) {
                called_1 = true;
            }));
        }

        boost::signals2::scoped_connection c(boost_sig.connect([&](int, Base*) {
            called_2 = true;
        }));

        boost_sig(i, &foo);

        ASSERT_FALSE(called_1);
        ASSERT_TRUE(called_2);
    }

    /// SLIM
    slim_signal::Signal<void(int, Base*)> slim_sig;

    {
        bool called_1 = false;
        bool called_2 = false;
        {
            slim_signal::ScopedConnection c = slim_sig.connect([&](int, Base*) {
                called_1 = true;
            });
        }
        slim_signal::ScopedConnection c = slim_sig.connect([&](int, Base*) {
            called_2 = true;
        });

        slim_sig(i, &foo);

        ASSERT_FALSE(called_1);
        ASSERT_TRUE(called_2);
    }
    {
        member_called_1 = false;
        member_called_2 = false;
        {
            slim_signal::ScopedConnection c = slim_sig.connect(delegate::Delegate<void(int, Base*)>(this, &SlimSignalsTest::member_callback));
        }
        slim_signal::ScopedConnection c = slim_sig.connect(delegate::Delegate<void(int, Base*)>(this, &SlimSignalsTest::member_callback2));

        slim_sig(i, &foo);

        ASSERT_FALSE(member_called_1);
        ASSERT_TRUE(member_called_2);
    }
}

TEST_F(SlimSignalsTest, Chaining)
{
    int i = 0;
    Base foo;

    /// BOOST
    {
        boost::signals2::signal<void(int, Base*)> boost_sig1;
        boost::signals2::signal<void(int, Base*)> boost_sig2;

        bool called = false;

        boost_sig2.connect(boost_sig1);
        boost_sig1.connect([&](int, Base*) {
            called = true;
        });

        boost_sig2(i, &foo);

        ASSERT_TRUE(called);
    }

    /// SLIM

    {
        slim_signal::Signal<void(int, Base*)> slim_sig1;
        slim_signal::Signal<void(int, Base*)> slim_sig2;

        bool called = false;

        slim_sig2.connect(slim_sig1);
        slim_sig1.connect([&](int, Base*) {
            called = true;
        });

        slim_sig2(i, &foo);

        ASSERT_TRUE(called);
    }

    {
        Child foo;

        slim_signal::ObservableSignal<void(int, Child*)> slim_sig1;
        slim_signal::Signal<void(int, Child*)> slim_sig2;
        slim_signal::Signal<void(int, Child*)> slim_sig3;
        slim_signal::Signal<void(int, Child*)> slim_sig4;

        slim_signal::Signal<void(int, Child*)> slim_sig_last;

        bool first_connected = false;
        bool last_disconnected = false;
        bool called = false;

        slim_sig1.first_connected.connect([&](){
            first_connected = true;
        });
        slim_sig1.last_disconnected.connect([&](){
            last_disconnected = true;
        });

        ASSERT_FALSE(first_connected);
        ASSERT_FALSE(last_disconnected);

        auto c1 = slim_sig1.connect(slim_sig2);

        ASSERT_TRUE(first_connected); first_connected = false;
        ASSERT_FALSE(last_disconnected); last_disconnected = false;

        auto c2 = slim_sig1.connect(slim_sig_last);

        ASSERT_FALSE(first_connected); first_connected = false;
        ASSERT_FALSE(last_disconnected); last_disconnected = false;

        slim_sig2.connect(slim_sig3);

        ASSERT_FALSE(first_connected); first_connected = false;
        ASSERT_FALSE(last_disconnected); last_disconnected = false;

        slim_sig3.connect(slim_sig4);

        ASSERT_FALSE(first_connected); first_connected = false;
        ASSERT_FALSE(last_disconnected); last_disconnected = false;

        slim_sig4.connect([&](int, Child*) {
            called = true;
        });

        slim_sig1(i, &foo);

        ASSERT_TRUE(called);

        ASSERT_FALSE(first_connected); first_connected = false;
        ASSERT_FALSE(last_disconnected); last_disconnected = false;

        c2.disconnect();

        ASSERT_FALSE(first_connected); first_connected = false;
        ASSERT_FALSE(last_disconnected); last_disconnected = false;

        c1.disconnect();

        ASSERT_FALSE(first_connected); first_connected = false;
        ASSERT_TRUE(last_disconnected); last_disconnected = false;
    }
}

TEST_F(SlimSignalsTest, ChainingUpcast)
{
    int i;
    Child foo;

    slim_signal::ObservableSignal<void(int, Child*)> slim_sig1;
    slim_signal::Signal<void(int, Base*)> slim_sig2;

    bool called = false;

    slim_sig1.connect(slim_sig2);
    slim_sig2.connect([&](int, Base*) {
        called = true;
    });

    slim_sig1(i, &foo);

    ASSERT_TRUE(called);
}

TEST_F(SlimSignalsTest, Deletion)
{
    int i = 0;
    Base foo;

    /// BOOST
    {
        auto boost_sig = std::make_shared<boost::signals2::signal<void(int, Base*)> >();

        bool called = false;

        boost::signals2::scoped_connection sc(boost_sig->connect([&](int, Base*) {
            called = true;
        }));

        (*boost_sig)(i, &foo);

        ASSERT_TRUE(called);

        boost_sig.reset();

        // ASSERT not to crash
        sc.disconnect();
    }

    /// SLIM

    {
        auto slim_sig = std::make_shared<slim_signal::Signal<void(int, Base*)> >();

        bool called = false;

        slim_signal::ScopedConnection sc(slim_sig->connect([&](int, Base*) {
            called = true;
        }));

        (*slim_sig)(i, &foo);

        ASSERT_TRUE(called);

        slim_sig.reset();

        // ASSERT not to crash
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

//        ASSERT_FALSE(called);

//        boost_sig.reset();

//        // ASSERT not to crash
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

//        ASSERT_FALSE(called);

//        slim_sig.reset();

//        // ASSERT not to crash
//        sc.disconnect();
//    }
//}

}


