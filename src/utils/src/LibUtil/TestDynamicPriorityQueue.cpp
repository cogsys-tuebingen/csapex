/*
 * TestDynamicPriorityQueue.cpp
 *
 *  Created on: May 6, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

/// COMPONENT
#include "DynamicPriorityQueue.hpp"

/// SYSTEM
#include <assert.h>
#include <iostream>

template <typename V>
struct Holder {
    struct Comparator {
        bool operator() (const Holder<V>& lhs, const Holder<V>& rhs) const {
            return lhs.value < rhs.value;
        }
        bool operator() (const Holder<V>* lhs, const Holder<V>* rhs) const {
            return lhs->value < rhs->value;
        }
    };

    Holder(V v)
        : value(v)
    {}
    V value;
};

typedef Holder<int> H;

void test1()
{
    DynamicPrioritySet<H, H::Comparator> pq;

    pq.push(H(1));
    pq.push(H(2));
    pq.push(H(3));
    pq.push(H(4));

    assert(pq.top().value == 4);
    pq.pop();

    assert(pq.top().value == 3);
    pq.pop();

    assert(pq.top().value == 2);
    pq.pop();

    assert(pq.top().value == 1);
    pq.pop();
}

void test2()
{
    DynamicPrioritySet<H, H::Comparator> pq;

    pq.push(H(2));
    pq.push(H(4));
    pq.push(H(1));
    pq.push(H(3));

    assert(pq.top().value == 4);
    pq.pop();

    assert(pq.top().value == 3);
    pq.pop();

    assert(pq.top().value == 2);
    pq.pop();

    assert(pq.top().value == 1);
    pq.pop();
}


void test3()
{
    DynamicPrioritySet<H*, H::Comparator> pq;

    H best(4);

    pq.push(new H(2));
    pq.push(&best);
    pq.push(new H(1));
    pq.push(new H(3));

    pq.remove(&best);
    best.value = 0;
    pq.push(&best);

    assert(pq.top()->value == 3);
    pq.pop();

    assert(pq.top()->value == 2);
    pq.pop();

    assert(pq.top()->value == 1);
    pq.pop();

    assert(pq.top()->value == 0);
    pq.pop();
}

int main(int argc, char** argv) {
    test1();
    test2();
    test3();

    std::cout << "every test passed" << std::endl;
}
