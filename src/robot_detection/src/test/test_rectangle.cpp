#include <tools/rectangle_cluster.h>

int fails = 0;
int tests = 0;

#undef assert
#define assert(test){\
    if(!test){\
        std::cout << __FUNCTION__ << " in line " << __LINE__ << " failed" << std::endl;\
        fails ++;\
    }\
    tests++;\
}

void finish()
{
    if(fails == 0)
        std::cout << "all " << tests << " tests passed" << std::endl;
    else
        std::cout << fails << " / " << tests << " tests failed" << std::endl;

    tests = 0;
    fails = 0;
}

void test1()
{
    RectangleCluster cluster;

    /*
     *    A------+
     *    |  E - |  -  -  -  - +
     *    +------B             ,
     *
     *       ,           C---------+
     *                   |     ,   |
     *       +  -  -  -  |  -  F   |
     *                   +-------- D
     *
     *
     *    A------------------------+
     *    |  E                     |
     *    |      B                 |
     *    |                        |
     *    |              C         |
     *    |                        |
     *    |                    F   |
     *    +----------------------- D
     */

    cv::Point a(10, 10);
    cv::Point b(90, 90);
    cv::Point c(100, 100);
    cv::Point d(190, 190);
    cv::Point e(30, 30);
    cv::Point f(140, 140);

    cluster.integrate(cv::Rect(a, b));
    assert(cluster.contains(cv::Rect(a, b)));
    assert(!cluster.contains(cv::Rect(c, d)));

    cluster.integrate(cv::Rect(c, d));

    assert(cluster.contains(cv::Rect(a, b)));
    assert(cluster.contains(cv::Rect(c, d)));

    assert(cluster.covers(cv::Rect(a, b)));
    assert(cluster.covers(cv::Rect(c, d)));
    assert(cluster.covers(cv::Rect(c, f)));
    assert(cluster.covers(cv::Rect(a, e)));
    assert(cluster.covers(cv::Rect(e, b)));
    assert(!cluster.covers(cv::Rect(e, f)));

    assert(cluster.conflicts(cv::Rect(a, e)));
    assert(!cluster.conflicts(cv::Rect(b, c)));
    assert(cluster.conflicts(cv::Rect(e, f)));

    cluster.integrate(cv::Rect(e, f));
    assert(!cluster.contains(cv::Rect(a, b)));
    assert(!cluster.contains(cv::Rect(c, d)));
    assert(!cluster.contains(cv::Rect(e, f)));
    assert(cluster.contains(cv::Rect(a, d)));

    finish();
}

void test2()
{
    RectangleCluster cluster;

    /*
     *    A------+
     *    |  E - |  -  -  -  - +
     *    +------B             ,
     *
     *       ,           C---------+
     *                   |     ,   |
     *       +  -  -  -  |  -  F   |
     *                   +-------- D
     *
     *
     *    A--------------------+
     *    |  E                 |
     *    |      B             |
     *    |                    |
     *    |              C     |
     *    |                    |
     *    +------------------- F
     *                            D
     */

    cv::Point a(10, 10);
    cv::Point b(90, 90);
    cv::Point c(100, 100);
    cv::Point d(190, 190);
    cv::Point e(30, 30);
    cv::Point f(140, 140);

    cluster.integrate(cv::Rect(a, b));
    assert(cluster.contains(cv::Rect(a, b)));
    assert(!cluster.contains(cv::Rect(e, f)));

    cluster.integrate(cv::Rect(e, f));

    assert(!cluster.contains(cv::Rect(a, b)));
    assert(!cluster.contains(cv::Rect(e, f)));
    assert(cluster.contains(cv::Rect(a, f)));
    assert(cluster.covers(cv::Rect(a, b)));
    assert(cluster.covers(cv::Rect(e, f)));

    cluster.integrate(cv::Rect(c, d));
    assert(!cluster.contains(cv::Rect(a, b)));
    assert(!cluster.contains(cv::Rect(c, d)));
    assert(!cluster.contains(cv::Rect(e, f)));
    assert(cluster.contains(cv::Rect(a, d)));

    finish();
}

int main(int argc, char** argv)
{
    std::cout << "testsuite: rectangle cluster" << std::endl;

    test1();
    test2();
}
