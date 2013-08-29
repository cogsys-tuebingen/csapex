#ifndef MATH_HPP
#define MATH_HPP

namespace cv_math {
/**
 * @brief Greatest common divident.
 * @param a - first number
 * @param b - second number
 * @return  gcd
 */
inline int gcd(int a, int b)
{
    int temp;
    while (b != 0) {
        temp = b;
        b = a % b;
        a = temp;
    }
    return a;
}
}
#endif // MATH_HPP
