#ifndef COLOR_HPP
#define COLOR_HPP

namespace color {
#define __HSV2RGB__(H, S, V, R, G, B) \
    { \
    double _h = H/60.; \
    int _hf = (int)floor(_h); \
    int _hi = ((int)_h)%6; \
    double _f = _h - _hf; \
    \
    double _p = V * (1. - S); \
    double _q = V * (1. - _f * S); \
    double _t = V * (1. - (1. - _f) * S); \
    \
    switch (_hi) \
    { \
    case 0: \
    R = 255.*V; G = 255.*_t; B = 255.*_p; \
    break; \
    case 1: \
    R = 255.*_q; G = 255.*V; B = 255.*_p; \
    break; \
    case 2: \
    R = 255.*_p; G = 255.*V; B = 255.*_t; \
    break; \
    case 3: \
    R = 255.*_p; G = 255.*_q; B = 255.*V; \
    break; \
    case 4: \
    R = 255.*_t; G = 255.*_p; B = 255.*V; \
    break; \
    case 5: \
    R = 255.*V; G = 255.*_p; B = 255.*_q; \
    break; \
} \
}

template <class Color>
inline Color fromCount(std::size_t count)
{
    double r = 0, g = 0, b = 0;
    __HSV2RGB__((double) ((count * 77) % 360), 1.0, 1.0, r, g, b);
    return Color(r,g,b);
}

inline void fromCount(std::size_t count, double& r, double &g, double &b)
{
    __HSV2RGB__((double) ((count * 77) % 360), 1.0, 1.0, r, g, b);
}
}

#endif // COLOR_HPP
