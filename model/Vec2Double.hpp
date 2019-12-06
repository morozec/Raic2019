#ifndef _MODEL_VEC2_DOUBLE_HPP_
#define _MODEL_VEC2_DOUBLE_HPP_

#include "../Stream.hpp"
#include <string>

class Vec2Double {
public:
    double x;
    double y;
    Vec2Double();
    Vec2Double(double x, double y);
    static Vec2Double readFrom(InputStream& stream);
    void writeTo(OutputStream& stream) const;
    std::string toString() const;
};

inline Vec2Double operator+(const Vec2Double& lhs, const Vec2Double& rhs)
{
	return { lhs.x + rhs.x, lhs.y + rhs.y };
}

inline Vec2Double operator-(const Vec2Double& lhs, const Vec2Double& rhs)
{
	return { lhs.x - rhs.x, lhs.y - rhs.y };
}

inline Vec2Double operator*(const Vec2Double& lhs, double c)
{
	return { lhs.x * c, lhs.y * c };
}

#endif
