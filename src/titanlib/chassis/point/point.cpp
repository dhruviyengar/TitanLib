#include "titanlib/chassis/point/point.hpp"
#include "titanlib/chassis/util/util.hpp"
#include <string>

namespace titanlib
{

    Point::Point() {
        this->x = 0;
        this->y = 0;
    }

    Point::Point(float x, float y)
    {
        this->x = x;
        this->y = y;
    }

    float Point::distance(Point other)
    {
        float xDiff = other.x - x;
        float yDiff = other.y - y;
        return sqrt(xDiff * xDiff + yDiff * yDiff);
    }

    float Point::angle(Point other)
    {
        float xDiff = other.x - x;
        float yDiff = other.y - y;
        return constrainAngle(atan2(xDiff, yDiff) * (180 / M_PI));
    }

    float Point::getX() {
        return x;
    }

    float Point::getY() {
        return y;
    }

    float Point::magnitude() {
        return sqrt(x * x + y * y);
    }

    Point Point::normalize() {
        return Point(x / magnitude(), y / magnitude());
    }

    Point Point::operator+(Point other) {
        return Point(x + other.x, y + other.y);
    }

    Point Point::operator-(Point other) {
        return Point(x - other.x, y - other.y);
    }

    Point Point::operator*(Point other) {
        return Point(x * other.x, y * other.y);
    }

    Point Point::operator/(Point other) {
        return Point(x / other.x, y / other.y);
    }

    Point Point::operator*(float scalar) {
        return Point(x * scalar, y * scalar);
    }

    Point Point::operator/(float scalar) {
        return Point(x / scalar, y / scalar);
    }

    float Point::dotProduct(Point other) {
        return x * other.x + y * other.y;
    }

    std::string Point::asString() {
        return std::to_string(getX()) + ", " + std::to_string(getY());
    }
    
}