#pragma once

#include <cmath>
#include <string>

namespace titanlib {

class Point {
public:
  Point(float x, float y);
  float distance(Point other);
  float angle(Point other);
  float getX();
  float getY();
  float magnitude();
  Point normalize();
  Point operator+(Point other);
  Point operator-(Point other);
  Point operator*(Point other);
  Point operator/(Point other);
  Point operator*(float scalar);
  Point operator/(float scalar);
  float dotProduct(Point other);
  std::string asString();

private:
  float x;
  float y;
};
} // namespace titanlib