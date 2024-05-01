#pragma once

#include "titanlib/chassis/point/point.hpp"
#include <vector>

namespace titanlib {

class CubicBezier {
public:
  CubicBezier();
  CubicBezier(Point start, Point end, Point startControl, Point endControl);
  Point getPoint(float t);
  float getYDerivative(float t);
  float getXDerivative(float t);
  float getYSecondDerivative(float t);
  float getXSecondDerivative(float t);
  float firstDistanceDerivative(Point point, float t);
  float secondDistanceDerivative(Point point, float t);
  float getSlope(float t);
  float closestPoint(Point point, float initialGuess, float tolerance);
  float arcLength(float startT, float endT);

private:
  Point start = Point(0, 0);
  Point end = Point(0, 0);
  Point startControl = Point(0, 0);
  Point endControl = Point(0, 0);
};

} // namespace titanlib
