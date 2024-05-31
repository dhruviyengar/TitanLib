#include "titanlib/chassis/bezier/bezier.hpp"
#include "titanlib/chassis/util/util.hpp"
#include <cmath>

namespace titanlib {

CubicBezier::CubicBezier() {
  this->start = Point(0, 0);
  this->end = Point(0, 0);
  this->startControl = Point(0, 0);
  this->endControl = Point(0, 0);
}

CubicBezier::CubicBezier(Point start, Point end, Point startControl,
                         Point endControl) {
  this->start = start;
  this->end = end;
  this->startControl = startControl;
  this->endControl = endControl;
}

Point CubicBezier::getStart() {
  return start;
}

Point CubicBezier::getStartControl() {
  return startControl;
}

Point CubicBezier::getEndControl() {
  return endControl;
}

Point CubicBezier::getEnd() {
  return end;
}

Point CubicBezier::getPoint(float t) {
  if (t == 0) {
    return start;
  } else if (t == 1) {
    return end;
  } else {
    float x = (powf(1 - t, 3) * start.getX()) +
              (3 * t * powf(1 - t, 2) * startControl.getX()) +
              (3 * powf(t, 2) * (1 - t) * endControl.getX()) +
              (powf(t, 3) * end.getX());
    float y = (powf(1 - t, 3) * start.getY()) +
              (3 * t * powf(1 - t, 2) * startControl.getY()) +
              (3 * powf(t, 2) * (1 - t) * endControl.getY()) +
              (powf(t, 3) * end.getY());
    return Point(x, y);
  }
}

float CubicBezier::getYDerivative(float t) {
  return (-3 * powf(1 - t, 2) * start.getY()) +
         (3 * powf(1 - t, 2) * startControl.getY()) +
         (-6 * t * (1 - t) * startControl.getY()) +
         (-3 * powf(t, 2) * endControl.getY()) +
         (6 * t * (1 - t) * endControl.getY()) + (3 * powf(t, 2) * end.getY());
}

float CubicBezier::getXDerivative(float t) {
  return (-3 * powf(1 - t, 2) * start.getX()) +
         (3 * powf(1 - t, 2) * startControl.getX()) +
         (-6 * t * (1 - t) * startControl.getX()) +
         (-3 * powf(t, 2) * endControl.getX()) +
         (6 * t * (1 - t) * endControl.getX()) + (3 * powf(t, 2) * end.getX());
}

float CubicBezier::getYSecondDerivative(float t) {
  return (6 * (1 - t) * start.getY()) - (6 * (1 - t) * startControl.getY()) -
         ((6 - (12 * t)) * startControl.getY()) - (6 * t * endControl.getY()) +
         ((6 - (12 * t)) * endControl.getY()) + (6 * t * end.getY());
}

float CubicBezier::getXSecondDerivative(float t) {
  return (6 * (1 - t) * start.getX()) - (6 * (1 - t) * startControl.getX()) -
         ((6 - (12 * t)) * startControl.getX()) - (6 * t * endControl.getX()) +
         ((6 - (12 * t)) * endControl.getX()) + (6 * t * end.getX());
}

float CubicBezier::getSlope(float t) {
  return getYDerivative(t) / getXDerivative(t);
}

float CubicBezier::getHeading(float t) {
  float m = getSlope(t);
  if (fabs(m) == INFINITY || fabs(m) == NAN) {
    if (getYDerivative(t) > 0) {
      return 0;
    } else {
      return 180;
    }
  }
  float heading = slopeToHeading(m);
  if (sgn(getXDerivative(t)) < 0) {
    heading = constrainAngle(heading - 180);
  }
  return heading;
}

float CubicBezier::firstDistanceDerivative(Point point, float t) {
  Point diff = getPoint(t) - point;
  return (2 * (diff.getX()) * getXDerivative(t)) +
         (2 * (diff.getY()) * getYDerivative(t));
}

float CubicBezier::secondDistanceDerivative(Point point, float t) {
  Point diff = getPoint(t) - point;
  return 2 *
         ((getXSecondDerivative(t) * diff.getX()) + powf(getXDerivative(t), 2) +
          (getYSecondDerivative(t) * diff.getY()) + powf(getYDerivative(t), 2));
}

float CubicBezier::getTFromArc(float arc, float tolerance) {
  float min = 0;
  float max = 1;
  while (min <= max) {
    float mid = min + max / 2;
    if (fabs(arcLength(0, mid, true) - arc) < tolerance) {
      return arc;
    } else if (arcLength(0, mid, true) < arc) {
      min = mid;
    } else if (arcLength(0, mid, true) > arc) {
      max = mid;
    }
  }
  return -1;
}

float CubicBezier::closestPoint(Point point, float initialGuess,
                                float tolerance) {
  float t = initialGuess;
  float prevChange = 0;
  while (getPoint(t).distance(point) > tolerance) { // tolerance
    float d = firstDistanceDerivative(point, t);
    float dPrime = secondDistanceDerivative(point, t);
    float change = d / dPrime;
    if (fabs(change - prevChange) < tolerance) {
      break;
    }
    t -= change;
    if (t < 0) {
      t = 0;
      break;
    }
    prevChange = change;
  }
  return t;
}

// Legendre-Gauss approximation
float CubicBezier::arcLength(float startT, float endT, bool useLegendre) {
  auto integrand = [this](double t) -> double {
        return std::sqrt(std::pow(getXDerivative(t), 2) + std::pow(getYDerivative(t), 2));
  };
  if (useLegendre) {
    return legendreGaussIntegral(integrand, startT, endT);
  } else {
    return trapezoidalSumIntegral(integrand, startT, endT, 10);
  }
}

float CubicBezier::getCurvature(float t) {
  float derivX = getXDerivative(t);
  float derivY = getYDerivative(t);
  float secDerivX = getXSecondDerivative(t);
  float secDerivY = getYSecondDerivative(t);
  float num = (derivX * secDerivY) - (secDerivX * derivY);
  float dem = powf((derivX * derivX) + (derivY * derivY), 1.5);
  return num / dem;
}

CubicBezier CubicBezier::deCasteljau(float t) {
  Point q0 = Point((1 - t) * start.getX() + t * startControl.getX(), (1 - t) * start.getY() + t * startControl.getY());
  Point q1 = Point((1 - t) * startControl.getX() + t * endControl.getX(), (1 - t) * startControl.getY() + t * endControl.getY());
  Point q2 = Point((1 - t) * endControl.getX() + t * end.getX(), (1 - t) * endControl.getY() + t * end.getY());
  Point r0 = Point((1 - t) * q0.getX() + t * q1.getX(), (1 - t) * q0.getY() + t * q1.getY());
  Point r1 = Point((1 - t) * q1.getX() + t * q2.getX(), (1 - t) * q1.getY() + t * q2.getY());
  Point s = Point((1 - t) * r0.getX() + t * r1.getX(), (1 - t) * r0.getY() + t * r1.getY());
  return CubicBezier(s, end, r1, q2);
}

} // namespace titanlib