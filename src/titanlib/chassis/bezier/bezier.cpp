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
  return slopeToHeading(getSlope(t)) * getXDerivative(t);
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
    if (fabs(arcLength(0, mid) - arc) < tolerance) {
      return arc;
    } else if (arcLength(0, mid) < arc) {
      min = mid;
    } else if (arcLength(0, mid) > arc) {
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
float CubicBezier::arcLength(float startT, float endT) {
  std::vector<std::vector<float>> coefficients{
      {0.1279381953467522, -0.0640568928626056},
      {0.1279381953467522, 0.0640568928626056},
      {0.1258374563468283, -0.1911188674736163},
      {0.1258374563468283, 0.1911188674736163},
      {0.1216704729278034, -0.3150426796961634},
      {0.1216704729278034, 0.3150426796961634},
      {0.1155056680537256, -0.4337935076260451},
      {0.1155056680537256, 0.4337935076260451},
      {0.1074442701159656, -0.5454214713888396},
      {0.1074442701159656, 0.5454214713888396},
      {0.0976186521041139, -0.6480936519369755},
      {0.0976186521041139, 0.6480936519369755},
      {0.0861901615319533, -0.7401241915785544},
      {0.0861901615319533, 0.7401241915785544},
      {0.0733464814110803, -0.8200019859739029},
      {0.0733464814110803, 0.8200019859739029},
      {0.0592985849154368, -0.8864155270044011},
      {0.0592985849154368, 0.8864155270044011},
      {0.0442774388174198, -0.9382745520027328},
      {0.0442774388174198, 0.9382745520027328},
      {0.0285313886289337, -0.9747285559713095},
      {0.0285313886289337, 0.9747285559713095},
      {0.0123412297999872, -0.9951872199970213},
      {0.0123412297999872, 0.9951872199970213}};
  float length = 0;
  for (int i = 0; i < coefficients.size(); i++) {
    float t = 0.5 * (coefficients[i][1] + 1) * (endT - startT) + startT;
    length += coefficients[i][0] * ((endT - startT) / 2) *
              sqrt((powf(getXDerivative(t), 2) + powf(getYDerivative(t), 2)));
  }
  return length;
}

float CubicBezier::getCurvature(float t) {
  float derivX = getXDerivative(t);
  float derivY = getYDerivative(t);
  float secDerivX = getXSecondDerivative(t);
  float secDerivY = getYSecondDerivative(t);
  float num = derivX * secDerivY - secDerivX * derivY;
  float dem = powf((derivX * derivX) + (derivY * derivY), 1.5);
  return num / dem;
}

} // namespace titanlib