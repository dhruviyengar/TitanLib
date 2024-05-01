#include "titanlib/chassis/point/point.hpp"
#include <cmath>
#include <vector>

namespace titanlib {

inline float avg(std::vector<float> values) {
  float count = 0;
  int size = 0;
  for (float value : values) {
    if (value != NAN) {
      count += value;
      size++;
    }
  }
  return count / size;
}

inline float constrainValue(float value, float min, float max) {
  if (value < min)
    value = min;
  if (value > max)
    value = max;
  return value;
}

inline float constrainAngle(float angle) {
  angle = fmod(angle, 360);
  if (angle < 0)
    angle += 360;
  return angle;
}

inline int sgn(float value) { return value >= 0 ? 1 : -1; }

inline int sgn(Point point) {
  float value = point.getX() * point.getY();
  return sgn(value);
}

inline float angleError(float angle1, float angle2) {
  float turnAngle = angle2 - angle1;
  if (turnAngle > 180 || turnAngle < -180) {
    turnAngle = -1 * sgn(turnAngle) * (360 - fabs(turnAngle));
  }
  return turnAngle;
}

inline Point lerp(Point point1, Point point2, float t) {
  Point diff = point2 - point1;
  return Point(point1.getX() + (diff.getX() * t),
               point1.getY() + (diff.getY() * t));
}

inline float heading_to_slope(float heading) {
  return tan((constrainAngle((heading - 90)) * -1) * (3.141592653 / 180));
}

inline float slope_to_heading(float slope) {
  return constrainAngle(((atan2(slope, 1) * (180 / 3.141592653)) - 90) * -1);
}

inline float getCurvature(Point position, Point target, float heading) {
  float m = heading_to_slope(heading);
  if (m == INFINITY || m == -INFINITY) {
    m = 99999999.0;
  }
  float x =
      ((m * powf(position.getX(), 2)) - (m * powf(target.getX(), 2)) -
       (m * powf(position.getY(), 2)) +
       (2 * m * position.getY() * target.getY()) -
       (m * powf(target.getY(), 2)) - (2 * position.getX() * position.getY()) +
       (2 * position.getX() * target.getY())) /
      ((2 * m * position.getX()) - (2 * m * target.getX()) -
       (2 * position.getY()) + (2 * target.getY()));
  float y =
      (2 * m * position.getX() * position.getY() -
       2 * m * target.getX() * position.getY() + powf(position.getX(), 2) -
       2 * position.getX() * target.getX() + powf(target.getX(), 2) -
       powf(position.getY(), 2) + powf(target.getY(), 2)) /
      (2 * m * position.getX() - 2 * m * target.getX() - 2 * position.getY() +
       2 * target.getY());
  Point center(x, y);
  return (1 / position.distance(center));
}

inline float getCrossTrackError(Point position, Point start, Point end) {
  Point mid = lerp(start, end, 0.5);
  Point vector = mid - position;
  Point tangent = end - start;
  return position.distance(mid) * sgn(vector) * sgn(tangent);
}

inline std::pair<float, float> ratioSpeedsToMax(float leftVel, float rightVel,
                                                float maxSpeed) {
  float ratio = 1;
  float velocity = 0;
  if (maxSpeed > 0) {
    velocity = fmax(leftVel, rightVel);
  } else if (maxSpeed < 0) {
    velocity = fmin(leftVel, rightVel);
  }
  ratio = velocity / maxSpeed;
  if (fabs(velocity) > fabs(maxSpeed)) {
    leftVel /= ratio;
    rightVel /= ratio;
  }
  return std::make_pair(leftVel, rightVel);
}
} // namespace titanlib
