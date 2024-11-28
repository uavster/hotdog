#include "point.h"
#include <math.h>

Point::Point() : x(0), y(0) {}

Point::Point(float x_, float y_) : x(x_), y(y_) {}

Point Point::operator+(const Point &p) const { 
  return Point(x + p.x, y + p.y);
}

Point Point::operator-(const Point &p) const { 
  return Point(x - p.x, y - p.y); 
}

Point Point::operator/(float d) const { 
  return Point(x / d, y / d); 
}

Point Point::operator*(float d) const { 
  return Point(x * d, y * d); \
}

float Point::norm() const { 
  return sqrtf(x * x + y * y); 
}

Point operator*(float k, const Point &p) { 
  return Point(k * p.x, k * p.y); 
}

float Point::DistanceFrom(const Point &p) const {
  return ((*this) - p).norm();
}
