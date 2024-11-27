#ifndef POINT_INCLUDED_
#define POINT_INCLUDED_

class Point {
  public:
    float x;
    float y;

    Point();
    Point(float x_, float y_);
    Point operator+(const Point &p) const;
    Point operator-(const Point &p) const;
    Point operator/(float d) const;
    Point operator*(float d) const;
    float norm() const;
    float DistanceFrom(const Point &p) const;
};

Point operator*(float k, const Point &p);

#endif  // POINT_INCLUDED_