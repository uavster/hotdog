#ifndef VECTOR_INCLUDED_
#define VECTOR_INCLUDED_

template<int NDims> class Vector {
public:
  static_assert(NDims > 0);
  Vector();
  Vector(const float (&components)[NDims]);
  template<typename... Args> Vector(float component1, Args... components) {
    Fill(ArgIndex{ .index = 0 }, component1, components...);
  }

  float operator[](int index) const { return components_[index]; }
  float x() const { return components_[0]; }
  float y() const { return components_[1]; }
  float z() const { return components_[2]; }

  Vector<NDims> operator+(const Vector<NDims> &p) const;
  Vector<NDims> operator-(const Vector<NDims> &p) const;
  Vector<NDims> operator/(float d) const;
  Vector<NDims> operator*(float d) const;

  float Norm() const;
  float DistanceFrom(const Vector<NDims> &p) const;

private:
  struct ArgIndex { int index; };
  template<typename... Args> void Fill(ArgIndex index, float component1, Args... components);
  void Fill(ArgIndex index) {}

  float components_[NDims];
};

#include "vector.hh"

#endif  // VECTOR_INCLUDED_