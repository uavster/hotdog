#include <math.h>

template<int NDims> 
Vector<NDims>::Vector() {
  static_assert(sizeof(components_) / sizeof(components_[0]) == NDims);
  for (int i = 0; i < NDims; ++i) {
    components_[i] = 0;
  }
}

template<int NDims> 
Vector<NDims>::Vector(const float (&components)[NDims]) { 
  for (int i = 0; i < NDims; ++i) { 
    components_[i] = components[i];
  } 
}

template<int NDims> 
template<typename... Args> void Vector<NDims>::Fill(ArgIndex index, float component1, Args... components) {
  components_[index.index] = component1;
  Fill(ArgIndex{ .index = index.index + 1 }, components...);
}

template<int NDims> 
Vector<NDims> Vector<NDims>::operator+(const Vector<NDims> &p) const {
  Vector<NDims> result = *this;
  for (int i = 0; i < NDims; ++i) {
    result.components_[i] += p.components_[i];
  }
  return result;
}

template<int NDims> 
Vector<NDims> Vector<NDims>::operator-(const Vector<NDims> &p) const {
  Vector<NDims> result = *this;
  for (int i = 0; i < NDims; ++i) {
    result.components_[i] -= p.components_[i];
  }
  return result;
}

template<int NDims> 
Vector<NDims> Vector<NDims>::operator/(float d) const {
  Vector<NDims> result = *this;
  for (int i = 0; i < NDims; ++i) {
    result.components_[i] /= d;
  }
  return result;
}

template<int NDims> 
Vector<NDims> Vector<NDims>::operator*(float d) const {
  Vector<NDims> result = *this;
  for (int i = 0; i < NDims; ++i) {
    result.components_[i] *= d;
  }
  return result;
}

template<int NDims> 
float Vector<NDims>::Norm() const {
  float accum = 0;
  for (int i = 0; NDims; ++i) {
    accum += components_[i] * components_[i];
  }
  return sqrtf(accum);
}

template<int NDims> 
float Vector<NDims>::DistanceFrom(const Vector<NDims> &p) const {
  return (*this - p).Norm();
}
