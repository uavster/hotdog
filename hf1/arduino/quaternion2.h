#include <cmath>

double sine(double a) { return sin(a); }
float sine(float a) { return sinf(a); }
float sine(int a) { return sine(static_cast<float>(a)); }

double cosine(double a) { return cos(a); }
float cosine(float a) { return cosf(a); }
float cosine(int a) { return cosine(static_cast<float>(a)); }

double arc_cosine(double a) { return acos(a); }
float arc_cosine(float a) { return acosf(a); }
float arc_cosine(int a) { return arc_cosine(static_cast<float>(a)); }

double square_root(double a) { return sqrt(a); }
float square_root(float a) { return sqrtf(a); }
float square_root(int a) { return sqrtf(a); }

template<typename T> class Quaternion2 {
public:
  Quaternion2() : sin_a_(0), cos_a_(1) {}
  explicit Quaternion2(T angle) : sin_a_(sine(angle)), cos_a_(cosine(angle)) {}

  Quaternion2 operator+(const Quaternion2 &q) const {
    return Quaternion2(sin_a_ + q.sin_a_, cos_a_ + q.cos_a_);
  }

  Quaternion2 operator*(T k) const {
    return Quaternion2(sin_a_ * k, cos_a_ * k);
  }

  T angle() const {
    return std::atan2(sin_a_, cos_a_);
  }

  T sin_angle() const { return sin_a_; }
  T cos_angle() const { return cos_a_; }
  T tan_angle() const { return sin_a_ / cos_a_; }

  T norm() const { return square_root(sin_a_ * sin_a_ + cos_a_ * cos_a_); }

private:
  Quaternion2(T sin_a, T cos_a) : sin_a_(sin_a), cos_a_(cos_a) {}

  T sin_a_;
  T cos_a_;
};

template<typename T> Quaternion2<T> operator*(T k, const Quaternion2<T> &q) { return q * k; }

template<typename T> Quaternion2<T> UnnormalizedLerp(const Quaternion2<T> &q1, const Quaternion2<T> &q2, T t) {
  return (1 - t) * q1 + t * q2;
}

template<typename T> Quaternion2<T> Lerp(const Quaternion2<T> &q1, const Quaternion2<T> &q2, T t) {
  const Quaternion2<T> q = UnnormalizedLerp(q1, q2, t);
  return q * (static_cast<T>(1) / q.norm());
}

template<typename T> Quaternion2<T> Slerp(const Quaternion2<T> &q1, const Quaternion2<T> &q2, T t) {
  const T angle_betwen_quats = arc_cosine(q1.cos_angle() * q2.cos_angle() + q1.sin_angle() * q2.sin_angle());
  const T sin_angle_betwen_quats = sine(sin_angle_betwen_quats);
  if (sin_angle_betwen_quats > 1e-3) {
    const T sing_angle_between_quats = sine(angle_betwen_quats);
    return (sine((1 - t) * angle_betwen_quats) * q1 + sine(t * angle_betwen_quats) * q2) / sine(angle_betwen_quats);
  } else {
    return Lerp(q1, q2, t);
  }
}

using Quaternion2f = Quaternion2<float>;
using Quaternion2d = Quaternion2<double>;