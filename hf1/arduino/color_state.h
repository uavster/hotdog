#ifndef COLOR_STATE_INCLUDED_
#define COLOR_STATE_INCLUDED_

#include "state.h"
#include "point.h"
#include "color.h"
#include <math.h>

class ColorRGBStateVars {
public:
  ColorRGBStateVars() {}
  ColorRGBStateVars(const ColorRGB &rgb)
    : rgb_(rgb) {}

  const ColorRGB &rgb() const {
    return rgb_;
  }

  ColorRGBStateVars operator+(const ColorRGBStateVars &other) const {
    return ColorRGBStateVars(ColorRGB(
      rgb_.red() + other.rgb_.red(),
      rgb_.green() + other.rgb_.green(),
      rgb_.blue() + other.rgb_.blue()));
  }

  ColorRGBStateVars operator*(float factor) const {
    return ColorRGBStateVars(ColorRGB(
      rgb_.red() * factor,
      rgb_.green() * factor,
      rgb_.blue() * factor));
  }

  float DistanceFrom(const ColorRGBStateVars &state) const {
    const float dr = rgb_.red() - state.rgb_.red();
    const float dg = rgb_.green() - state.rgb_.green();
    const float db = rgb_.blue() - state.rgb_.blue();
    return sqrtf(dr * dr + dg * dg + db * db);
  }

private:
  ColorRGB rgb_;
};

// Target state of an RGB color controller.
using ColorRGBTargetState = State<ColorRGBStateVars, /*Order=*/0>;

class ColorHSVStateVars {
public:
  ColorHSVStateVars() {}
  ColorHSVStateVars(const ColorHSV &hsv)
    : sin_h_(sinf(hsv.hue() * 2 * M_PI)), cos_h_(cosf(hsv.hue() * 2 * M_PI)), s_(hsv.saturation()), v_(hsv.value()) {}
  ColorHSVStateVars(float sin_h, float cos_h, float s, float v)
    : sin_h_(sin_h), cos_h_(cos_h), s_(s), v_(v) {}

  ColorHSV hsv() const {
    return ColorHSV(atan2f(sin_h_, cos_h_) / (2 * M_PI), s_, v_).Normalized();
  }

  ColorHSVStateVars operator+(const ColorHSVStateVars &other) const {
    return ColorHSVStateVars(
      sin_h_ + other.sin_h_,
      cos_h_ + other.cos_h_,
      s_ + other.s_,
      v_ + other.v_);
  }

  ColorHSVStateVars operator*(float factor) const {
    return ColorHSVStateVars(
      sin_h_ * factor,
      cos_h_ * factor,
      s_ * factor,
      v_ * factor);
  }

  float DistanceFrom(const ColorHSVStateVars &state) const {
    // For the hue component, take the shortest distance in the circle, including the path that crosses the 0 angle.
    const ColorHSV hsv = this->hsv();
    const ColorHSV other_hsv = state.hsv();
    const float dh = fminf(fabsf(hsv.hue() - other_hsv.hue()), fabsf(hsv.hue() - other_hsv.hue() - 1.0f));
    const float ds = hsv.saturation() - other_hsv.saturation();
    const float dv = hsv.value() - other_hsv.value();
    return sqrtf(dh * dh + ds * ds + dv * dv);
  }

private:
  float sin_h_;
  float cos_h_;
  float s_;
  float v_;
};

// Target state of an HSV color controller.
using ColorHSVTargetState = State<ColorHSVStateVars, /*Order=*/0>;

#endif  // COLOR_STATE_INCLUDED_