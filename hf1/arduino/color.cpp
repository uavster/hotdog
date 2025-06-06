#include <algorithm>
#include "color.h"

ColorRGB ColorRGB::From(const ColorHSV &hsv_raw) {
  const ColorHSV hsv = hsv_raw.Normalized();
  const float c = hsv.value() * hsv.saturation();  // Chroma.
  const float x = c * (1 - fabsf(fmodf(hsv.hue() * 6.0f, 2) - 1));
  const float m = hsv.value() - c;

  float r_prime, g_prime, b_prime;

  if (hsv.hue() < 1 / 6.0f) {
    r_prime = c; g_prime = x; b_prime = 0;
  } else if (hsv.hue() < 2 / 6.0f) {
    r_prime = x; g_prime = c; b_prime = 0;
  } else if (hsv.hue() < 3 / 6.0f) {
    r_prime = 0; g_prime = c; b_prime = x;
  } else if (hsv.hue() < 4 / 6.0f) {
    r_prime = 0; g_prime = x; b_prime = c;
  } else if (hsv.hue() < 5 / 6.0f) {
    r_prime = x; g_prime = 0; b_prime = c;
  } else {
    r_prime = c; g_prime = 0; b_prime = x;
  }

  return ColorRGB(r_prime + m, g_prime + m, b_prime + m);
}

ColorRGB ColorRGB::Normalized() const {
  return ColorRGB(std::clamp(r_, 0.0f, 1.0f), std::clamp(g_, 0.0f, 1.0f), std::clamp(b_, 0.0f, 1.0f));
}

ColorHSV ColorHSV::From(const ColorRGB &rgb_raw) {
  const ColorRGB rgb = rgb_raw.Normalized();

  const float r = rgb.red();
  const float g = rgb.green();
  const float b = rgb.blue();

  const float max = fmaxf(r, fmaxf(g, b));
  const float min = fminf(r, fminf(g, b));
  const float delta = max - min;

  // Value.
  float v = max;

  // Saturation.
  float s;
  if (max == 0.0f) {
      s = 0.0f;
  } else {
      s = delta / max;
  }

  // Hue.
  float h;
  if (delta == 0.0f) {
    h = 0.0f; // Undefined, achromatic gray.
  } else if (max == r) {
    h = 60.0f * fmodf(((g - b) / delta), 6.0f);
  } else if (max == g) {
    h = 60.0f * (((b - r) / delta) + 2.0f);
  } else {
    h = 60.0f * (((r - g) / delta) + 4.0f);
  }

  if (h < 0.0f) {
    h += 360.0f;
  }

  return ColorHSV(h, s, v);
}

ColorHSV ColorHSV::Normalized() const {
  // Map hue as an angle between 0 and 1, e.g. -1.25 -> 0.75.
  float h = remainderf(h_, 1);
  h = h < 0 ? h + 1 : h;

  return ColorHSV(h, std::clamp(s_, 0.0f, 1.0f), std::clamp(v_, 0.0f, 1.0f));
}