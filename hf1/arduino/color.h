#ifndef COLOR_INCLUDED_
#define COLOR_INCLUDED_

#include <math.h>
#include "logger_interface.h"

class ColorHSV;

// A color in RGB space (Red, Green, Blue).
// Each component is between 0 (lowest intensity) to 1 (highest intensity).
class ColorRGB {
public:
  ColorRGB() : r_(0), g_(0), b_(0) {}
  ColorRGB(float r, float g, float b) : r_(r), g_(g), b_(b) {}

  static ColorRGB From(const ColorHSV &hsv);

  float red() const { return r_; }
  float green() const { return g_; }
  float blue() const { return b_; }

  // Clamps all components to [0, 1].
  ColorRGB Normalized() const;

private:
  float r_;
  float g_;
  float b_;
};

// A color in HSV space (Hue, Saturation, Value).
// Each component is between 0 to 1.
// For saturation, 0 is least colorful and 1 is most colorful.
// For value, 0 is least dark and 1 is most dark.
// For hue, it's the fraction of a turn around the color circle, where 
// red is at 0 degrees (h=0), yellow is at 60 degrees (h=1/6), green is at 120 degrees (h=1/3), 
// cyan is at 180 degrees (h=1/2), blue is at 240 degrees (h=2/3) and purple is at 300 degrees (h=5/6).
class ColorHSV {
public:
  ColorHSV() : h_(0), s_(0), v_(0) {}
  ColorHSV(float h, float s, float v) : h_(h), s_(s), v_(v) {}

  static ColorHSV From(const ColorRGB &rgb);

  float hue() const { return h_; }
  float saturation() const { return s_; }
  float value() const { return v_; }

  // Maps all components to [0, 1].
  // Hue: subtracts all complete turns around the color circle.
  // Saturation and value: clamps them to the range.
  ColorHSV Normalized() const;

private:
  float h_;
  float s_;
  float v_;
};

#endif  // COLOR_INCLUDED_