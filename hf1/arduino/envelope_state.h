#ifndef ENVELOPE_STATE_INCLUDED_
#define ENVELOPE_STATE_INCLUDED_

class EnvelopeStateVars {
public:
  EnvelopeStateVars() : amplitude_(1.0f) {}
  explicit EnvelopeStateVars(float amplitude) : amplitude_(amplitude) {}

  float amplitude() const { return amplitude_; }

  EnvelopeStateVars operator+(const EnvelopeStateVars &other) const {
    return EnvelopeStateVars(amplitude_ + other.amplitude_);
  }

  EnvelopeStateVars operator*(float factor) const {
    return EnvelopeStateVars(amplitude_ * factor);
  }

  float DistanceFrom(const EnvelopeStateVars &state) const {
    return amplitude_ - state.amplitude_;
  }

private:
  float amplitude_;
};

using EnvelopeTargetState = State<EnvelopeStateVars, 0>;

#endif  // ENVELOPE_STATE_INCLUDED_