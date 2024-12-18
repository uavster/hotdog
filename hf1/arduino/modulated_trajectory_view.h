#ifndef MODULATED_TRAJECTORY_INCLUDED_
#define MODULATED_TRAJECTORY_INCLUDED_

#include "trajectory_view.h"

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
using EnvelopeWaypoint = Waypoint<EnvelopeTargetState>;
using EnvelopeTrajectoryView = TrajectoryView<EnvelopeTargetState>;

template<typename TState>
class ModulatedTrajectoryView : public TrajectoryViewInterface<TState> {
public:
  ModulatedTrajectoryView() 
    : carrier_(nullptr), modulator_(nullptr), envelope_(nullptr) {}

  // Returns the waypoint at the given index, after applying interpolation.
  virtual Waypoint<TState> GetWaypoint(float seconds) const override = 0;

  // Returns true if the trajectory is set to restart from the beginning some time after
  // reaching the end.
  bool IsLoopingEnabled() const override { return carrier().IsLoopingEnabled(); }

  // Returns the duration of one trajectory lap. 
  // If no looping is enabled, this is the time between the first and last waypoints.
  // If looping is enabled, this is the time above plus the time it takes to return to the 
  // starting waypoint.
  float LapDuration() const override { return carrier().LapDuration(); }

  const TrajectoryView<TState> &carrier() const { return *ASSERT_NOT_NULL(carrier_); }
  const TrajectoryView<TState> &modulator() const { return *ASSERT_NOT_NULL(modulator_); }
  const EnvelopeTrajectoryView &envelope() const { return *ASSERT_NOT_NULL(envelope_); }

  // Does not take ownsership of the pointees, which must outlive this object.
  ModulatedTrajectoryView &carrier(const TrajectoryView<TState> *carrier) { carrier_ = carrier; return *this; }
  ModulatedTrajectoryView &modulator(const TrajectoryView<TState> *modulator) { modulator_ = modulator; return *this; }
  ModulatedTrajectoryView &envelope(const EnvelopeTrajectoryView *envelope) { envelope_ = envelope; return *this; }

private:
  const TrajectoryView<TState> *carrier_;
  const TrajectoryView<TState> *modulator_;
  const EnvelopeTrajectoryView *envelope_;
};

#endif  // MODULATED_TRAJECTORY_INCLUDED_