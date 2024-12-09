#ifndef MODULATED_TRAJECTORY_INCLUDED_
#define MODULATED_TRAJECTORY_INCLUDED_

#include "trajectory_view.h"

class EnvelopeStateVars {
public:
  EnvelopeStateVars() : amplitude_(1.0f) {}
  EnvelopeStateVars(float amplitude) : amplitude_(amplitude) {}
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
  ModulatedTrajectoryView() {}
  ModulatedTrajectoryView(const TrajectoryView<TState> &carrier, 
                          const TrajectoryView<TState> &modulator, 
                          const EnvelopeTrajectoryView &envelope);

  // Returns the number of waypoints in the trajectory, after applying interpolation.
  int NumWaypoints() const override;

  // Returns the waypoint at the given index, after applying interpolation.
  virtual Waypoint<TState> GetWaypoint(int index) const = 0;

  // Returns true if the trajectory is set to restart from the beginning some time after
  // reaching the end.
  bool IsLoopingEnabled() const override;

  const TrajectoryView<TState> &carrier() const { return carrier_; }
  const TrajectoryView<TState> &modulator() const { return modulator_; }
  const EnvelopeTrajectoryView &envelope() const { return envelope_; }

private:
  TrajectoryView<TState> carrier_;
  TrajectoryView<TState> modulator_;
  EnvelopeTrajectoryView envelope_;
};

#include "modulated_trajectory_view.hh"

#endif  // MODULATED_TRAJECTORY_INCLUDED_