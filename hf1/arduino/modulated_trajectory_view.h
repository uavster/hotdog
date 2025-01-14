#ifndef MODULATED_TRAJECTORY_INCLUDED_
#define MODULATED_TRAJECTORY_INCLUDED_

#include "trajectory_view.h"
#include "envelope_trajectory.h"

template<typename TState>
class ModulatedTrajectoryView : public TrajectoryViewInterface<TState> {
public:
  ModulatedTrajectoryView() 
    : carrier_(NULL), modulator_(NULL), envelope_(NULL) {}

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

  const TrajectoryViewInterface<TState> &carrier() const { return *ASSERT_NOT_NULL(carrier_); }
  const TrajectoryViewInterface<TState> &modulator() const { return *ASSERT_NOT_NULL(modulator_); }
  const EnvelopeTrajectoryView &envelope() const { return *ASSERT_NOT_NULL(envelope_); }

  // Does not take ownsership of the pointees, which must outlive this object.
  ModulatedTrajectoryView &carrier(const TrajectoryViewInterface<TState> *carrier) { carrier_ = carrier; return *this; }
  ModulatedTrajectoryView &modulator(const TrajectoryViewInterface<TState> *modulator) { modulator_ = modulator; return *this; }
  ModulatedTrajectoryView &envelope(const EnvelopeTrajectoryView *envelope) { envelope_ = envelope; return *this; }

private:
  const TrajectoryViewInterface<TState> *carrier_;
  const TrajectoryViewInterface<TState> *modulator_;
  const EnvelopeTrajectoryView *envelope_;
};

#endif  // MODULATED_TRAJECTORY_INCLUDED_