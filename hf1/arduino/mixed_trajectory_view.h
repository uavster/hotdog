#ifndef MIXED_TRAJECTORY_INCLUDED_
#define MIXED_TRAJECTORY_INCLUDED_

#include "trajectory_view.h"

template<typename TState>
class MixedTrajectoryView : public TrajectoryViewInterface<TState> {
public:
  MixedTrajectoryView() 
    : trajectory1_(NULL), trajectory2_(NULL), alpha_(NULL) {}

  Waypoint<TState> GetWaypoint(float seconds) const override {
    const auto factor = alpha().state(seconds).location().amplitude();
    return Waypoint<TState>(seconds, trajectory1().state(seconds) * (1 - factor) + trajectory2().state(seconds) * factor);
  }

  bool IsLoopingEnabled() const override { 
    return trajectory1().IsLoopingEnabled() || trajectory2().IsLoopingEnabled();
  }

  float LapDuration() const override { 
    return std::max(trajectory1().LapDuration(), trajectory2().LapDuration());
  }

  const TrajectoryViewInterface<TState> &trajectory1() const { return *ASSERT_NOT_NULL(trajectory1_); }
  const TrajectoryViewInterface<TState> &trajectory2() const { return *ASSERT_NOT_NULL(trajectory2_); }
  const EnvelopeTrajectoryView &alpha() const { return *ASSERT_NOT_NULL(alpha_); }

  // Does not take ownsership of the pointees, which must outlive this object.
  MixedTrajectoryView &trajectory1(const TrajectoryViewInterface<TState> *trajectory1) { trajectory1_ = trajectory1; return *this; }
  MixedTrajectoryView &trajectory2(const TrajectoryViewInterface<TState> *trajectory2) { trajectory2_ = trajectory2; return *this; }
  MixedTrajectoryView &alpha(const EnvelopeTrajectoryView *alpha) { alpha_ = alpha; return *this; }

private:
  const TrajectoryViewInterface<TState> *trajectory1_;
  const TrajectoryViewInterface<TState> *trajectory2_;
  const EnvelopeTrajectoryView *alpha_;
};

#endif  // MIXED_TRAJECTORY_INCLUDED_