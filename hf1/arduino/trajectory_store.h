#ifndef TRAJECTORY_STORE_
#define TRAJECTORY_STORE_

#include "store.h"
#include "base_state.h"
#include "head_state.h"
#include "trajectory.h"
#include "trajectory_view.h"
#include "mixed_trajectory_view.h"
#include "base_trajectory.h"
#include "head_trajectory.h"
#include "p2p_application_protocol.h"

template<int MaxNumTrajectoriesPerType, int MaxNumWaypointsPerTrajectory> 
class TrajectoryStore_ {
public:
  Store<Trajectory<BaseTargetState, MaxNumWaypointsPerTrajectory>, MaxNumTrajectoriesPerType> &base_trajectories() { return base_trajectories_; }
  Store<Trajectory<HeadTargetState, MaxNumWaypointsPerTrajectory>, MaxNumTrajectoriesPerType> &head_trajectories() { return head_trajectories_; }
  Store<Trajectory<EnvelopeTargetState, MaxNumWaypointsPerTrajectory>, MaxNumTrajectoriesPerType> &envelope_trajectories() { return envelope_trajectories_; }

  Store<TrajectoryView<BaseTargetState>, MaxNumTrajectoriesPerType> &base_trajectory_views() { return base_trajectory_views_; };
  Store<TrajectoryView<HeadTargetState>, MaxNumTrajectoriesPerType> &head_trajectory_views() { return head_trajectory_views_; }
  Store<TrajectoryView<EnvelopeTargetState>, MaxNumTrajectoriesPerType> &envelope_trajectory_views() { return envelope_trajectory_views_; }

  Store<BaseModulatedTrajectoryView, MaxNumTrajectoriesPerType> &base_modulated_trajectory_views() { return base_modulated_trajectory_views_; };
  Store<HeadModulatedTrajectoryView, MaxNumTrajectoriesPerType> &head_modulated_trajectory_views() { return head_modulated_trajectory_views_; }

  Store<MixedTrajectoryView<BaseTargetState>, MaxNumTrajectoriesPerType> &base_mixed_trajectory_views() { return base_mixed_trajectory_views_; };
  Store<MixedTrajectoryView<HeadTargetState>, MaxNumTrajectoriesPerType> &head_mixed_trajectory_views() { return head_mixed_trajectory_views_; }

private:
  Store<Trajectory<BaseTargetState, MaxNumWaypointsPerTrajectory>, MaxNumTrajectoriesPerType> base_trajectories_;
  Store<Trajectory<HeadTargetState, MaxNumWaypointsPerTrajectory>, MaxNumTrajectoriesPerType> head_trajectories_;
  Store<Trajectory<EnvelopeTargetState, MaxNumWaypointsPerTrajectory>, MaxNumTrajectoriesPerType> envelope_trajectories_;

  Store<TrajectoryView<BaseTargetState>, MaxNumTrajectoriesPerType> base_trajectory_views_;
  Store<TrajectoryView<HeadTargetState>, MaxNumTrajectoriesPerType> head_trajectory_views_;
  Store<TrajectoryView<EnvelopeTargetState>, MaxNumTrajectoriesPerType> envelope_trajectory_views_;

  Store<BaseModulatedTrajectoryView, MaxNumTrajectoriesPerType> base_modulated_trajectory_views_;
  Store<HeadModulatedTrajectoryView, MaxNumTrajectoriesPerType> head_modulated_trajectory_views_;

  Store<MixedTrajectoryView<BaseTargetState>, MaxNumTrajectoriesPerType> base_mixed_trajectory_views_;
  Store<MixedTrajectoryView<HeadTargetState>, MaxNumTrajectoriesPerType> head_mixed_trajectory_views_;
};

using TrajectoryStore = TrajectoryStore_</*MaxNumTrajectoriesPerType=*/16, kP2PMaxNumWaypointsPerTrajectory>;

#endif