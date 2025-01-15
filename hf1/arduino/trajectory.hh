#include "logger_interface.h"

template<typename TState, int Capacity>
Trajectory<TState, Capacity>::Trajectory(int num_waypoints, const Waypoint<TState> *waypoints)
  : size_(0) {
  for (int i = 0; i < num_waypoints; ++i) {
    Insert(waypoints[i]);
  }
  // Ensure no two points exist for the same time.
  for (int i = 0; i < size_ - 1; ++i) {
    ASSERT(waypoints_[i].seconds() < waypoints_[i + 1].seconds());
  }
}

template<typename TState, int Capacity>
template<int Size> Trajectory<TState, Capacity>::Trajectory(const Waypoint<TState> (&waypoints)[Size]) 
  : Trajectory(Size, waypoints) {}

template<typename TState, int Capacity>
const Waypoint<TState> &Trajectory<TState, Capacity>::operator[](int i) const {
  ASSERT(i >= 0 && i < size_);
  return waypoints_[i];
}

template<typename TState, int Capacity>
int Trajectory<TState, Capacity>::FindInsertionIndex(float seconds, int start_index, int end_index) const {
  if (start_index >= end_index) {
    return seconds < waypoints_[start_index].seconds() ? start_index : start_index + 1;
  }
  const int middle_index = (start_index + end_index) / 2;
  if (seconds < waypoints_[middle_index].seconds()) {
    return FindInsertionIndex(seconds, start_index, middle_index);
  } else {
    return FindInsertionIndex(seconds, middle_index + 1, end_index);
  }
}

template<typename TState, int Capacity>
void Trajectory<TState, Capacity>::Insert(const Waypoint<TState> &waypoint) {
  ASSERT(size_ < Capacity);
  if (size_ == 0) {
    waypoints_[0] = waypoint;
  }
  // Find insertion point with a binary search.
  // All existing elements must be sorted by time.
  const int insertion_index = FindInsertionIndex(waypoint.seconds(), 0, size_ - 1);
  // Shift all waypoints after the insertion point to the right.
  for (int i = 0; i < size_ - insertion_index; ++i) {
    waypoints_[size_ - i] = waypoints_[size_ - i - 1];
  }
  // Insert new waypoint.
  waypoints_[insertion_index] = waypoint;
  ++size_;
}

template<typename TState, int Capacity>
void Trajectory<TState, Capacity>::Clear() {
  size_ = 0;
}

template<typename TState, int Capacity>
int Trajectory<TState, Capacity>::FindWaypointAtOrBeforeSeconds(TimerSecondsType seconds) const {
  if (size_ == 0) {
    return -1;
  }
  return FindInsertionIndex(seconds, 0, size_ - 1) - 1;
}

