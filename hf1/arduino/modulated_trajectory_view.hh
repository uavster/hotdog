template<typename TState>
ModulatedTrajectoryView<TState>::ModulatedTrajectoryView(
  const TrajectoryView<TState> &carrier, 
  const TrajectoryView<TState> &modulator, 
  const EnvelopeTrajectoryView &envelope)
  : carrier_(carrier),
    modulator_(modulator),
    envelope_(envelope) {
  // All signals must be resampled with the same number of points than the carrier.
  const auto period = carrier_.interpolation_config().sampling_period_seconds;
  modulator_.EnableInterpolation(InterpolationConfig{ .type = InterpolationType::kLinear, .sampling_period_seconds = period });
  envelope_.EnableInterpolation(InterpolationConfig{ .type = InterpolationType::kLinear, .sampling_period_seconds = period });
}

template<typename TState>
int ModulatedTrajectoryView<TState>::NumWaypoints() const {
  return carrier_.NumWaypoints();
}

template<typename TState>
bool ModulatedTrajectoryView<TState>::IsLoopingEnabled() const {
  return carrier_.IsLoopingEnabled();
}
