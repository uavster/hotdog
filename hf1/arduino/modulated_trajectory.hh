template<typename TState>
ModulatedTrajectoryView<TState>::ModulatedTrajectoryView(
  const TrajectoryView<TState> &carrier, 
  const TrajectoryView<TState> &modulator, 
  const EnvelopeTrajectoryView &envelope)
  : carrier_(carrier),
    modulator_(modulator),
    envelope_(envelope) {
}

template<typename TState>
int ModulatedTrajectoryView<TState>::NumWaypoints() const {
  return carrier_.NumWaypoints();
}

template<typename TState>
bool ModulatedTrajectoryView<TState>::IsLoopingEnabled() const {
  return carrier_.IsLoopingEnabled();
}
