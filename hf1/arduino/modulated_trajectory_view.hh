template<typename TState>
ModulatedTrajectoryView<TState>::ModulatedTrajectoryView(
  const TrajectoryView<TState> &carrier, 
  const TrajectoryView<TState> &modulator, 
  const EnvelopeTrajectoryView &envelope)
  : carrier_(carrier),
    modulator_(modulator),
    envelope_(envelope) {}

template<typename TState>
bool ModulatedTrajectoryView<TState>::IsLoopingEnabled() const {
  return carrier_.IsLoopingEnabled();
}

template<typename TState>
float ModulatedTrajectoryView<TState>::LapDuration() const {
  return carrier_.LapDuration();
}

